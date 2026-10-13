#pragma once
#include "sensors/health.h"
#include <Arduino.h>
#include <VL53L0X.h>
#include <Wire.h>
#include <freertos/task.h>

namespace sensor_policy {
inline SensorError wire_error(uint8_t rc) {
  return rc == 7                ? SensorError::SHORT_READ
         : rc == 5              ? SensorError::TIMEOUT
         : (rc == 2 || rc == 3) ? SensorError::NACK
                                : SensorError::BUS_ERROR;
}
// Hardware faults and range quality are deliberately separate. See fork
// provenance.
inline bool accept_range(SensorData &out, uint16_t raw, uint8_t status,
                         uint32_t now, uint16_t minimum, uint16_t maximum,
                         bool allow_no_return = false) {
  out.raw_range_mm = raw;
  out.raw_status = status;
  out.raw_timestamp_us = now;
  uint8_t code = (status & 0x78) >> 3;
  if (code >= 1 && code <= 3) {
    fail(out, SensorError::HARDWARE_ERROR);
    return false;
  }
  out.consecutive_errors = 0;
  // Observed on Sniegbaltite in open space: signal failure 4 / 8191 and
  // phase failure 6 / 8190. They prove a live result stream, not clear space.
  // Permit only these exact sentinel classes under an explicit racing policy;
  // preserve NO_RETURN quality rather than claiming a valid measurement.
  if (allow_no_return && (code == 4 || code == 6) &&
      (raw == 8190 || raw == 8191)) {
    out.distance_mm = maximum;
    out.timestamp_us = now;
    out.health = SensorHealth::NO_RETURN;
  } else if (code != 11 || raw >= 8190) {
    out.health = SensorHealth::RANGE_INVALID;
  } else {
    out.distance_mm = std::max(minimum, std::min(maximum, raw));
    success(out, now);
  }
  return true; // Complete non-hardware result, even if quality is unreliable.
}
struct ToFDevice {
  VL53L0X driver;
  SensorData data;
  int8_t xshut = -1;
  uint8_t address = 0x30;
  bool active = false, pending = true;
  uint32_t result_us = 0, grace_start_us = 0;
  bool grace_eligible = false;
  Backoff backoff;
};

// One instance belongs exclusively to one task (or a stopped C3 control loop).
class ToFBus {
public:
  using Publish = void (*)(void *);
  ToFBus(TwoWire &bus, int sda, int scl, uint16_t minimum, uint16_t maximum,
         uint32_t budget = 20000, const Settings &settings = {})
      : m_bus(bus), m_sda(sda), m_scl(scl), m_min(minimum), m_max(maximum),
        m_budget(budget), m_settings(settings) {
    m_bus_backoff.configure(settings);
    for (auto &device : devices)
      device.backoff.configure(settings);
  }
  ToFDevice devices[4];
  unsigned count = 0;
  void set_publisher(Publish publish, void *context) {
    m_publish = publish;
    m_context = context;
  }
  void begin() {
    m_bus.begin(m_sda, m_scl, 400000);
    m_bus.setTimeOut(m_settings.bus_timeout_ms);
    hold_group();
    m_group_pending = true;
  }
  void publish() {
    if (m_publish)
      m_publish(m_context);
  }
  bool pending() const {
    if (m_group_pending)
      return true;
    for (unsigned i = 0; i < count; ++i)
      if (devices[i].pending)
        return true;
    return false;
  }
  /** Whether a recovery attempt is due now; backoff alone does not stop a C3.
   */
  bool recovery_due() const {
    if (grace_active())
      return false;
    if (m_group_pending)
      return m_bus_backoff.due(millis());
    for (unsigned i = 0; i < count; ++i)
      if (devices[i].pending && devices[i].backoff.due(millis()))
        return true;
    return false;
  }
  bool grace_active() const {
    for (unsigned i = 0; i < count; ++i)
      if (devices[i].grace_start_us)
        return true;
    return false;
  }
  // Called around known blocking work, never around an ordinary scheduling
  // overrun.
  void pause_begin() {
    uint32_t now = micros();
    m_bus_backoff.fault();
    for (unsigned i = 0; i < count; ++i) {
      auto &d = devices[i];
      d.grace_eligible = d.active && !d.pending && !d.grace_start_us &&
                         now - d.result_us <= m_settings.tof_freshness_us;
      d.backoff.fault();
    }
  }
  void pause_end() {
    uint32_t now = micros();
    for (unsigned i = 0; i < count; ++i) {
      auto &d = devices[i];
      if (d.grace_eligible && d.active && !d.pending)
        d.grace_start_us = now;
      d.grace_eligible = false;
    }
  }
  void poll() {
    unsigned faults = 0;
    bool all_healthy = m_peer_healthy;
    for (unsigned i = 0; i < count; ++i) {
      auto &d = devices[i];
      if (!d.active || d.pending) {
        all_healthy = false;
        continue;
      }
      d.driver.clear_error();
      uint8_t ready = d.driver.readReg(VL53L0X::RESULT_INTERRUPT_STATUS);
      bool complete = false;
      bool failed_acquisition = false;
      if (!d.driver.first_error() && (ready & 7)) {
        uint8_t result[12] = {};
        d.driver.read_checked(VL53L0X::RESULT_RANGE_STATUS, result, 12);
        d.driver.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 1);
        if (!d.driver.first_error()) {
          uint32_t now = micros();
          if (now - d.result_us > m_settings.tof_freshness_us) {
            d.backoff.fault();
            m_bus_backoff.fault();
          }
          complete = true;
          d.result_us = now;
          d.grace_start_us = 0;
          bool healthy = accept_range(
              d.data, (uint16_t(result[10]) << 8) | result[11], result[0], now,
              m_min, m_max, m_settings.allow_no_return);
          if (healthy)
            d.backoff.acquired(now);
          else {
            d.backoff.fault();
            failed_acquisition = true;
          }
          if (d.data.health != SensorHealth::OK)
            publish();
        }
      }
      if (d.driver.first_error()) {
        failed_acquisition = true;
        d.grace_start_us = 0;
        d.backoff.fault();
        fail(d.data, wire_error(d.driver.first_error()));
        publish();
      } else if (!complete) {
        uint32_t now = micros();
        bool timeout =
            d.grace_start_us
                ? now - d.grace_start_us >= m_settings.tof_resume_grace_us
                : now - d.result_us >= m_settings.tof_freshness_us;
        if (timeout) {
          fail(d.data, SensorError::MEASUREMENT_TIMEOUT);
          d.grace_start_us = 0;
          d.backoff.fault();
          d.pending = true;
          publish();
        }
      }
      if (failed_acquisition &&
          d.data.consecutive_errors >= m_settings.max_consecutive_errors)
        d.pending = true;
      if (d.data.health == SensorHealth::READ_ERROR) {
        ++faults;
        all_healthy = false;
      }
      if (micros() - d.result_us > m_settings.tof_freshness_us)
        all_healthy = false;
      if (d.pending) {
        d.active = false;
        d.backoff.defer(millis());
        d.data.health = SensorHealth::OFFLINE;
        publish();
        if (d.xshut < 0 && !m_group_pending) {
          m_group_pending = true;
          m_bus_backoff.defer(millis());
        }
      }
    }
    if (all_healthy)
      m_bus_backoff.acquired(micros());
    else
      m_bus_backoff.fault();
    if (faults >= 2 || digitalRead(m_sda) == LOW || digitalRead(m_scl) == LOW) {
      if (!m_group_pending)
        m_bus_backoff.defer(millis());
      m_group_pending = true;
      m_reset_bus = true;
      mark_group(SensorHealth::RECOVERING);
    }
  }
  void set_peer_healthy(bool healthy) { m_peer_healthy = healthy; }
  bool bus_restart_due() const {
    return m_group_pending && m_reset_bus && m_bus_backoff.due(millis());
  }
  // One bounded device or group attempt, yielding between group members.
  // C3 callers MUST have written zero motor outputs before calling this method.
  bool recover() {
    if (grace_active())
      return false;
    if (m_group_pending) {
      if (!m_bus_backoff.due(millis()))
        return false;
      mark_group(SensorHealth::RECOVERING);
      pause_begin();
      hold_group();
      delay(10);
      if (m_reset_bus) {
        restart_bus();
        m_reset_bus = false;
      }
      for (unsigned i = 0; i < count; ++i) {
        bool initialized = initialize(devices[i]);
        publish();
        vTaskDelay(1);
        // An always-powered device may still answer at 0x29 after a failed
        // address write. Keep remaining XSHUT devices isolated until it
        // recovers.
        if (!initialized && devices[i].xshut < 0)
          break;
      }
      m_group_pending = false;
      for (unsigned i = 0; i < count; ++i)
        if (devices[i].pending && devices[i].xshut < 0)
          m_group_pending = true;
      if (m_group_pending)
        m_bus_backoff.defer(millis());
      pause_end();
      return true;
    }
    for (unsigned i = 0; i < count; ++i) {
      auto &d = devices[i];
      if (!d.pending || !d.backoff.due(millis()))
        continue;
      d.data.health = SensorHealth::RECOVERING;
      publish();
      pause_begin();
      pinMode(d.xshut, OUTPUT);
      digitalWrite(d.xshut, LOW);
      delay(10);
      initialize(d);
      publish();
      pause_end();
      return true;
    }
    return false;
  }

private:
  void mark_group(SensorHealth health) {
    for (unsigned i = 0; i < count; ++i) {
      devices[i].data.health = health;
      devices[i].active = false;
      devices[i].pending = true;
      devices[i].grace_start_us = 0;
    }
    publish();
  }
  void hold_group() {
    for (unsigned i = 0; i < count; ++i)
      if (devices[i].xshut >= 0) {
        pinMode(devices[i].xshut, OUTPUT);
        digitalWrite(devices[i].xshut, LOW);
      }
  }
  bool probe(uint8_t address) {
    m_bus.beginTransmission(address);
    return m_bus.endTransmission() == 0;
  }
  bool initialize(ToFDevice &d) {
    d.driver = VL53L0X();
    d.driver.setBus(&m_bus);
    d.driver.setTimeout(m_settings.tof_timeout_ms);
    m_bus.setTimeOut(m_settings.bus_timeout_ms);
    d.driver.clear_error();
    if (d.xshut < 0) {
      if (probe(d.address))
        d.driver.set_host_address(d.address);
      else if (probe(0x29)) {
        d.driver.set_host_address(0x29);
        d.driver.setAddress(d.address);
      } else
        return init_failed(d, SensorError::INIT_FAILED);
    } else {
      pinMode(d.xshut, INPUT);
      delay(10);
      d.driver.setAddress(d.address);
    }
    if (d.driver.first_error())
      return init_failed(d, wire_error(d.driver.first_error()));
    if (!d.driver.init())
      return init_failed(d, d.driver.first_error()
                                ? wire_error(d.driver.first_error())
                                : SensorError::INIT_FAILED);
    bool configured = d.driver.setMeasurementTimingBudget(m_budget);
    d.driver.startContinuous();
    if (!configured || d.driver.first_error())
      return init_failed(d, d.driver.first_error()
                                ? wire_error(d.driver.first_error())
                                : SensorError::CONFIG_FAILED);
    d.active = true;
    d.pending = false;
    d.result_us = micros();
    d.grace_start_us = 0;
    d.data.health = SensorHealth::INITIALIZING;
    return true;
  }
  bool init_failed(ToFDevice &d, SensorError error) {
    fail(d.data, error);
    publish();
    d.active = false;
    d.pending = true;
    d.backoff.defer(millis());
    d.data.health = SensorHealth::OFFLINE;
    // A failed address/config transaction can leave a released device at 0x29.
    // Isolate it now so it cannot collide with the next group member.
    if (d.xshut >= 0) {
      pinMode(d.xshut, OUTPUT);
      digitalWrite(d.xshut, LOW);
    }
    return false;
  }
  void restart_bus() {
    m_bus.end();
    hold_group();
    pinMode(m_sda, OUTPUT_OPEN_DRAIN);
    digitalWrite(m_sda, HIGH);
    pinMode(m_scl, OUTPUT_OPEN_DRAIN);
    digitalWrite(m_scl, HIGH);
    for (unsigned i = 0; i < 9 && digitalRead(m_sda) == LOW; ++i) {
      digitalWrite(m_scl, LOW);
      delayMicroseconds(5);
      digitalWrite(m_scl, HIGH);
      delayMicroseconds(5);
    }
    digitalWrite(m_scl, LOW);
    digitalWrite(m_sda, LOW);
    delayMicroseconds(5);
    digitalWrite(m_scl, HIGH);
    delayMicroseconds(5);
    digitalWrite(m_sda, HIGH);
    m_bus.begin(m_sda, m_scl, 400000);
    m_bus.setTimeOut(m_settings.bus_timeout_ms);
  }
  TwoWire &m_bus;
  int m_sda, m_scl;
  uint16_t m_min, m_max;
  uint32_t m_budget;
  Settings m_settings;
  Publish m_publish = nullptr;
  void *m_context = nullptr;
  bool m_group_pending = true, m_reset_bus = false, m_peer_healthy = true;
  Backoff m_bus_backoff;
};
} // namespace sensor_policy
