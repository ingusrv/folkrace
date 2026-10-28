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

/** Accept a completed result, including maximum-range/sentinel returns.
 * Range status is diagnostic only; recovery depends on communication/cadence.
 */
inline void accept_range(SensorData &out, uint16_t raw, uint8_t status,
                         uint32_t now, uint16_t minimum, uint16_t maximum) {
  out.raw_range_mm = raw;
  out.raw_status = status;
  out.raw_timestamp_us = now;
  out.distance_mm = std::max(minimum, std::min(maximum, raw));
  success(out, now);
  if (raw >= maximum)
    out.health = SensorHealth::OUT_OF_RANGE;
}

/** Accept a VL53L0X result using its existing sentinel-based range policy. */
inline void tof_accept_range(VL53L0X &, SensorData &out, uint16_t raw,
                             uint8_t status, uint32_t now, uint16_t minimum,
                             uint16_t maximum) {
  accept_range(out, raw, status, now, minimum, maximum);
}

// Sensor-specific polling stays separate from bus recovery and freshness.
inline uint8_t tof_read_sample(VL53L0X &driver, bool &ready,
                               uint16_t &range_mm, uint8_t &status) {
  uint8_t interrupt = driver.readReg(VL53L0X::RESULT_INTERRUPT_STATUS);
  if (driver.last_status)
    return driver.last_status;
  ready = (interrupt & 7) != 0;
  if (!ready)
    return 0;
  uint8_t result[12] = {};
  driver.readMulti(VL53L0X::RESULT_RANGE_STATUS, result, sizeof(result));
  if (driver.last_status)
    return driver.last_status;
  driver.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 1);
  range_mm = (static_cast<uint16_t>(result[10]) << 8) | result[11];
  status = result[0];
  return driver.last_status;
}

template <typename Driver> struct ToFDeviceT {
  Driver driver;
  SensorData data;
  int8_t xshut = -1;
  uint8_t address = 0x30;
  // ERROR waits for retry; RECOVERING polls for the first fresh result.
  uint32_t retry_at_ms = 0, result_us = 0, grace_start_us = 0;
};

// One task owns each bus. C3 callers stop motors before blocking recovery.
template <typename Driver> class ToFBusT {
public:
  using Publish = void (*)(void *);
  ToFBusT(TwoWire &bus, int sda, int scl, uint16_t minimum, uint16_t maximum,
         uint32_t budget = 20000, const Settings &settings = {})
      : m_bus(bus), m_sda(sda), m_scl(scl), m_min(minimum), m_max(maximum),
        m_budget(budget), m_settings(settings) {}
  using ToFDevice = ToFDeviceT<Driver>;
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
    for (unsigned i = 0; i < count; ++i)
      devices[i].data.health = SensorHealth::RECOVERING;
  }
  bool pending() const {
    if (m_group_pending)
      return true;
    for (unsigned i = 0; i < count; ++i)
      if (devices[i].data.health == SensorHealth::READ_ERROR)
        return true;
    return false;
  }
  bool recovery_due() const {
    if (grace_active())
      return false;
    if (m_group_pending)
      return due(m_group_retry_ms);
    for (unsigned i = 0; i < count; ++i)
      if (devices[i].data.health == SensorHealth::READ_ERROR &&
          due(devices[i].retry_at_ms))
        return true;
    return false;
  }
  bool bus_restart_due() const {
    return m_group_pending && m_reset_bus && due(m_group_retry_ms);
  }
  bool grace_active() const {
    for (unsigned i = 0; i < count; ++i)
      if (devices[i].grace_start_us)
        return true;
    return false;
  }
  // Known owner pauses (IMU/device initialization) allow one bounded window
  // to receive a new result. Published sample timestamps never get extended.
  void pause_end() {
    for (unsigned i = 0; i < count; ++i) {
      auto &d = devices[i];
      if (d.data.health != SensorHealth::READ_ERROR)
        d.grace_start_us = micros();
    }
  }
  void poll() {
    if (digitalRead(m_sda) == LOW || digitalRead(m_scl) == LOW) {
      m_reset_bus = true;
      schedule_group();
    }
    if (m_group_pending)
      return;
    for (unsigned i = 0; i < count; ++i) {
      auto &d = devices[i];
      if (d.data.health == SensorHealth::READ_ERROR)
        continue;
      poll_device(d);
    }
  }
  /** Attempt one sensor or isolated group; caller may block during init.
   * @return Whether an attempt ran (not whether it succeeded).
   */
  bool recover() {
    if (!recovery_due())
      return false;
    if (m_group_pending) {
      mark_group(SensorHealth::RECOVERING);
      hold_group();
      delay(10);
      if (m_reset_bus) {
        restart_bus();
        m_reset_bus = false;
      }
      bool primary_failed = false;
      for (unsigned i = 0; i < count; ++i) {
        bool ok = initialize(devices[i]);
        publish();
        vTaskDelay(1);
        // The always-powered sensor can remain at 0x29. Do not release
        // another sensor at that address until it has been reassigned.
        if (!ok && devices[i].xshut < 0) {
          primary_failed = true;
          break;
        }
      }
      m_group_pending = primary_failed;
      if (primary_failed) {
        mark_group(SensorHealth::READ_ERROR);
        m_group_retry_ms = millis() + m_settings.recovery_initial_ms;
      } else {
        pause_end();
      }
      publish();
      return true;
    }
    for (unsigned i = 0; i < count; ++i) {
      auto &d = devices[i];
      if (d.data.health != SensorHealth::READ_ERROR || !due(d.retry_at_ms))
        continue;
      d.data.health = SensorHealth::RECOVERING;
      publish();
      pinMode(d.xshut, OUTPUT);
      digitalWrite(d.xshut, LOW);
      delay(10);
      initialize(d);
      pause_end();
      publish();
      return true;
    }
    return false;
  }

private:
  bool due(uint32_t deadline_ms) const {
    return static_cast<int32_t>(millis() - deadline_ms) >= 0;
  }
  void publish() {
    if (m_publish)
      m_publish(m_context);
  }
  void poll_device(ToFDevice &d) {
    bool ready = false;
    uint16_t range_mm = 0;
    uint8_t status = 0;
    uint8_t error = tof_read_sample(d.driver, ready, range_mm, status);
    if (error) {
      fault(d, wire_error(error));
      return;
    }
    if (ready) {
      d.result_us = micros();
      d.grace_start_us = 0;
      tof_accept_range(d.driver, d.data, range_mm, status, d.result_us,
                       m_min, m_max);
    } else {
      uint32_t start = d.grace_start_us ? d.grace_start_us : d.result_us;
      uint32_t timeout = d.grace_start_us ? m_settings.tof_resume_grace_us
                                          : m_settings.tof_freshness_us;
      if (micros() - start >= timeout)
        fault(d, SensorError::MEASUREMENT_TIMEOUT);
    }
  }
  void fault(ToFDevice &d, SensorError error) {
    fail(d.data, error);
    d.grace_start_us = 0;
    d.retry_at_ms = millis() + m_settings.recovery_initial_ms;
    publish();
    if (d.xshut < 0)
      schedule_group();
  }
  void mark_group(SensorHealth health) {
    for (unsigned i = 0; i < count; ++i) {
      devices[i].data.health = health;
      devices[i].grace_start_us = 0;
    }
    publish();
  }
  void schedule_group() {
    if (!m_group_pending)
      m_group_retry_ms = millis() + m_settings.recovery_initial_ms;
    m_group_pending = true;
    mark_group(SensorHealth::READ_ERROR);
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
    d.driver = Driver();
    d.driver.setBus(&m_bus);
    d.driver.setTimeout(m_settings.tof_timeout_ms);
    if (d.xshut >= 0) {
      pinMode(d.xshut, INPUT);
      delay(10);
      d.driver.setAddress(d.address);
      if (d.driver.last_status)
        return init_failed(d, wire_error(d.driver.last_status));
    } else {
      // Upstream setAddress also updates the host-side address on NACK.
      // At MCU reboot this sensor may already be at its assigned address;
      // verify that address explicitly instead of requiring a write at 0x29.
      d.driver.setAddress(d.address);
      if (!probe(d.address))
        return init_failed(d, SensorError::NACK);
    }
    if (!d.driver.init() || d.driver.last_status)
      return init_failed(d, SensorError::INIT_FAILED);
    if (!d.driver.setMeasurementTimingBudget(m_budget) || d.driver.last_status)
      return init_failed(d, SensorError::CONFIG_FAILED);
    d.driver.startContinuous();
    if (d.driver.last_status)
      return init_failed(d, wire_error(d.driver.last_status));
    d.result_us = micros();
    d.grace_start_us = 0;
    d.data.health = SensorHealth::RECOVERING;
    return true;
  }
  bool init_failed(ToFDevice &d, SensorError error) {
    fail(d.data, error);
    d.retry_at_ms = millis() + m_settings.recovery_initial_ms;
    d.grace_start_us = 0;
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
  bool m_group_pending = true, m_reset_bus = false;
  uint32_t m_group_retry_ms = 0;
};
using ToFDevice = ToFDeviceT<VL53L0X>;
using ToFBus = ToFBusT<VL53L0X>;
} // namespace sensor_policy
