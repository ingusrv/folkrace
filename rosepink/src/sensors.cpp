#include "sensors.h"
#include "config.h"
#include "sensors/main_owner.h"
#include "side_fit.h"
#include <freertos/task.h>
#include <type_traits>

namespace {
constexpr sensor_policy::Settings settings = {
    config::sensor_freshness_tof_us,
    config::sensor_freshness_imu_us,
    config::sensor_max_consecutive_errors,
    config::sensor_recovery_delay_initial_ms,
    config::sensor_recovery_delay_max_ms,
    config::sensor_recovery_healthy_ms,
    config::sensor_bus_timeout_ms,
    config::tof_timeout_ms,
    config::tof_resume_grace_us};
// Control, BLE and telemetry use this same fixed freshness contract.
static_assert(config::sensor_freshness_tof_us == sensor_policy::tof_age_us);
static_assert(config::sensor_freshness_imu_us == sensor_policy::imu_age_us);

QueueHandle_t main_queue = nullptr, side_queue = nullptr,
              command_queue = nullptr;
TaskHandle_t main_handle = nullptr, side_handle = nullptr;
void cadence(TickType_t &wake) {
  if (xTaskDelayUntil(&wake, pdMS_TO_TICKS(10)) != pdTRUE) {
    vTaskDelay(1);
    wake = xTaskGetTickCount();
  }
}
void main_sensor_task(void *) {
  sensor_policy::MainSensorOwner owner(
      Wire, pin::sda, pin::scl, -1, pin::xshut_left, pin::xshut_right,
      config::sensor_min_raw, config::sensor_max_raw,
      config::tof_timing_budget_us, main_queue, command_queue,
      config::imu_min_calib_samples, config::imu_max_init_attempts_per_boot,
      settings);
  owner.begin();
  TickType_t wake = xTaskGetTickCount();
  uint32_t previous_start_us = micros();
  for (;;) {
    uint32_t start = micros();
    owner.snapshot.max_period_us =
        std::max(owner.snapshot.max_period_us, start - previous_start_us);
    previous_start_us = start;
    owner.tick();
    owner.snapshot.max_tick_us =
        std::max(owner.snapshot.max_tick_us, micros() - start);
    owner.snapshot.task_stack_min_bytes = uxTaskGetStackHighWaterMark(nullptr);
    owner.snapshot.last_imu_init_us = owner.init_duration_us;
    cadence(wake);
  }
}
// Large firmware/configuration and result buffers stay off the 4 KiB task
// stack.
struct SideDevice {
  SparkFun_VL53L5CX driver;
  VL53L5CX_ResultsData result = {};
  SensorData front, rear;
  sensor_policy::Backoff backoff;
  bool active = false, pending = true;
  uint32_t result_us = 0, grace_us = 0;
};
struct SideOwner {
  SideDevice devices[2];
  SideSensorSnapshot s;
  sensor_policy::Backoff bus_backoff;
  bool group_pending = true;
  void publish() {
    s.dist_side_front_left = devices[0].front;
    s.dist_side_rear_left = devices[0].rear;
    s.dist_side_front_right = devices[1].front;
    s.dist_side_rear_right = devices[1].rear;
    s.publish_timestamp_us = micros();
    xQueueOverwrite(side_queue, &s);
  }
  void health(SideDevice &d, SensorHealth h) {
    d.front.health = d.rear.health = h;
  }
  void fail(SideDevice &d, SensorError e) {
    sensor_policy::fail(d.front, e);
    sensor_policy::fail(d.rear, e);
    d.backoff.fault();
    bus_backoff.fault();
    publish(); // Failure visible before any other blocking transaction.
  }
  bool probe(uint8_t address) {
    Wire1.beginTransmission(address);
    return Wire1.endTransmission() == 0;
  }
  void restart_bus() {
    Wire1.end();
    pinMode(pin::sda_side, OUTPUT_OPEN_DRAIN);
    pinMode(pin::scl_side, OUTPUT_OPEN_DRAIN);
    digitalWrite(pin::sda_side, HIGH);
    digitalWrite(pin::scl_side, HIGH);
    for (unsigned i = 0; i < 9; ++i) {
      digitalWrite(pin::scl_side, LOW);
      delayMicroseconds(5);
      digitalWrite(pin::scl_side, HIGH);
      delayMicroseconds(5);
    }
    digitalWrite(pin::scl_side, LOW);
    digitalWrite(pin::sda_side, LOW);
    delayMicroseconds(5);
    digitalWrite(pin::scl_side, HIGH);
    delayMicroseconds(5);
    digitalWrite(pin::sda_side, HIGH);
    Wire1.begin(pin::sda_side, pin::scl_side, 400000);
    Wire1.setTimeOut(settings.bus_timeout_ms);
  }
  void begin() {
    bus_backoff.configure(settings);
    for (auto &d : devices)
      d.backoff.configure(settings);
    pinMode(pin::xshut_side_right, OUTPUT);
    digitalWrite(pin::xshut_side_right, LOW);
    Wire1.begin(pin::sda_side, pin::scl_side, 400000);
    Wire1.setTimeOut(settings.bus_timeout_ms);
  }
  void init_device(unsigned i) {
    auto &d = devices[i];
    health(d, SensorHealth::RECOVERING);
    publish();
    d.active = false;
    uint8_t address =
        i ? config::vl53l5cx_addr_right : config::vl53l5cx_addr_left;
    if (i) {
      pinMode(pin::xshut_side_right, OUTPUT);
      digitalWrite(pin::xshut_side_right, LOW);
      vTaskDelay(pdMS_TO_TICKS(10));
      pinMode(pin::xshut_side_right, INPUT);
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    // LPn may retain the address; adopt an existing address before default.
    uint8_t found = probe(address) ? address : 0x29;
    d.driver.VL53L5CX_i2c->clear_error();
    bool ok = d.driver.begin(found, Wire1) &&
              (found == address || d.driver.setAddress(address)) &&
              d.driver.setResolution(16) &&
              d.driver.setRangingFrequency(config::vl53l5cx_frequency_hz) &&
              d.driver.startRanging() && !d.driver.VL53L5CX_i2c->first_error;
    d.backoff.defer(millis());
    d.pending = !ok;
    d.active = ok;
    d.grace_us = 0;
    if (ok) {
      d.result_us = micros();
      health(d, SensorHealth::INITIALIZING);
    } else {
      fail(d, SensorError::INIT_FAILED);
      health(d, SensorHealth::OFFLINE);
    }
    publish();
    vTaskDelay(1);
  }
  void poll() {
    bool healthy = true;
    for (unsigned i = 0; i < 2; ++i) {
      auto &d = devices[i];
      if (!d.active || d.pending) {
        healthy = false;
        continue;
      }
      auto &io = *d.driver.VL53L5CX_i2c;
      io.clear_error();
      bool ready = d.driver.isDataReady();
      bool ok = !io.first_error && d.driver.lastError.lastErrorCode ==
                                       SF_VL53L5CX_ERROR_TYPE::NO_ERROR;
      bool complete = ready && ok && d.driver.getRangingData(&d.result);
      ok = ok && !io.first_error &&
           d.driver.lastError.lastErrorCode == SF_VL53L5CX_ERROR_TYPE::NO_ERROR;
      if (!ok || (ready && !complete)) {
        SensorError error = io.first_error == 6   ? SensorError::SHORT_READ
                            : io.first_error == 5 ? SensorError::TIMEOUT
                            : (io.first_error == 2 || io.first_error == 3)
                                ? SensorError::NACK
                                : SensorError::BUS_ERROR;
        fail(d, error);
        d.grace_us = 0;
        if (d.front.consecutive_errors >= settings.max_consecutive_errors)
          d.pending = true;
      } else if (complete) {
        uint32_t now = micros();
        if (now - d.result_us > settings.tof_freshness_us)
          d.backoff.fault();
        d.result_us = now;
        d.grace_us = 0;
        d.front.consecutive_errors = d.rear.consecutive_errors = 0;
        auto estimate = process_vl53l5cx(
            d.result, i ? config::side_right_flip_x : config::side_left_flip_x,
            i ? config::side_right_flip_y : config::side_left_flip_y, i == 0);
        if (estimate.valid) {
          d.front.distance_mm =
              constrain(estimate.front_dist_mm, config::sensor_min_raw,
                        config::sensor_max_raw);
          d.rear.distance_mm =
              constrain(estimate.rear_dist_mm, config::sensor_min_raw,
                        config::sensor_max_raw);
          sensor_policy::success(d.front, now);
          sensor_policy::success(d.rear, now);
        } else {
          health(d, SensorHealth::RANGE_INVALID);
          publish();
        }
        d.backoff.acquired(now); // Quality is not a transport fault.
      } else if (d.grace_us
                     ? micros() - d.grace_us >= settings.tof_resume_grace_us
                     : micros() - d.result_us >= settings.tof_freshness_us) {
        fail(d, SensorError::MEASUREMENT_TIMEOUT);
        d.pending = true;
      }
      if (d.pending) {
        d.active = false;
        d.grace_us = 0;
        d.backoff.defer(millis());
        health(d, SensorHealth::OFFLINE);
        publish();
        // Isolate the right sensor before adopting the left's default address.
        if (i == 0 && !group_pending) {
          group_pending = true;
          bus_backoff.defer(millis());
        }
      }
      if (!ok || d.pending ||
          micros() - d.result_us > settings.tof_freshness_us)
        healthy = false;
    }
    if (healthy)
      bus_backoff.acquired(micros());
    else
      bus_backoff.fault();
    if (digitalRead(pin::sda_side) == LOW ||
        digitalRead(pin::scl_side) == LOW) {
      if (!group_pending)
        bus_backoff.defer(millis());
      group_pending = true;
      for (auto &d : devices) {
        d.active = false;
        health(d, SensorHealth::RECOVERING);
      }
      publish();
    }
  }
  void recover() {
    // Consume resumed frames before another long initialization.
    for (auto &d : devices)
      if (d.grace_us)
        return;
    if (group_pending && bus_backoff.due(millis())) {
      for (auto &d : devices) {
        health(d, SensorHealth::RECOVERING);
        d.active = false;
      }
      publish();
      pinMode(pin::xshut_side_right, OUTPUT);
      digitalWrite(pin::xshut_side_right, LOW);
      vTaskDelay(pdMS_TO_TICKS(10));
      restart_bus();
      init_device(0);
      // Never release another default-address device until left is assigned.
      if (devices[0].active)
        init_device(1);
      else {
        devices[1].pending = true;
        health(devices[1], SensorHealth::OFFLINE);
      }
      group_pending = !devices[0].active;
      bus_backoff.defer(millis());
      for (auto &d : devices)
        if (d.active)
          d.grace_us = micros();
      publish();
      return;
    }
    if (group_pending)
      return;
    auto &right = devices[1];
    if (right.pending && right.backoff.due(millis())) {
      bool grace = devices[0].active &&
                   micros() - devices[0].result_us <= settings.tof_freshness_us;
      devices[0].backoff.fault();
      bus_backoff.fault();
      init_device(1);
      if (grace)
        devices[0].grace_us = micros();
    }
  }
};
void side_tof_task(void *) {
  static SideOwner owner;
  owner.begin();
  TickType_t wake = xTaskGetTickCount();
  uint32_t previous_start_us = micros();
  for (;;) {
    uint32_t start = micros();
    owner.s.max_period_us =
        std::max(owner.s.max_period_us, start - previous_start_us);
    previous_start_us = start;
    owner.poll();
    owner.publish();
    owner.recover();
    owner.s.max_tick_us = std::max(owner.s.max_tick_us, micros() - start);
    owner.s.task_stack_min_bytes = uxTaskGetStackHighWaterMark(nullptr);
    cadence(wake);
  }
}
} // namespace
namespace sensors {
bool init() {
  static_assert(std::is_trivially_copyable<MainSensorSnapshot>::value,
                "snapshot must copy");
  static_assert(std::is_trivially_copyable<SideSensorSnapshot>::value,
                "snapshot must copy");
  main_queue = xQueueCreate(1, sizeof(MainSensorSnapshot));
  side_queue = xQueueCreate(1, sizeof(SideSensorSnapshot));
  command_queue = xQueueCreate(4, sizeof(SensorCommand));
  if (!main_queue || !side_queue || !command_queue)
    return false;
  MainSensorSnapshot main;
  SideSensorSnapshot side;
  xQueueOverwrite(main_queue, &main);
  xQueueOverwrite(side_queue, &side);
  if (xTaskCreatePinnedToCore(
          main_sensor_task, "main_sensor_task", config::sensor_task_stack_size,
          nullptr, config::sensor_task_priority, &main_handle, 0) != pdPASS)
    return false;
  if (xTaskCreatePinnedToCore(
          side_tof_task, "side_tof_task", config::sensor_task_stack_size,
          nullptr, config::sensor_task_priority, &side_handle, 0) != pdPASS)
    return false;
  return true;
}
bool get_main_snapshot(MainSensorSnapshot &out) {
  return main_queue && xQueuePeek(main_queue, &out, 0) == pdTRUE;
}
bool get_side_snapshot(SideSensorSnapshot &out) {
  return side_queue && xQueuePeek(side_queue, &out, 0) == pdTRUE;
}
bool send_command(const SensorCommand &cmd) {
  return command_queue && xQueueSend(command_queue, &cmd, 0) == pdTRUE;
}
} // namespace sensors
