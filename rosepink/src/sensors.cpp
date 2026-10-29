#include "sensors.h"
#include "ble/ble.h"
#include "config.h"
#include "imu_owner.h"
#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <freertos/task.h>
#include <type_traits>

namespace {
QueueHandle_t diagnostic_queue = nullptr;
struct DiagnosticLine {
  char text[192];
};
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
static_assert(config::sensor_freshness_tof_us == sensor_policy::tof_age_us);
static_assert(config::sensor_freshness_imu_us == sensor_policy::imu_age_us);
QueueHandle_t main_queue = nullptr, tof_queue = nullptr,
              command_queue = nullptr;
std::atomic<uint32_t> stopped_recovery_id{0};
void cadence(TickType_t &wake) {
  if (xTaskDelayUntil(&wake, pdMS_TO_TICKS(10)) != pdTRUE) {
    vTaskDelay(1);
    wake = xTaskGetTickCount();
  }
}
void main_sensor_task(void *) {
  sensor_policy::RosepinkIMUOwner owner(Wire, main_queue, command_queue,
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
// Persistent storage keeps firmware/configuration/results off the task stack.
struct ToFDevice {
  SparkFun_VL53L5CX driver;
  VL53L5CX_ResultsData result = {};
  SensorData distance;
  SideWallEstimate wall;
  sensor_policy::Backoff backoff;
  bool active = false, pending = true;
  uint32_t result_us = 0;
};
struct ToFOwner {
  ToFDevice devices[3];
  RosepinkToFSnapshot s;
  sensor_policy::Backoff bus_backoff;
  bool group_pending = true;
  bool request_pending = false;
  void publish() {
    s.dist_front = devices[0].distance;
    s.dist_left = devices[1].distance;
    s.dist_right = devices[2].distance;
    s.left_wall = devices[1].wall;
    s.right_wall = devices[2].wall;
    s.publish_timestamp_us = micros();
    xQueueOverwrite(tof_queue, &s);
  }
  void fail(ToFDevice &d, SensorError error) {
    sensor_policy::fail(d.distance, error);
    d.wall.valid = false;
    d.backoff.fault();
    publish();
  }
  static void hold(unsigned i) {
    digitalWrite(pin::lpn_tof[i], LOW);
    pinMode(pin::lpn_tof[i], OUTPUT);
  }
  void start_bus() {
    Wire1.setBufferSize(config::vl53l5cx_packet_bytes);
    Wire1.begin(pin::sda_side, pin::scl_side, config::vl53l5cx_bus_hz);
    Wire1.setTimeOut(settings.bus_timeout_ms);
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
    start_bus();
  }
  bool probe(uint8_t address) {
    Wire1.beginTransmission(address);
    return Wire1.endTransmission() == 0;
  }
  void begin() {
    bus_backoff.configure(settings);
    for (unsigned i = 0; i < 3; ++i) {
      devices[i].backoff.configure(settings);
      devices[i].driver.setWireMaxPacketSize(config::vl53l5cx_packet_bytes);
      hold(i);
    }
    start_bus();
    publish();
  }
  void init_device(unsigned i, bool isolated = false) {
    auto &d = devices[i];
    Wire1.setClock(config::vl53l5cx_init_bus_hz);
    d.driver.setWireMaxPacketSize(config::vl53l5cx_init_packet_bytes);
    d.active = false;
    d.distance.health = SensorHealth::RECOVERING;
    d.wall = {};
    publish();
    hold(i);
    vTaskDelay(pdMS_TO_TICKS(10));
    // LPn gates communication, retaining firmware/address; release to pull-up.
    pinMode(pin::lpn_tof[i], INPUT);
    vTaskDelay(pdMS_TO_TICKS(10));
    uint8_t address = config::vl53l5cx_addresses[i];
    uint8_t found = probe(address) ? address : 0x29;
    // After a warm flash LPn can retain an address from the old layout. During
    // group init every other device is held low, so safely discover that
    // address.
    if (isolated && !probe(found)) {
      for (uint8_t candidate : config::vl53l5cx_addresses)
        if (probe(candidate)) {
          found = candidate;
          break;
        }
    }
    d.driver.VL53L5CX_i2c->clear_error();
    uint32_t start = micros();
    auto &diagnostic = s.initialization[i];
    // Retain the last completed failure while a retry is in progress.
    diagnostic.in_progress = true;
    diagnostic.started = true;
    if (!diagnostic.complete) {
      diagnostic.found_address = found;
      snprintf(diagnostic.stage, sizeof(diagnostic.stage), "firmware");
    }
    publish();
    sensors::queue_diagnostic(
        "[TOF] sensor=%u LPn=%u found=0x%02x target=0x%02x "
        "init_bus=%lu Hz packet=%u init begin\n",
        i, pin::lpn_tof[i], found, address,
        static_cast<unsigned long>(config::vl53l5cx_init_bus_hz),
        config::vl53l5cx_init_packet_bytes);
    const char *stage = "firmware";
    bool ok = d.driver.begin(found, Wire1);
    if (ok && found != address) {
      stage = "address";
      ok = d.driver.setAddress(address);
    }
    if (ok) {
      stage = "resolution";
      ok = d.driver.setResolution(16);
    }
    if (ok) {
      stage = "frequency";
      ok = d.driver.setRangingFrequency(config::vl53l5cx_frequency_hz);
    }
    if (ok) {
      stage = "start_ranging";
      ok = d.driver.startRanging();
    }
    ok = ok && !d.driver.VL53L5CX_i2c->first_error;
    // No peer polling occurs during stopped recovery; every L5CX supports both
    // rates. Restore fast frame reads even if this device failed
    // initialization.
    Wire1.setClock(config::vl53l5cx_bus_hz);
    d.driver.setWireMaxPacketSize(config::vl53l5cx_packet_bytes);
    diagnostic.complete = true;
    diagnostic.in_progress = false;
    diagnostic.found_address = found;
    diagnostic.success = ok;
    diagnostic.duration_ms = (micros() - start) / 1000;
    diagnostic.transport_error = d.driver.VL53L5CX_i2c->first_error;
    diagnostic.driver_error =
        static_cast<unsigned>(d.driver.lastError.lastErrorCode);
    diagnostic.driver_value = d.driver.lastError.lastErrorValue;
    snprintf(diagnostic.stage, sizeof(diagnostic.stage), "%s", stage);
    if (!ok)
      sensors::queue_diagnostic(
          "[TOF] sensor=%u failed_step=%s transport=%u "
          "driver_error=%u value=%lu\n",
          i, stage, d.driver.VL53L5CX_i2c->first_error,
          static_cast<unsigned>(d.driver.lastError.lastErrorCode),
          static_cast<unsigned long>(d.driver.lastError.lastErrorValue));
    d.backoff.defer(millis());
    d.pending = !ok;
    d.active = ok;
    if (ok) {
      d.result_us = micros();
      d.distance.health = SensorHealth::INITIALIZING;
      s.frame_bytes[i] = d.driver.Dev->data_read_size;
    } else {
      fail(d, SensorError::INIT_FAILED);
      hold(i); // Never leave a failed/default-address device visible.
      d.distance.health = SensorHealth::OFFLINE;
    }
    sensors::queue_diagnostic(
        "[TOF] sensor=%u address=0x%02x init=%s time=%lu ms bytes=%u\n", i,
        address, ok ? "ok" : "failed",
        static_cast<unsigned long>((micros() - start) / 1000),
        s.frame_bytes[i]);
    publish();
    vTaskDelay(1);
  }
  void poll() {
    MainSensorSnapshot imu;
    xQueuePeek(main_queue, &imu, 0);
    uint32_t now = micros();
    float pitch_deg =
        imu.imu_calib_valid &&
                is_imu_available(imu.imu, now, settings.imu_freshness_us)
            ? imu.imu.measurement.pitch - imu.imu_calib_flat_pitch
            : 0.0f;
    for (unsigned i = 0; i < 3; ++i) {
      auto &d = devices[i];
      if (!d.active || d.pending)
        continue;
      auto &io = *d.driver.VL53L5CX_i2c;
      io.clear_error();
      bool ready = d.driver.isDataReady();
      bool ok = !io.first_error && d.driver.lastError.lastErrorCode ==
                                       SF_VL53L5CX_ERROR_TYPE::NO_ERROR;
      uint32_t read_start = micros();
      bool complete = ready && ok && d.driver.getRangingData(&d.result);
      if (ready)
        s.max_read_us[i] = std::max(s.max_read_us[i], micros() - read_start);
      ok = ok && !io.first_error &&
           d.driver.lastError.lastErrorCode == SF_VL53L5CX_ERROR_TYPE::NO_ERROR;
      if (!ok || (ready && !complete)) {
        SensorError error = io.first_error == 6   ? SensorError::SHORT_READ
                            : io.first_error == 5 ? SensorError::TIMEOUT
                            : (io.first_error == 2 || io.first_error == 3)
                                ? SensorError::NACK
                                : SensorError::BUS_ERROR;
        fail(d, error);
        if (d.distance.consecutive_errors >= settings.max_consecutive_errors)
          d.pending = true;
      } else if (complete) {
        now = micros();
        d.result_us = now;
        ++s.frame_count[i];
        d.distance.consecutive_errors = 0;
        auto frame = process_forward_vl53l5cx(d.result, i, pitch_deg);
        auto &zones = s.zones[i];
        zones.pitch_deg = pitch_deg;
        zones.valid_zones = frame.valid_zones;
        zones.floor_zones = frame.floor_zones;
        zones.clearance_zones = frame.clearance_zones;
        zones.nearest_zone = frame.nearest_zone;
        for (unsigned r = 0; r < 4; ++r)
          for (unsigned c = 0; c < 4; ++c) {
            unsigned logical = r * 4 + c;
            unsigned raw = (config::tof_flip_y[i] ? 3 - r : r) * 4 +
                           (config::tof_flip_x[i] ? 3 - c : c);
            unsigned target = raw * VL53L5CX_NB_TARGET_PER_ZONE;
            zones.range_mm[logical] = d.result.distance_mm[target];
            zones.status[logical] = d.result.target_status[target];
            zones.targets[logical] = d.result.nb_target_detected[raw];
          }
        d.wall = frame.wall;
        d.distance.raw_timestamp_us = now;
        d.distance.raw_range_mm = frame.distance_mm;
        d.distance.raw_status = frame.valid ? (frame.open ? 255 : 5) : 0;
        if (frame.valid) {
          d.distance.distance_mm =
              std::max(config::sensor_min_raw,
                       std::min(config::sensor_max_raw, frame.distance_mm));
          sensor_policy::success(d.distance, now);
          if (frame.open)
            d.distance.health = SensorHealth::OUT_OF_RANGE;
        } else {
          d.distance.health = SensorHealth::RANGE_INVALID;
        }
        d.backoff.acquired(now);
      } else if (micros() - d.result_us >= settings.tof_freshness_us) {
        fail(d, SensorError::MEASUREMENT_TIMEOUT);
        d.pending = true;
      }
      if (d.pending) {
        d.active = false;
        hold(i);
        d.backoff.defer(millis());
        d.distance.health = SensorHealth::OFFLINE;
        d.wall.valid = false;
        publish();
      }
    }
    bool healthy = true;
    for (auto &d : devices)
      healthy &= d.active && !d.pending &&
                 micros() - d.result_us <= settings.tof_freshness_us;
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
        d.distance.health = SensorHealth::RECOVERING;
        d.wall.valid = false;
      }
      publish();
    }
  }
  void recover() {
    bool due = group_pending && bus_backoff.due(millis());
    if (!group_pending)
      for (auto &d : devices)
        due |= d.pending && d.backoff.due(millis());
    if (!due)
      return;
    // Epoch handshake prevents a previous zero-throttle acknowledgement from
    // authorizing a new blocking reload while the robot is driving.
    if (!request_pending) {
      if (++s.recovery_id == 0)
        ++s.recovery_id;
      request_pending = true;
      s.recovering = true;
      publish();
      return;
    }
    if (stopped_recovery_id.load() != s.recovery_id)
      return;
    for (auto &d : devices) {
      d.distance.health = SensorHealth::RECOVERING;
      d.wall.valid = false;
    }
    publish();
    if (group_pending) {
      for (unsigned i = 0; i < 3; ++i)
        hold(i);
      vTaskDelay(pdMS_TO_TICKS(10));
      restart_bus();
      // A failed device is held low, so subsequent devices can still
      // initialize.
      for (unsigned i = 0; i < 3; ++i) {
        init_device(i, true);
        // Keep assigned peers isolated too, until every address is unique.
        hold(i);
      }
      for (unsigned i = 0; i < 3; ++i)
        if (devices[i].active)
          pinMode(pin::lpn_tof[i], INPUT);
      group_pending = false;
      bus_backoff.defer(millis());
    } else {
      // One retry per pass; consume healthy-peer frames before the next retry.
      for (unsigned i = 0; i < 3; ++i) {
        if (devices[i].pending && devices[i].backoff.due(millis())) {
          init_device(i);
          break;
        }
      }
    }
    for (auto &d : devices) {
      if (d.active) {
        d.distance.health = SensorHealth::INITIALIZING;
        d.result_us =
            micros(); // Grace for the first consumed frame, not freshness.
      }
    }
    request_pending = false;
    s.recovering = false;
    publish();
  }
};
void tof_task(void *) {
  static ToFOwner owner;
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
  static_assert(std::is_trivially_copyable<MainSensorSnapshot>::value);
  static_assert(std::is_trivially_copyable<RosepinkToFSnapshot>::value);
  main_queue = xQueueCreate(1, sizeof(MainSensorSnapshot));
  tof_queue = xQueueCreate(1, sizeof(RosepinkToFSnapshot));
  command_queue = xQueueCreate(4, sizeof(SensorCommand));
  diagnostic_queue = xQueueCreate(40, sizeof(DiagnosticLine));
  if (!main_queue || !tof_queue || !command_queue || !diagnostic_queue)
    return false;
  MainSensorSnapshot main;
  RosepinkToFSnapshot tof;
  xQueueOverwrite(main_queue, &main);
  xQueueOverwrite(tof_queue, &tof);
  if (xTaskCreatePinnedToCore(
          main_sensor_task, "imu_task", config::sensor_task_stack_size, nullptr,
          config::sensor_task_priority, nullptr, 0) != pdPASS)
    return false;
  return xTaskCreatePinnedToCore(
             tof_task, "tof_task", config::sensor_task_stack_size, nullptr,
             config::sensor_task_priority, nullptr, 0) == pdPASS;
}
bool get_main_snapshot(MainSensorSnapshot &out) {
  return main_queue && xQueuePeek(main_queue, &out, 0) == pdTRUE;
}
bool get_tof_snapshot(RosepinkToFSnapshot &out) {
  return tof_queue && xQueuePeek(tof_queue, &out, 0) == pdTRUE;
}
void acknowledge_stopped(uint32_t recovery_id) {
  stopped_recovery_id.store(recovery_id);
}
void queue_diagnostic(const char *format, ...) {
  DiagnosticLine line = {};
  va_list args;
  va_start(args, format);
  vsnprintf(line.text, sizeof(line.text), format, args);
  va_end(args);
  if (diagnostic_queue)
    xQueueSend(diagnostic_queue, &line, 0);
}
void poll_diagnostics() {
  static uint32_t last_ms = 0;
  uint32_t now = millis();
  if (now - last_ms < config::diagnostic_line_interval_ms)
    return;
  last_ms = now;
  DiagnosticLine line;
  if (diagnostic_queue && xQueueReceive(diagnostic_queue, &line, 0) == pdTRUE)
    notify("%s", line.text);
}
void notify_init_diagnostics(const RosepinkToFSnapshot &snapshot) {
  for (unsigned i = 0; i < 3; ++i) {
    const auto &d = snapshot.initialization[i];
    queue_diagnostic(
        "[TOF] sensor=%u LPn=%u addr=0x%02x->0x%02x init_bus=%lu Hz pkt=%u "
        "run_bus=%lu Hz "
        "init=%s step=%s transport=%u driver_error=%u value=%lu time=%lu ms "
        "busy=%u\n",
        i, pin::lpn_tof[i], d.found_address, config::vl53l5cx_addresses[i],
        static_cast<unsigned long>(config::vl53l5cx_init_bus_hz),
        config::vl53l5cx_init_packet_bytes,
        static_cast<unsigned long>(config::vl53l5cx_bus_hz),
        !d.started    ? "not_started"
        : !d.complete ? "busy"
        : d.success   ? "ok"
                      : "failed",
        d.stage, d.transport_error, d.driver_error,
        static_cast<unsigned long>(d.driver_value),
        static_cast<unsigned long>(d.duration_ms), d.in_progress);
  }
}
bool send_command(const SensorCommand &cmd) {
  return command_queue && xQueueSend(command_queue, &cmd, 0) == pdTRUE;
}
} // namespace sensors
