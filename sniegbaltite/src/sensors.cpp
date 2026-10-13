#include "sensors.h"
#include "config.h"
#include "sensors/main_owner.h"
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
      Wire, pin::sda, pin::scl, pin::xshut_left, pin::xshut_right,
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
struct SideOwner {
  sensor_policy::ToFBus bus{Wire1,
                            pin::sda_side,
                            pin::scl_side,
                            config::sensor_min_raw,
                            config::sensor_max_raw,
                            config::tof_timing_budget_us,
                            settings};
  SideSensorSnapshot s;
  void publish() {
    s.dist_side_rear_left = bus.devices[0].data;
    s.dist_side_front_left = bus.devices[1].data;
    s.dist_side_front_right = bus.devices[2].data;
    s.dist_side_rear_right = bus.devices[3].data;
    s.publish_timestamp_us = micros();
    xQueueOverwrite(side_queue, &s);
  }
};
void side_tof_task(void *) {
  SideOwner owner;
  auto &bus = owner.bus;
  bus.count = 4;
  bus.devices[0].address = 0x30;
  bus.devices[1].address = 0x31;
  bus.devices[1].xshut = pin::xshut_front_left;
  bus.devices[2].address = 0x32;
  bus.devices[2].xshut = pin::xshut_front_right;
  bus.devices[3].address = 0x33;
  bus.devices[3].xshut = pin::xshut_rear_right;
  bus.set_publisher([](void *ctx) { static_cast<SideOwner *>(ctx)->publish(); },
                    &owner);
  bus.begin();
  TickType_t wake = xTaskGetTickCount();
  uint32_t previous_start_us = micros();
  for (;;) {
    uint32_t start = micros();
    owner.s.max_period_us =
        std::max(owner.s.max_period_us, start - previous_start_us);
    previous_start_us = start;
    bus.poll();
    owner.publish();
    bus.recover();
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
