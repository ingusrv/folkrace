#pragma once
#include "state.h"
#include <stdint.h>

// Calibration commands apply strictly to the IMU (gyro zero-rate bias & flat
// pitch). ToF sensors are factory-calibrated and do not undergo runtime
// calibration. If ToF calibration is ever required in the future, dedicated
// command types will be added.
enum class SensorCommandType : uint8_t {
  START_IMU_CALIBRATION,
  FINISH_IMU_CALIBRATION,
  ABORT_IMU_CALIBRATION,
};

enum class SensorCommandResult : uint8_t {
  NONE,
  ACCEPTED,
  COMPLETED,
  INSUFFICIENT_SAMPLES,
  ABORTED,
  REJECTED,
};

struct SensorCommand {
  uint32_t cmd_id = 0;
  SensorCommandType type = SensorCommandType::START_IMU_CALIBRATION;
};

struct MainSensorSnapshot {
  uint32_t publish_timestamp_us = 0;
  uint32_t task_stack_min_bytes = 0;
  uint32_t max_tick_us = 0;
  uint32_t max_period_us = 0;
  uint32_t orientation_generation = 0;
  uint32_t last_imu_init_us = 0;
  IMUSensorData imu = {};
  bool imu_disabled_until_reboot = false;
  SensorData dist_front = {};
  SensorData dist_left = {};
  SensorData dist_right = {};

  // IMU calibration acknowledgement & status
  uint32_t last_cmd_id_ack = 0;
  SensorCommandResult last_cmd_result = SensorCommandResult::NONE;
  uint32_t imu_calib_start_cmd_id = 0;
  bool imu_calib_in_progress = false;
  bool imu_calib_completed = false; // Completion of the current attempt
  bool imu_calib_valid = false;     // Committed initial calibration exists
  uint16_t imu_calib_sample_count = 0;
  float imu_calib_flat_pitch = 0.0f;
};

struct SideSensorSnapshot {
  uint32_t publish_timestamp_us = 0;
  uint32_t task_stack_min_bytes = 0;
  uint32_t max_tick_us = 0;
  uint32_t max_period_us = 0;
  SensorData dist_side_front_left = {};
  SensorData dist_side_rear_left = {};
  SensorData dist_side_front_right = {};
  SensorData dist_side_rear_right = {};
};
