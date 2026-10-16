#pragma once
#include <stdint.h>

enum class RunningState : uint8_t {
  IDLE,
  COUNTDOWN,
  RUNNING,
  REVERSE,
  REVERSE_TURN,
  WRONG_WAY_RECOVERY,
};

inline const char *running_state_to_str(RunningState s) {
  switch (s) {
  case RunningState::IDLE:
    return "IDLE";
  case RunningState::COUNTDOWN:
    return "COUNTDOWN";
  case RunningState::RUNNING:
    return "RUNNING";
  case RunningState::REVERSE:
    return "REVERSE";
  case RunningState::REVERSE_TURN:
    return "REVERSE_TURN";
  case RunningState::WRONG_WAY_RECOVERY:
    return "WRONG_WAY_RECOVERY";
  default:
    return "UNKNOWN";
  }
}

enum class TrackDirection : uint8_t {
  COUNTER_CLOCKWISE = 0,
  CLOCKWISE = 1,
};

inline const char *track_direction_to_str(TrackDirection d) {
  switch (d) {
  case TrackDirection::COUNTER_CLOCKWISE:
    return "CCW";
  case TrackDirection::CLOCKWISE:
    return "CW";
  default:
    return "UNKNOWN";
  }
}

struct PIDState {
  float Kp;
  float Ki;
  float Kd;

  float error;
  float last_error;
  float integral;
  float derivative;

  float offset;
};

struct IMUState {
  float accel_x;
  float accel_y;
  float accel_z;

  float gyro_x;
  float gyro_y;
  float gyro_z;

  float roll;
  float pitch;
  float yaw;
};

enum class SensorHealth : uint8_t {
  ABSENT,        // Robot does not populate this distance/IMU slot
  INITIALIZING,  // Initializing or waiting for first sample
  OK,            // Actively acquiring valid samples
  STALE,         // Sample exceeded freshness limit
  READ_ERROR,    // Transport or device hardware failure
  OFFLINE,       // Unavailable: recovery backoff or IMU disabled until reboot
  RECOVERING,    // Bus reset or XSHUT sequencing in progress
  RANGE_INVALID, // Device returned a result, but distance is unreliable
  OUT_OF_RANGE,  // Fresh ToF result at/beyond configured maximum distance
};

inline const char *sensor_health_to_str(SensorHealth h) {
  switch (h) {
  case SensorHealth::ABSENT:
    return "ABSENT";
  case SensorHealth::INITIALIZING:
    return "INIT";
  case SensorHealth::OK:
    return "OK";
  case SensorHealth::STALE:
    return "STALE";
  case SensorHealth::READ_ERROR:
    return "ERROR";
  case SensorHealth::OFFLINE:
    return "OFFLINE";
  case SensorHealth::RECOVERING:
    return "RECOVER";
  case SensorHealth::RANGE_INVALID:
    return "RANGE_INVALID";
  case SensorHealth::OUT_OF_RANGE:
    return "OUT_OF_RANGE";
  default:
    return "UNKNOWN";
  }
}

enum class SensorError : uint8_t {
  NONE,
  NACK,
  TIMEOUT,
  SHORT_READ,
  BUS_ERROR,
  INIT_FAILED,
  CONFIG_FAILED,
  MEASUREMENT_TIMEOUT,
  HARDWARE_ERROR,
};

enum class SensorSlot : uint8_t {
  FRONT = 0,
  RIGHT = 1,
  LEFT = 2,
  SIDE_FRONT_LEFT = 3,
  SIDE_REAR_LEFT = 4,
  SIDE_FRONT_RIGHT = 5,
  SIDE_REAR_RIGHT = 6,
  IMU = 7,
  COUNT = 8,
};

struct SensorStatus {
  uint32_t timestamp_us =
      0; // Last usable sample (measured or out of range)
  SensorHealth health = SensorHealth::INITIALIZING;
  uint32_t error_count = 0;
  uint16_t consecutive_errors = 0;
  SensorError last_error = SensorError::NONE;
};

struct SensorData : SensorStatus {
  SensorData() { health = SensorHealth::RECOVERING; }
  uint16_t distance_mm = 0;
  uint16_t raw_range_mm = 0; // Unclamped VL53L0X result, including sentinels
  uint8_t raw_status = 0;    // Full RESULT_RANGE_STATUS register
  uint32_t raw_timestamp_us = 0; // Zero until the first consumed result
};

struct IMUSensorData : SensorStatus {
  IMUState measurement = {};
};

inline bool is_sensor_fresh(uint32_t timestamp_us, uint32_t now_us,
                            uint32_t max_age_us) {
  return (timestamp_us != 0) &&
         (static_cast<uint32_t>(now_us - timestamp_us) <= max_age_us);
}

inline bool is_sensor_available(const SensorData &s, uint32_t now_us,
                                uint32_t max_age_us) {
  return (s.health == SensorHealth::OK ||
          s.health == SensorHealth::OUT_OF_RANGE) &&
         is_sensor_fresh(s.timestamp_us, now_us, max_age_us);
}

inline bool is_imu_available(const IMUSensorData &s, uint32_t now_us,
                             uint32_t max_age_us) {
  return (s.health == SensorHealth::OK) &&
         is_sensor_fresh(s.timestamp_us, now_us, max_age_us);
}

struct RobotState {
  RunningState running_state;
  uint32_t current_time_ms;
  uint32_t current_time_us;
  uint32_t maneuver_start_us;
  SensorData dist_front;
  SensorData dist_right;
  SensorData dist_left;
  SensorData dist_side_front_left;
  SensorData dist_side_rear_left;
  SensorData dist_side_front_right;
  SensorData dist_side_rear_right;
  uint16_t dist_far;
  uint16_t dist_reverse;
  float target_steering;
  float current_steering;
  float target_throttle;
  float current_throttle;

  float base_throttle;
  float straight_throttle;
  float reverse_throttle;

  PIDState steering_pid;
  IMUSensorData imu;

  TrackDirection track_direction;
};
