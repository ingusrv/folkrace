#pragma once
#include <stdint.h>

enum class RunningState : uint8_t {
  IDLE,
  COUNTDOWN,
  RUNNING,
  REVERSE,
  REVERSE_TURN,
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

struct RobotState {
  RunningState running_state;
  uint32_t current_time_ms;
  uint32_t current_time_us;
  uint32_t maneuver_start_us;
  uint16_t dist_front;
  uint16_t dist_right;
  uint16_t dist_left;
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
  IMUState imu;
};