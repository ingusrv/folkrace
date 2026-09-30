#pragma once
#include "state.h"
#include <stdint.h>

enum class PacketType : uint8_t {
  TELEMETRY = 0,
  CONFIG,
};

enum class RobotId : uint8_t {
  UNKNOWN = 0,
  SNIEGBALTITE = 1,
  ROSEPINK = 2,
  BANANA = 3,
  NAKTS = 4,
  ALL = 255,
};

inline const char *robot_id_to_str(RobotId id) {
  switch (id) {
  case RobotId::SNIEGBALTITE:
    return "sniegbaltite";
  case RobotId::ROSEPINK:
    return "rosepink";
  case RobotId::BANANA:
    return "banana";
  case RobotId::NAKTS:
    return "nakts";
  case RobotId::ALL:
    return "all";
  default:
    return "unknown";
  }
}

struct __attribute__((packed)) PacketHeader {
  uint32_t magic;
  PacketType type;
  RobotId robot_id;
  uint16_t length;
};

struct __attribute__((packed)) TelemetryPacket {
  uint32_t timestamp_ms;
  RunningState running_state;
  uint8_t flags;

  uint16_t dist_front;
  uint16_t dist_right;
  uint16_t dist_left;

  uint16_t dist_side_front_left;
  uint16_t dist_side_rear_left;
  uint16_t dist_side_front_right;
  uint16_t dist_side_rear_right;

  float target_steering;
  float current_steering;
  float target_throttle;
  float current_throttle;

  float pid_error;
  float pid_integral;
  float pid_derivative;

  // IMU fields (for future usage)
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
