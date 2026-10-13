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

constexpr uint8_t TELEMETRY_PROTOCOL_VERSION = 1;

struct __attribute__((packed)) SensorDiagnostic {
  uint8_t health;       // SensorHealth (1 byte)
  uint16_t age_ms;      // 0xFFFF = no sample, saturated at 65534 (2 bytes)
  uint16_t error_count; // Saturated (2 bytes)
  uint16_t consecutive_errors; // Saturated (2 bytes)
  uint8_t last_error;          // SensorError (1 byte)
}; // Total: 8 bytes per slot

struct __attribute__((packed)) RawTofDiagnostic {
  uint16_t range_mm; // Unclamped VL53L0X result
  uint8_t status;    // Full RESULT_RANGE_STATUS byte
  uint16_t age_ms;   // Raw result age; 0xFFFF = no raw sample
}; // Total: 5 bytes per distance slot

struct __attribute__((packed)) TelemetryPacketV1 {
  uint8_t protocol_version; // TELEMETRY_PROTOCOL_VERSION = 1
  uint32_t timestamp_ms;
  RunningState running_state;
  uint8_t flags; // Bit 0: obstacle, Bit 1: BLE, Bit 2: CW, Bit 3: required
                 // sensors OK, Bit 4: slope

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

  float accel_x;
  float accel_y;
  float accel_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float roll;
  float pitch;
  float yaw;

  SensorDiagnostic sensors[8]; // Order: FRONT, RIGHT, LEFT, SIDE_FL, SIDE_RL,
                               // SIDE_FR, SIDE_RR, IMU
  RawTofDiagnostic raw_tof[7]; // Same distance-slot order, excludes IMU
};

static_assert(sizeof(SensorDiagnostic) == 8, "SensorDiagnostic size mismatch");
static_assert(sizeof(RawTofDiagnostic) == 5, "RawTofDiagnostic size mismatch");
static_assert(sizeof(TelemetryPacketV1) == 184,
              "TelemetryPacketV1 size mismatch");

using TelemetryPacket = TelemetryPacketV1;
static_assert(sizeof(PacketHeader) == 8, "PacketHeader size mismatch");
static_assert(sizeof(PacketHeader) + sizeof(TelemetryPacketV1) <= 250,
              "telemetry must fit ESP-NOW v1");
