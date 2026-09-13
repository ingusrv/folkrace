#pragma once
#include <stdint.h>

enum class PacketType : uint8_t {
  TELEMETRY = 0,
  CONFIG,
};

struct __attribute__((packed)) PacketHeader {
  uint32_t magic;
  PacketType type;
  uint16_t length;
};

struct __attribute__((packed)) TelemetryPacket {
  uint32_t timestamp_ms;
  uint16_t dist_front;
  uint16_t dist_right;
  uint16_t dist_left;

  float target_steering;
  float current_steering;
  float target_throttle;
  float current_throttle;
};
