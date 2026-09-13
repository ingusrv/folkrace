#pragma once
#include <stdint.h>

namespace config {
constexpr float steering_slew_rate = 20.0f;
constexpr float throttle_slew_rate_decel = 6.0f;
constexpr float throttle_slew_rate_accel = 2.0f;

constexpr uint8_t servo_deg_middle = 90;
constexpr uint8_t servo_deg_right = 130;
constexpr uint8_t servo_deg_left = 50;

constexpr uint16_t start_delay_ms = 5000;
constexpr uint16_t reverse_duration_ms = 400;
constexpr uint16_t reverse_turn_duration_ms = 500;
constexpr uint16_t reverse_trigger_delay_ms = 1000;

constexpr uint16_t sensor_min_raw = 25;
constexpr uint16_t sensor_max_raw = 3800;
constexpr float sensor_range = static_cast<float>(sensor_max_raw - sensor_min_raw);
} // namespace config

namespace pin {
constexpr uint8_t esc = 10;
constexpr uint8_t servo = 2;

constexpr uint8_t sda = 8;
constexpr uint8_t scl = 9;

constexpr uint8_t right_sensor = 0;
constexpr uint8_t left_sensor = 1;
constexpr uint8_t mid_sensor = 3;
} // namespace pin
