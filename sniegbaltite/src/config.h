#pragma once
#include <stdint.h>

namespace config {
constexpr float steering_slew_rate = 14.0f;
// Slew rates in normalized full-scale units per second (1.0 = 100% throttle /
// sec). e.g., 2.0f means 0% to 100% takes 0.5s; 6.0f means 100% to 0% takes
// ~0.17s.
constexpr float throttle_slew_rate_decel = 3.0f;
constexpr float throttle_slew_rate_accel = 1.0f;

constexpr uint8_t servo_deg_middle = 90;
constexpr uint8_t servo_deg_right = 130;
constexpr uint8_t servo_deg_left = 50;

constexpr uint16_t start_delay_ms = 5000;
constexpr uint16_t reverse_duration_ms = 400;
constexpr uint16_t reverse_turn_duration_ms = 600;
constexpr uint16_t reverse_trigger_delay_ms = 1000;

constexpr uint16_t sensor_min_raw = 25;
constexpr uint16_t sensor_max_raw = 3800;
constexpr float sensor_range =
    static_cast<float>(sensor_max_raw - sensor_min_raw);

constexpr float wrong_way_yaw_threshold_deg = 210.0f;
constexpr uint16_t wrong_way_debounce_ms = 400;
constexpr uint16_t wrong_way_turnaround_timeout_ms = 2500;
constexpr float wrong_way_resume_threshold_deg = 100.0f;
} // namespace config

namespace pin {
constexpr uint8_t esc = 10;
constexpr uint8_t servo = 2;

constexpr uint8_t sda = 8;
constexpr uint8_t scl = 9;

constexpr uint8_t right_sensor = 0;
constexpr uint8_t left_sensor = 1;
constexpr uint8_t mid_sensor = 3;

constexpr uint8_t direction_switch = 4;
} // namespace pin
