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
constexpr uint16_t reverse_trigger_delay_ms = 400;

constexpr uint16_t sensor_min_raw = 30;
constexpr uint16_t sensor_max_raw = 1000;
constexpr float sensor_range =
    static_cast<float>(sensor_max_raw - sensor_min_raw);

// Cornering and obstacle navigation parameters
constexpr uint16_t front_turn_dist = 500;       // mm: distance to start corner turn assist
constexpr uint16_t front_critical_dist = 180;   // mm: distance where front demands full-lock turn
constexpr uint16_t side_danger_dist = 140;      // mm: emergency side wall repulsion
constexpr float side_turn_range = 350.0f;       // mm: lateral difference for full side correction
constexpr uint16_t default_dist_reverse = 110;  // mm: fallback obstacle distance if unset in prefs
constexpr uint32_t tof_timing_budget_us = 20000; // us: 20ms timing budget (50Hz)

constexpr float wrong_way_yaw_threshold_deg = 210.0f;
constexpr uint16_t wrong_way_debounce_ms = 400;
constexpr uint16_t wrong_way_turnaround_timeout_ms = 2500;
constexpr float wrong_way_resume_threshold_deg = 100.0f;
} // namespace config

namespace pin {
constexpr uint8_t esc = 2;
constexpr uint8_t servo = 1;

constexpr uint8_t xshut_left = 42;
constexpr uint8_t xshut_right = 41;

constexpr uint8_t sda = 4;
constexpr uint8_t scl = 5;

constexpr uint8_t sda_side = 6;
constexpr uint8_t scl_side = 7;

constexpr uint8_t xshut_front_left = 40;
constexpr uint8_t xshut_front_right = 39;
constexpr uint8_t xshut_rear_right = 38;

constexpr uint8_t direction_switch = 21;
} // namespace pin
