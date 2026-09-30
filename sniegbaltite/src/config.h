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
constexpr uint16_t front_turn_dist =
    500; // mm: distance to start corner turn assist
constexpr uint16_t front_critical_dist =
    180; // mm: distance where front demands full-lock turn
constexpr uint16_t side_danger_dist = 140; // mm: emergency side wall repulsion
constexpr float side_turn_range =
    350.0f; // mm: lateral difference for full side correction
constexpr uint16_t default_dist_reverse =
    110; // mm: fallback obstacle distance if unset in prefs
constexpr uint32_t tof_timing_budget_us =
    20000; // us: 20ms timing budget (50Hz)

// Stanley controller parameters
constexpr float stanley_sensor_baseline_mm =
    110.0f; // mm: distance between front & rear side sensors
constexpr float stanley_k_heading = 1.0f; // Gain for heading error
constexpr float stanley_k_cte = 2.2f;     // Cross-track error gain (1/m)
constexpr float stanley_k_soft = 0.25f;   // Velocity softening term
constexpr float stanley_k_d = 0.04f;      // Yaw rate damping gain
constexpr float stanley_max_steer_rad = 0.69813f; // 40 degrees in radians
constexpr float stanley_target_wall_dist_mm =
    200.0f; // mm: target distance when single-wall tracking
constexpr float stanley_max_valid_dist_mm =
    600.0f; // mm: max distance to trust side wall
constexpr float stanley_max_delta_mm =
    110.0f; // mm: max |d_front - d_rear| for flat wall
constexpr float stanley_heading_ema_alpha =
    0.35f; // EMA alpha for heading angle

constexpr float wrong_way_yaw_threshold_deg = 210.0f;
constexpr uint16_t wrong_way_debounce_ms = 400;
constexpr uint16_t wrong_way_turnaround_timeout_ms = 2500;
constexpr float wrong_way_resume_threshold_deg = 100.0f;

// Slope / hill detection parameters
constexpr float slope_pitch_threshold_deg =
    10.0f; // deg: pitch deviation to detect hill/ramp
constexpr uint16_t slope_hold_ms =
    600; // ms: hold slope state after leveling to avoid crest launch

constexpr uint8_t rgb_led_brightness = 32;
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

constexpr uint8_t rgb_led_pin = 48;
} // namespace pin
