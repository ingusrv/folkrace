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
constexpr uint8_t servo_deg_right = 140;
constexpr uint8_t servo_deg_left = 40;

constexpr uint16_t start_delay_ms = 5000;

// Reverse and recovery timings
// ms: maximum straight backup after the reverse command begins.
constexpr uint16_t reverse_duration_ms = 200;
constexpr uint16_t reverse_min_duration_ms = 100;  // ms: before clearance exit
constexpr uint16_t reverse_turn_duration_ms = 250; // ms: turn time, maximum
constexpr uint16_t reverse_total_timeout_ms = 900; // ms: includes throttle ramp
// ms: confirm a close obstacle while forward throttle ramps toward zero.
constexpr uint16_t reverse_trigger_delay_ms = 60;
constexpr float reverse_max_yaw_deg = 30.0f; // deg: total recovery rotation
constexpr float reverse_motion_throttle = 0.03f; // commanded reverse magnitude
constexpr uint16_t reverse_clearance_margin_mm = 100;

// Stall detection while commanding forward drive.
constexpr uint32_t stall_timeout_ms = 2000;        // ms: no significant movement
constexpr uint16_t stall_distance_change_mm = 30;  // mm: half-window mean change
constexpr float stall_rotation_change_deg = 15.0f; // deg: mean yaw change
constexpr uint8_t stall_min_valid_percent = 80;    // %: real returns in each half
constexpr float stall_min_throttle = 0.03f;        // normalized forward command

// Escalate repeated short reverses into a timed rocking maneuver.
constexpr uint8_t escape_reverse_attempts = 5;
constexpr uint32_t escape_clear_forward_ms = 500; // ms: clear attempt history
constexpr uint32_t escape_backup_ms = 700;        // ms: straight reverse command
constexpr uint32_t escape_reverse_turn_ms = 650;  // ms: reverse at full lock
constexpr uint32_t escape_forward_ms = 600;       // ms: forward rock at full lock
constexpr uint32_t escape_timeout_ms = 4000;      // ms: includes throttle ramps

// Avoidance pulse parameters
constexpr uint16_t avoidance_direction_margin_mm = 60; // ignore gap noise
constexpr uint16_t avoidance_max_ms = 180;             // ms: one front-avoidance pulse
constexpr float avoidance_max_yaw_deg = 18.0f;         // deg: hand back to wall following
constexpr uint16_t avoidance_release_margin_mm = 50;   // mm: rearm hysteresis

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
} // namespace config

namespace pin {
constexpr uint8_t bi = 5;
constexpr uint8_t fi = 6;

constexpr uint8_t servo = 2;

constexpr uint8_t xshut_front = 3;
constexpr uint8_t xshut_left = 0;
constexpr uint8_t xshut_right = 1;

constexpr uint8_t sda = 8;
constexpr uint8_t scl = 9;
} // namespace pin