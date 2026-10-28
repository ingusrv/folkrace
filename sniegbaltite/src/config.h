#pragma once
#include <stdint.h>

namespace config {
// LiPo protection: select 2 for 2S or 3 for 3S; cutoff stays latched until
// reboot.
constexpr uint8_t battery_cells = 3;
static_assert(battery_cells == 2 || battery_cells == 3,
              "Select a 2S or 3S LiPo pack");
constexpr uint16_t battery_cutoff_per_cell_mv = 3600; // mV per cell
constexpr uint32_t battery_cutoff_mv =
    battery_cells * battery_cutoff_per_cell_mv;
// 10k from battery to GPIO, 2.2k to ground, 0.1uF across the 2.2k.
constexpr float battery_divider_ratio = (10000.0f + 2200.0f) / 2200.0f;
constexpr uint32_t battery_sample_interval_ms = 50;
constexpr uint32_t battery_cutoff_confirm_ms = 500;
constexpr uint32_t battery_low_blink_interval_ms = 500; // Each on/off phase

// Sensor freshness limits
constexpr uint32_t sensor_freshness_imu_us = 50000;  // 50 ms max age
constexpr uint32_t sensor_freshness_tof_us = 100000; // 100 ms max age
constexpr uint16_t sensor_max_consecutive_errors = 3;

// Recovery timings
constexpr uint32_t sensor_recovery_delay_initial_ms = 500;
constexpr uint32_t sensor_recovery_delay_max_ms = 5000;
constexpr uint32_t sensor_recovery_healthy_ms = 2000;
constexpr uint32_t sensor_bus_timeout_ms = 25;
constexpr uint32_t tof_timeout_ms = 100;
constexpr uint32_t tof_resume_grace_us = 100000;      // No-result recovery only
constexpr uint8_t imu_max_init_attempts_per_boot = 3; // Includes startup

// IMU OFFLINE indication (also add to Rosepink's config)
constexpr uint8_t imu_offline_led_r = 128;
constexpr uint8_t imu_offline_led_g = 32;
constexpr uint8_t imu_offline_led_b = 64;
constexpr uint32_t imu_offline_blink_interval_ms = 250; // Each on/off phase

// Initial stationary IMU calibration only
constexpr uint16_t imu_min_calib_samples = 50;
constexpr uint32_t imu_calib_extension_ms = 5000;

// FreeRTOS tasks
constexpr uint32_t sensor_task_stack_size = 4096;
constexpr uint32_t sensor_task_priority = 2;

constexpr float steering_slew_rate = 14.0f;
// Slew rates in normalized full-scale units per second (1.0 = 100% throttle /
// sec). e.g., 2.0f means 0% to 100% takes 0.5s; 6.0f means 100% to 0% takes
// ~0.17s.
constexpr float throttle_slew_rate_decel = 3.0f;
constexpr float throttle_slew_rate_accel = 0.6f;
constexpr float throttle_slew_rate_reverse_accel = 1.0f;

constexpr uint8_t servo_deg_middle = 90;
constexpr uint8_t servo_deg_right = 130;
constexpr uint8_t servo_deg_left = 50;

constexpr uint16_t start_delay_ms = 5000;
// ms: maximum straight backup after the reverse command begins.
constexpr uint16_t reverse_duration_ms = 200;
constexpr uint16_t reverse_min_duration_ms = 100;  // ms: before clearance exit
constexpr uint16_t reverse_turn_duration_ms = 250; // ms: turn time, maximum
constexpr uint16_t reverse_total_timeout_ms = 900; // ms: includes throttle ramp
// ms: confirm a close obstacle while forward throttle ramps toward zero.
constexpr uint16_t reverse_trigger_delay_ms = 500;
// Stall detection while commanding forward drive.
constexpr uint32_t stall_timeout_ms = 2000;       // ms: no significant movement
constexpr uint16_t stall_distance_change_mm = 30; // mm: half-window mean change
constexpr float stall_rotation_change_deg = 15.0f; // deg: mean yaw change
constexpr uint8_t stall_min_valid_percent = 80; // %: real returns in each half
constexpr float stall_min_throttle = 0.03f;     // normalized forward command
// Escalate repeated short reverses into a timed rocking maneuver.
constexpr uint8_t escape_reverse_attempts = 5;
constexpr uint32_t escape_clear_forward_ms = 500; // ms: clear attempt history
constexpr uint32_t escape_backup_ms = 700;       // ms: straight reverse command
constexpr uint32_t escape_reverse_turn_ms = 650; // ms: reverse at full lock
constexpr uint32_t escape_forward_ms = 600;  // ms: forward rock at full lock
constexpr uint32_t escape_timeout_ms = 4000; // ms: includes throttle ramps
constexpr float reverse_max_yaw_deg = 30.0f; // deg: total recovery rotation
constexpr float reverse_motion_throttle = 0.03f; // commanded reverse magnitude
constexpr uint16_t reverse_clearance_margin_mm = 100;
constexpr uint16_t avoidance_direction_margin_mm = 60; // ignore gap noise
constexpr uint16_t avoidance_max_ms = 180; // ms: one front-avoidance pulse
constexpr float avoidance_max_yaw_deg =
    18.0f; // deg: hand back to wall following
constexpr uint16_t avoidance_release_margin_mm = 50; // mm: rearm hysteresis

constexpr uint16_t sensor_min_raw = 30;
constexpr uint16_t sensor_max_raw = 1000;
constexpr float sensor_range =
    static_cast<float>(sensor_max_raw - sensor_min_raw);

// Cornering and obstacle navigation parameters
constexpr uint16_t front_turn_dist = 550;
// mm: distance where front demands full-lock turn
constexpr uint16_t front_critical_dist = 180;
// mm: emergency side wall repulsion
constexpr uint16_t side_danger_dist = 160;
// mm: lateral difference for full side correction
constexpr float side_turn_range = 350.0f;
// mm: fallback obstacle distance if unset in prefs
constexpr uint16_t default_dist_reverse = 110;
// us: 10ms timing budget (100Hz), VL53L4CD front
constexpr uint32_t front_tof_timing_budget_us = 10000;
// us: 20ms timing budget (50Hz), VL53L0X side
constexpr uint32_t side_tof_timing_budget_us = 20000;
constexpr float wrong_way_yaw_threshold_deg = 210.0f;
constexpr uint16_t wrong_way_debounce_ms = 400;
constexpr uint16_t wrong_way_turnaround_timeout_ms = 2500;
constexpr float wrong_way_resume_threshold_deg = 100.0f;

// Slope / hill detection parameters
// deg: pitch deviation to detect hill/ramp
constexpr float slope_pitch_threshold_deg = 10.0f;
// ms: hold slope state after leveling to avoid crest launch
constexpr uint16_t slope_hold_ms = 600;

constexpr uint8_t rgb_led_brightness = 32;
} // namespace config

namespace pin {
constexpr uint8_t battery_adc = 15;
constexpr uint8_t esc = 2;
constexpr uint8_t servo = 1;

constexpr uint8_t xshut_front = 16;
constexpr uint8_t xshut_left = 42;
constexpr uint8_t xshut_right = 41;

constexpr uint8_t sda = 4;
constexpr uint8_t scl = 5;

constexpr uint8_t sda_side = 6;
constexpr uint8_t scl_side = 7;

constexpr uint8_t xshut_front_left = 40;
constexpr uint8_t xshut_rear_left = 17;
constexpr uint8_t xshut_front_right = 39;
constexpr uint8_t xshut_rear_right = 38;

constexpr uint8_t direction_switch = 21;

constexpr uint8_t rgb_led_pin = 48;
} // namespace pin
