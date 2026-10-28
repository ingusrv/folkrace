#pragma once
#include <cstdint>

namespace config {
// Slew rates in normalized units per second (1.0 = 100% throttle / sec)
constexpr float throttle_slew_rate_decel = 3.0f;
constexpr float throttle_slew_rate_accel = 1.0f;
// Per-wheel slew after mixing; must be fast so steering and reverse maneuvers
// are not throttled by the (slow) forward acceleration limit.
constexpr float wheel_slew_rate_accel = 6.0f;
constexpr float wheel_slew_rate_decel = 10.0f;

constexpr uint16_t start_delay_ms = 5000;
constexpr uint16_t reverse_duration_ms = 350;
constexpr uint16_t reverse_turn_duration_ms = 100; // Shorter pivot to prevent over-rotation
constexpr uint16_t reverse_trigger_delay_ms = 250;
constexpr float reverse_turn_steer = 0.30f; // Calmer pivot turn authority (prevents reverse oversteer)
constexpr float reverse_steer_factor = 0.6f; // Fraction of reverse_turn_steer applied while backing up
constexpr float motor_min_throttle = 0.18f;

// Stall detection (no encoders): stuck if all sensors stay within these bands
// for the detect time while driving forward. Signals are smoothed by a slow
// EMA first so sensor noise doesn't keep resetting the stall window.
// When the front ToF sees something (< cap) it reliably proves motion, so a
// short window is enough; when it sees open space only the IRs can prove
// motion, which also happens on straights / sweeping turns -> longer window.
constexpr uint16_t stall_detect_ms = 600;
constexpr uint16_t stall_detect_blind_ms = 1200;
constexpr float stall_min_throttle = 0.05f;  // only check above this throttle
constexpr float stall_filter_alpha = 0.08f;  // slow EMA (~120 ms @ 100 Hz)
constexpr float stall_front_cap_mm = 950.0f; // mm: "front sees open space"
constexpr float stall_front_delta_mm = 30.0f; // mm: front ToF movement band
constexpr float stall_ir_delta_adc = 80.0f;   // ADC: Sharp IR movement band

// Sharp analog IR distance sensors
constexpr uint16_t sensor_min_raw = 25;
constexpr uint16_t sensor_max_raw = 3800;
constexpr float sensor_range =
    static_cast<float>(sensor_max_raw - sensor_min_raw);
constexpr float sensor_ema_alpha = 0.30f;
constexpr float side_turn_range = 800.0f; // 800 ADC diff for 1.0 error (improves mid-curve bite)

// Low-pass filter alpha for derivative (0.25 for minimal phase lag)
constexpr float derivative_filter_alpha = 0.25f;

// VL53L0X Time-of-Flight sensor
constexpr uint16_t tof_timeout_ms = 100;
constexpr uint32_t tof_timing_budget_us =
    20000; // us: 20ms timing budget (50Hz)
constexpr uint16_t tof_min_mm = 30;
constexpr uint16_t tof_max_mm = 1000;
} // namespace config

namespace pin {
constexpr uint8_t left_bi = 5;
constexpr uint8_t left_fi = 6;

constexpr uint8_t right_bi = 10;
constexpr uint8_t right_fi = 7;

constexpr uint8_t left_sensor = 1;
constexpr uint8_t right_sensor = 0;

constexpr uint8_t scl = 9;
constexpr uint8_t sda = 8;
} // namespace pin
