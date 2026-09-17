#pragma once
#include <cstdint>

namespace config {
// Slew rates in normalized units per second (1.0 = 100% throttle / sec)
constexpr float throttle_slew_rate_decel = 3.0f;
constexpr float throttle_slew_rate_accel = 1.0f;

constexpr uint16_t start_delay_ms = 5000;
constexpr uint16_t reverse_duration_ms = 250;
constexpr uint16_t reverse_turn_duration_ms = 150;
constexpr float reverse_turn_steer = 0.50f;

// Sharp analog IR distance sensors
constexpr uint16_t sensor_min_raw = 25;
constexpr uint16_t sensor_max_raw = 3800;
constexpr float sensor_range =
    static_cast<float>(sensor_max_raw - sensor_min_raw);

// VL53L0X Time-of-Flight sensor
constexpr uint16_t tof_timeout_ms = 100;
constexpr uint16_t tof_min_mm = 30;
constexpr uint16_t tof_max_mm = 1100;

// Sensible defaults if not set in NVS
constexpr uint16_t default_dist_reverse = 150; // mm
constexpr uint16_t default_dist_far = 400;     // mm
constexpr float default_base_throttle = 0.50f;
constexpr float default_straight_throttle = 0.65f;
constexpr float default_reverse_throttle = 0.40f;
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
