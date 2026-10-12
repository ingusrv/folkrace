#pragma once
#if __has_include(<BMI088.h>)
#include <Arduino.h>
#include <Wire.h>
#include <BMI088.h>
#include "state.h"

class BMI088Driver {
public:
  BMI088Driver(TwoWire &bus = Wire, uint8_t accel_addr = 0x19, uint8_t gyro_addr = 0x69);

  bool begin();

  // Calibration routines
  void start_calibration();
  void update_calibration();
  void finish_calibration();
  void calibrate(uint16_t samples = 200, uint16_t delay_ms = 5);

  // Periodic sensor read & fusion
  void update();
  void update(IMUState &state);

  const IMUState &get_state() const { return m_state; }
  void reset_yaw(float yaw = 0.0f) { m_state.yaw = yaw; }
  void set_alpha(float alpha) { m_alpha = alpha; }

  bool is_initialized() const { return m_initialized; }
  bool is_calibrated() const { return m_calibrated; }

  float get_gyro_bias_x() const { return m_gyro_bias_x; }
  float get_gyro_bias_y() const { return m_gyro_bias_y; }
  float get_gyro_bias_z() const { return m_gyro_bias_z; }

private:
  Bmi088Accel m_accel;
  Bmi088Gyro m_gyro;

  IMUState m_state = {};

  float m_gyro_bias_x = 0.0f;
  float m_gyro_bias_y = 0.0f;
  float m_gyro_bias_z = 0.0f;

  float m_calib_sum_x = 0.0f;
  float m_calib_sum_y = 0.0f;
  float m_calib_sum_z = 0.0f;
  uint32_t m_calib_samples = 0;

  float m_alpha = 0.98f;
  unsigned long m_last_time_us = 0;

  bool m_initialized = false;
  bool m_calibrating = false;
  bool m_calibrated = false;
};
#endif
