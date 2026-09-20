#include "driver/bmi088.h"
#if __has_include(<BMI088.h>)
#include <math.h>

BMI088Driver::BMI088Driver(TwoWire &bus, uint8_t accel_addr, uint8_t gyro_addr)
    : m_accel(bus, accel_addr), m_gyro(bus, gyro_addr) {}

bool BMI088Driver::begin() {
  int status = m_accel.begin();
  if (status < 0) {
    return false;
  }

  status = m_gyro.begin();
  if (status < 0) {
    return false;
  }

  m_initialized = true;

  // Initialize roll and pitch from accelerometer gravity vector
  m_accel.readSensor();
  float ax = m_accel.getAccelX_mss();
  float ay = m_accel.getAccelY_mss();
  float az = m_accel.getAccelZ_mss();

  m_state.accel_x = ax;
  m_state.accel_y = ay;
  m_state.accel_z = az;

  m_state.roll = atan2f(-ax, az) * 180.0f / PI;
  m_state.pitch = atan2f(ay, sqrtf(ax * ax + az * az)) * 180.0f / PI;
  m_state.yaw = 0.0f;

  m_last_time_us = micros();
  return true;
}

void BMI088Driver::startCalibration() {
  m_calib_sum_x = 0.0f;
  m_calib_sum_y = 0.0f;
  m_calib_sum_z = 0.0f;
  m_calib_samples = 0;
  m_calibrating = true;
}

void BMI088Driver::updateCalibration() {
  if (!m_initialized || !m_calibrating) {
    return;
  }

  m_gyro.readSensor();
  m_calib_sum_x += m_gyro.getGyroX_rads();
  m_calib_sum_y += m_gyro.getGyroY_rads();
  m_calib_sum_z += m_gyro.getGyroZ_rads();
  m_calib_samples++;
}

void BMI088Driver::finishCalibration() {
  if (m_calib_samples > 0) {
    m_gyro_bias_x = m_calib_sum_x / static_cast<float>(m_calib_samples);
    m_gyro_bias_y = m_calib_sum_y / static_cast<float>(m_calib_samples);
    m_gyro_bias_z = m_calib_sum_z / static_cast<float>(m_calib_samples);
    m_calibrated = true;
  }
  m_calibrating = false;
  m_state.yaw = 0.0f;
  m_last_time_us = micros();
}

void BMI088Driver::calibrate(uint16_t samples, uint16_t delay_ms) {
  startCalibration();
  for (uint16_t i = 0; i < samples; ++i) {
    updateCalibration();
    delay(delay_ms);
  }
  finishCalibration();
}

void BMI088Driver::update() {
  if (!m_initialized) {
    return;
  }

  m_accel.readSensor();
  m_gyro.readSensor();

  unsigned long now_us = micros();
  float dt = static_cast<float>(now_us - m_last_time_us) * 1e-6f;
  m_last_time_us = now_us;

  // Protect against large dt jump (e.g. after long delay or overflow)
  if (dt > 0.2f || dt <= 0.0f) {
    dt = 0.01f;
  }

  float ax = m_accel.getAccelX_mss();
  float ay = m_accel.getAccelY_mss();
  float az = m_accel.getAccelZ_mss();

  m_state.accel_x = ax;
  m_state.accel_y = ay;
  m_state.accel_z = az;

  // Accelerometer pitch and roll (degrees) - adapted for mounted orientation
  float pitch_acc = atan2f(ay, sqrtf(ax * ax + az * az)) * 180.0f / PI;
  float roll_acc = atan2f(-ax, az) * 180.0f / PI;

  // Gyro angular rates in deg/s (with bias removed)
  float gx_deg = (m_gyro.getGyroX_rads() - m_gyro_bias_x) * 180.0f / PI;
  float gy_deg = (m_gyro.getGyroY_rads() - m_gyro_bias_y) * 180.0f / PI;
  float gz_deg = (m_gyro.getGyroZ_rads() - m_gyro_bias_z) * 180.0f / PI;

  m_state.gyro_x = gx_deg;
  m_state.gyro_y = gy_deg;
  m_state.gyro_z = gz_deg;

  // Complementary filter: fuse gyro integration with accelerometer
  m_state.roll = m_alpha * (m_state.roll + gy_deg * dt) + (1.0f - m_alpha) * roll_acc;
  m_state.pitch = m_alpha * (m_state.pitch + gx_deg * dt) + (1.0f - m_alpha) * pitch_acc;
  m_state.yaw += gz_deg * dt;
}

void BMI088Driver::update(IMUState &state) {
  update();
  state = m_state;
}
#endif
