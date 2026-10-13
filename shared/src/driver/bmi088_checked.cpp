#include "driver/bmi088.h"
#if __has_include(<BMI088.h>)
#include <math.h>

BMI088Driver::BMI088Driver(TwoWire &bus, uint8_t accel_addr, uint8_t gyro_addr)
    : m_bus(bus), m_accel_addr(accel_addr), m_gyro_addr(gyro_addr),
      m_accel(bus, accel_addr), m_gyro(bus, gyro_addr) {}

bool BMI088Driver::begin() {
  m_initialized = false;
  if (m_accel.begin() < 0 || m_gyro.begin() < 0)
    return false;
  m_initialized = true;
  reset_orientation();
  return true;
}

bool BMI088Driver::read_registers(uint8_t address, uint8_t reg, uint8_t *data,
                                  uint8_t length, SensorError &error) {
  m_bus.beginTransmission(address);
  bool written = m_bus.write(reg) == 1;
  uint8_t rc = m_bus.endTransmission();
  if (!written && rc == 0)
    rc = 1;
  if (rc != 0) {
    error = rc == 5                ? SensorError::TIMEOUT
            : (rc == 2 || rc == 3) ? SensorError::NACK
                                   : SensorError::BUS_ERROR;
    return false;
  }
  if (m_bus.requestFrom(address, length) != length) {
    while (m_bus.available())
      m_bus.read();
    error = SensorError::SHORT_READ;
    return false;
  }
  for (uint8_t i = 0; i < length; ++i)
    data[i] = m_bus.read();
  return true;
}

bool BMI088Driver::read_sample(IMUSample &out, SensorError &error) {
  error = SensorError::NONE;
  if (!m_initialized) {
    error = SensorError::INIT_FAILED;
    return false;
  }
  uint8_t accel_id, gyro_id, accel_error, gyro_test, accel[6], gyro[6];
  if (!read_registers(m_accel_addr, 0x00, &accel_id, 1, error) ||
      !read_registers(m_gyro_addr, 0x00, &gyro_id, 1, error) ||
      !read_registers(m_accel_addr, 0x02, &accel_error, 1, error) ||
      !read_registers(m_gyro_addr, 0x3C, &gyro_test, 1, error))
    return false;
  if (accel_id != 0x1E || gyro_id != 0x0F || (accel_error & 0x1D) ||
      (gyro_test & 0x04)) {
    error = SensorError::HARDWARE_ERROR;
    return false;
  }
  if (!read_registers(m_accel_addr, 0x12, accel, 6, error) ||
      !read_registers(m_gyro_addr, 0x02, gyro, 6, error))
    return false;
  // BolderFlight BMI088 v1.0.1 (5e839cbddf5eb540d9530e36385804ec0b47ed23):
  // default 24 g / 2000 deg/s ranges and (X,-Y,-Z) transform, exactly once.
  // Physical scaling follows upstream readSensor(); see BMI088_PROVENANCE.md.
  auto axis = [](const uint8_t *bytes, int i) {
    return static_cast<int16_t>(uint16_t(bytes[i]) |
                                (uint16_t(bytes[i + 1]) << 8));
  };
  constexpr float accel_scale = 24.0f * 9.807f / 32768.0f;
  constexpr float gyro_scale = 2000.0f * PI / 180.0f / 32767.0f;
  out = {axis(accel, 0) * accel_scale,  -axis(accel, 2) * accel_scale,
         -axis(accel, 4) * accel_scale, axis(gyro, 0) * gyro_scale,
         -axis(gyro, 2) * gyro_scale,   -axis(gyro, 4) * gyro_scale};
  return true;
}

void BMI088Driver::reset_orientation() {
  m_seed = true;
  m_last_time_us = 0;
}
void BMI088Driver::set_gyro_biases(float bx, float by, float bz) {
  m_gyro_bias_x = bx;
  m_gyro_bias_y = by;
  m_gyro_bias_z = bz;
  m_calibrated = true;
}
void BMI088Driver::fuse(const IMUSample &s, uint32_t timestamp_us) {
  constexpr float degrees = 180.0f / PI;
  const float pitch = atan2f(s.ay, sqrtf(s.ax * s.ax + s.az * s.az)) * degrees;
  const float roll = atan2f(-s.ax, s.az) * degrees;
  m_state.accel_x = s.ax;
  m_state.accel_y = s.ay;
  m_state.accel_z = s.az;
  m_state.gyro_x = (s.gx - m_gyro_bias_x) * degrees;
  m_state.gyro_y = (s.gy - m_gyro_bias_y) * degrees;
  m_state.gyro_z = (s.gz - m_gyro_bias_z) * degrees;
  if (m_seed) {
    m_state.roll = roll;
    m_state.pitch = pitch;
    m_state.yaw = 0.0f;
    m_seed = false;
  } else {
    float dt = static_cast<uint32_t>(timestamp_us - m_last_time_us) * 1e-6f;
    m_state.roll = 0.98f * (m_state.roll + m_state.gyro_y * dt) + 0.02f * roll;
    m_state.pitch =
        0.98f * (m_state.pitch + m_state.gyro_x * dt) + 0.02f * pitch;
    m_state.yaw += m_state.gyro_z * dt;
  }
  m_last_time_us = timestamp_us;
}
#endif
