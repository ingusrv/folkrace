#pragma once
#if __has_include(<BMI088.h>)
#include "state.h"
#include <Arduino.h>
#include <BMI088.h>
#include <Wire.h>

struct IMUSample {
  float ax, ay, az; // m/s², mounted axes
  float gx, gy, gz; // rad/s, mounted axes, before bias correction
};

class BMI088Driver {
public:
  BMI088Driver(TwoWire &bus = Wire, uint8_t accel_addr = 0x19,
               uint8_t gyro_addr = 0x69);
  /** Synchronous vendor reset/self-test; run only in the bus owner. */
  bool begin();
  /** Read both devices atomically from the caller's perspective.
   * @param out Unchanged on failure. @param error Transaction/device failure.
   * @return true only after all sample and status reads succeeded. */
  bool read_sample(IMUSample &out, SensorError &error);
  /** Fuse an already checked sample. @param timestamp_us Acquisition time. */
  void fuse(const IMUSample &sample, uint32_t timestamp_us);
  /** Reseed orientation on next checked sample, without changing biases. */
  void reset_orientation();
  /** Restore committed gyro biases in rad/s. */
  void set_gyro_biases(float bx, float by, float bz);
  const IMUState &get_state() const { return m_state; }
  bool is_initialized() const { return m_initialized; }
  bool is_calibrated() const { return m_calibrated; }

private:
  bool read_registers(uint8_t address, uint8_t reg, uint8_t *data,
                      uint8_t length, SensorError &error);
  TwoWire &m_bus;
  uint8_t m_accel_addr, m_gyro_addr;
  Bmi088Accel m_accel;
  Bmi088Gyro m_gyro;
  IMUState m_state = {};
  float m_gyro_bias_x = 0.0f, m_gyro_bias_y = 0.0f, m_gyro_bias_z = 0.0f;
  uint32_t m_last_time_us = 0;
  bool m_seed = true, m_initialized = false, m_calibrated = false;
};
#endif
