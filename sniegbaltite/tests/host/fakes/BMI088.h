#pragma once
#include "Wire.h"
inline bool fake_imu_begin_ok = true;
inline unsigned fake_accel_begins = 0;
class Bmi088Accel {
public:
  Bmi088Accel(TwoWire &, uint8_t) {}
  int begin() {
    ++fake_accel_begins;
    delay(450);
    return fake_imu_begin_ok ? 1 : -1;
  }
};
class Bmi088Gyro {
public:
  Bmi088Gyro(TwoWire &, uint8_t) {}
  int begin() { return fake_imu_begin_ok ? 1 : -1; }
};
