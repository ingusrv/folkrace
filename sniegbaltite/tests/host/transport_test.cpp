#include "driver/bmi088.h"
#include <cassert>
#include <cmath>
#include <iostream>

int main() {
  TwoWire bus;
  BMI088Driver imu(bus);
  assert(imu.begin());
  bus.registers[0x1900] = 0x1e;
  bus.registers[0x6900] = 0x0f;
  // +256 counts on all chip axes -> mounted (+,-,-), exactly once.
  for (unsigned reg : {0x1913, 0x1915, 0x1917, 0x6903, 0x6905, 0x6907})
    bus.registers[reg] = 1;
  IMUSample sample{};
  SensorError error;
  assert(imu.read_sample(sample, error));
  assert(sample.ax > 0 && sample.ay < 0 && sample.az < 0);
  assert(sample.gx > 0 && sample.gy < 0 && sample.gz < 0);
  assert(std::abs(sample.ax - 256.0f * 24.0f * 9.807f / 32768) < 0.00001f);
  imu.fuse(sample, 10000);
  IMUState before = imu.get_state();
  IMUSample saved = sample;
  bus.short_reg = 0x02; // Gyro sample transaction, not just presence probe.
  assert(!imu.read_sample(sample, error) && error == SensorError::SHORT_READ);
  assert(memcmp(&sample, &saved, sizeof(sample)) == 0);
  assert(memcmp(&before, &imu.get_state(), sizeof(before)) == 0);
  bus.short_reg = -1;
  bus.fail_reg = 0x12;
  bus.fail_status = 2;
  assert(!imu.read_sample(sample, error) && error == SensorError::NACK);
  assert(memcmp(&sample, &saved, sizeof(sample)) == 0);
  bus.fail_status = 0;
  bus.registers[0x1902] = 1;
  assert(!imu.read_sample(sample, error) &&
         error == SensorError::HARDWARE_ERROR);
  bus.registers[0x1902] = 0;
  imu.set_gyro_biases(saved.gx, saved.gy, saved.gz);
  imu.reset_orientation();
  imu.fuse(saved, 20000);
  imu.fuse(saved, 30000);
  assert(imu.get_state().gyro_z == 0.0f && imu.get_state().yaw == 0.0f);
  fake_imu_begin_ok = false;
  assert(!imu.begin() && !imu.is_initialized());
  std::cout << "checked BMI088 transport and axis tests passed\n";
}
