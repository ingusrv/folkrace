#include "side_fit.h"
#include <SparkFun_VL53L5CX_Library.h>
#include <cassert>
#include <cstdio>

int main() {
  TwoWire bus;
  bus.register16 = true;
  SparkFun_VL53L5CX sensor;
  auto &io = *sensor.VL53L5CX_i2c;
  assert(io.begin(0x30, bus));
  bus.registers[(0x30 << 16) | 0x1234] = 0x42;
  assert(io.readSingleByte(0x1234) == 0x42);
  bus.short_reg = 0x1234;
  uint8_t sample[4] = {9, 8, 7, 6};
  assert(io.readMultipleBytes(0x1234, sample, 4) == 6);
  assert(sample[0] == 9 && sample[3] == 6);
  bus.short_reg = -1;
  unsigned transactions = bus.transactions.size();
  assert(io.writeSingleByte(0x1234, 5) == 6);
  assert(bus.transactions.size() ==
         transactions); // Sticky error aborts rest of group.
  io.clear_error();
  bus.fail_status = 2;
  bus.fail_reg = 0x1234;
  assert(io.readMultipleBytes(0x1234, sample, 4) == 2);
  bus.fail_status = 0;
  io.clear_error();
  sensor.Dev->platform.VL53L5CX_i2c = &io;
  bus.short_reg = 0x1234;
  assert(RdByte(&sensor.Dev->platform, 0x1234, sample) == 6);
  bus.short_reg = -1;
  io.clear_error();
  bus.fail_status = 3;
  bus.fail_reg = 0x04;
  assert(!sensor.setAddress(0x31));
  assert(bus.address ==
         0x30); // Address write failed: never adopt the new address.
  bus.fail_status = 0;
  io.clear_error();
  // Runtime ready poll distinguishes a short read from a successful not-ready.
  sensor.Dev->platform.VL53L5CX_i2c = &io;
  bus.short_reg = 0;
  assert(!sensor.isDataReady());
  assert(io.first_error == 6);
  io.clear_error();
  bus.short_reg = -1;
  assert(!sensor.isDataReady());
  assert(io.first_error == 0);
  // Responsive but silent firmware must return from init's polling loops.
  bus.registers[(0x30 << 16)] = 0xf0;
  bus.registers[(0x30 << 16) | 1] = 0x02;
  uint32_t start = fake_us;
  assert(!sensor.begin(0x30, bus));
  assert(fake_us - start < 60000000);
  auto *storage = sensor.Dev;
  io.clear_error();
  assert(!sensor.begin(0x30, bus));
  assert(sensor.Dev == storage); // Retry reuses storage, no allocation/leak.

  VL53L5CX_ResultsData result{};
  for (unsigned i = 0; i < 16; ++i) {
    result.target_status[i] = 5;
    result.distance_mm[i] = 400;
  }
  auto fit = process_vl53l5cx(result, false, false, true);
  assert(fit.valid && fit.front_dist_mm > 350 && fit.rear_dist_mm < 410);
  for (unsigned i = 0; i < 16; ++i)
    result.target_status[i] = 6;
  assert(!process_vl53l5cx(result, false, false, true).valid);
  for (unsigned i = 0; i < 16; ++i) {
    result.target_status[i] = 5;
    result.distance_mm[i] = 2000;
  }
  assert(process_vl53l5cx(result, false, false, true).valid);
  puts("checked VL53L5CX transport, bounded init, and side-fit tests passed");
}
