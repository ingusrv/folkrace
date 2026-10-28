#include "config.h"
#include "front_tof.h"
#include "sensors/tof_bus.h"
#include <cassert>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

static void reg16(TwoWire &bus, uint8_t address, uint16_t reg, uint16_t value) {
  bus.registers[(address << 16) | reg] = value >> 8;
  bus.registers[(address << 16) | (reg + 1)] = value & 255;
}
static void seed(TwoWire &bus, uint8_t address) {
  reg16(bus, address, VL53L4CD_IDENTIFICATION_MODEL_ID, 0xebaa);
  bus.registers[(address << 16) | VL53L4CD_FIRMWARE_SYSTEM_STATUS] = 3;
  reg16(bus, address, 0x0006, 0xbccc); // Factory oscillator trim
  reg16(bus, address, VL53L4CD_RESULT_OSC_CALIBRATE_VAL, 100);
  // Default active-low interrupt: ready at 0.
  bus.registers[(address << 16) | VL53L4CD_GPIO_TIO_HV_STATUS] = 0;
  bus.registers[(address << 16) | VL53L4CD_RESULT_RANGE_STATUS] = 9;
  reg16(bus, address, VL53L4CD_RESULT_SPAD_NB, 256);
  reg16(bus, address, VL53L4CD_RESULT_DISTANCE, 500);
}
static void recorded_startup_no_returns(const char *path) {
  std::ifstream input(path);
  assert(input);
  FrontToF driver;
  unsigned rows = 0, false_cuts = 0, valid_near = 0;
  std::string line;
  while (std::getline(input, line)) {
    if (line.empty() || line[0] == '#')
      continue;
    std::istringstream record(line);
    uint32_t ms;
    unsigned raw[3], status[3], fl, fr;
    float recorded_throttle;
    record >> ms;
    for (unsigned i = 0; i < 3; ++i)
      record >> raw[i] >> status[i];
    record >> fl >> fr >> recorded_throttle;
    assert(record);
    RobotState state{};
    state.current_time_us = ms * 1000U;
    for (unsigned i = 0; i < 3; ++i) {
      SensorData &out = *sensor_policy::distances(state, i);
      tof_accept_range(driver, out, raw[i], status[i], state.current_time_us,
                       config::sensor_min_raw, config::sensor_max_raw);
      assert(out.raw_range_mm == raw[i] && out.raw_status == status[i]);
      if (status[i] == 2)
        assert(out.distance_mm == config::sensor_max_raw &&
               out.health == SensorHealth::OUT_OF_RANGE);
      if (i == 0 && status[i] == 0 && raw[i] < 220) {
        ++valid_near;
        assert(out.distance_mm < 220);
      }
    }
    sensor_policy::accept_range(state.dist_side_front_left, fl, 90,
                                state.current_time_us, 30, 1000);
    sensor_policy::accept_range(state.dist_side_front_right, fr, 90,
                                state.current_time_us, 30, 1000);
    assert(sensor_policy::required(state, state.current_time_us));
    if (recorded_throttle == 0.0f && status[0] == 2 && raw[0] < 220) {
      ++false_cuts;
      assert(state.dist_front.distance_mm == config::sensor_max_raw);
    }
    ++rows;
  }
  assert(rows > 300 && false_cuts > 100 && valid_near > 0);
  std::cout << "recorded L4CD startup replay: " << false_cuts
            << " false-close throttle cuts removed; valid close returns kept\n";
}

int main(int argc, char **argv) {
  assert(argc == 2);
  recorded_startup_no_returns(argv[1]);
  TwoWire wire;
  wire.register16 = true;
  sensor_policy::ToFBusT<FrontToF> bus(wire, 4, 5, 30, 1000,
                                       config::front_tof_timing_budget_us);
  bus.count = 3;
  for (unsigned i = 0; i < bus.count; ++i) {
    auto &d = bus.devices[i];
    d.address = 0x30 + i;
    d.xshut = 16 + i;
    seed(wire, d.address);
  }
  bus.begin();
  assert(bus.recover());
  unsigned address_writes = 0;
  for (auto &tx : wire.transactions)
    if (tx.first == 0x29 && tx.second == VL53L4CD_I2C_SLAVE_DEVICE_ADDRESS)
      ++address_writes;
  assert(address_writes == 3);
  for (auto &d : bus.devices) {
    // Unused fourth device stays untouched.
    if (d.xshut < 0)
      continue;
    assert(d.data.health == SensorHealth::RECOVERING);
    uint32_t budget = 0, inter = 99;
    assert(d.driver.VL53L4CD_GetRangeTiming(&budget, &inter) == 0);
    assert(budget >= 9 && budget <= 10 && inter == 0);
  }
  bus.poll();
  for (unsigned i = 0; i < bus.count; ++i) {
    const auto &data = bus.devices[i].data;
    assert(data.distance_mm == 500 && data.raw_status == 0);
    assert(data.health == SensorHealth::OK);
  }
  // A weak return can contain a false near distance. It must remain visible
  // in raw telemetry without triggering braking, steering or bus recovery.
  wire.registers[(0x30 << 16) | VL53L4CD_RESULT_RANGE_STATUS] =
      4; // ULD status 2
  reg16(wire, 0x30, VL53L4CD_RESULT_DISTANCE, 75);
  bus.poll();
  assert(bus.devices[0].data.raw_range_mm == 75);
  assert(bus.devices[0].data.raw_status == 2);
  assert(bus.devices[0].data.distance_mm == 1000);
  assert(bus.devices[0].data.health == SensorHealth::OUT_OF_RANGE);
  assert(!bus.pending());
  wire.registers[(0x30 << 16) | VL53L4CD_RESULT_RANGE_STATUS] =
      9; // ULD status 0
  reg16(wire, 0x30, VL53L4CD_RESULT_DISTANCE, 75);
  bus.poll();
  assert(bus.devices[0].data.distance_mm == 75);
  assert(bus.devices[0].data.health == SensorHealth::OK);
  SensorData close;
  tof_accept_range(bus.devices[0].driver, close, 0, 3, micros(), 30, 1000);
  assert(close.distance_mm == 30 && close.health == SensorHealth::OK);
  uint32_t timestamp = bus.devices[0].data.timestamp_us;
  // Not-ready polling must preserve the old sample timestamp.
  wire.registers[(0x30 << 16) | VL53L4CD_GPIO_TIO_HV_STATUS] = 1;
  fake_us += 10000;
  bus.poll();
  assert(bus.devices[0].data.timestamp_us == timestamp);
  wire.registers[(0x30 << 16) | VL53L4CD_GPIO_TIO_HV_STATUS] = 0;
  reg16(wire, 0x30, VL53L4CD_RESULT_DISTANCE, 1400);
  bus.poll();
  assert(bus.devices[0].data.raw_range_mm == 1400);
  assert(bus.devices[0].data.distance_mm == 1000);
  assert(bus.devices[0].data.health == SensorHealth::OUT_OF_RANGE);
  timestamp = bus.devices[0].data.timestamp_us;
  // A short result read must not publish partially populated results.
  wire.short_reg = VL53L4CD_RESULT_DISTANCE;
  wire.short_address = 0x30;
  bus.poll();
  assert(bus.devices[0].data.last_error == SensorError::SHORT_READ);
  assert(bus.devices[0].data.timestamp_us == timestamp);
  assert(bus.devices[1].data.health == SensorHealth::OK);
  wire.short_reg = -1;
  fake_us += 500000;
  assert(bus.recover());
  bus.poll();
  assert(bus.devices[0].data.health == SensorHealth::OUT_OF_RANGE);
  // Acknowledgement errors cannot refresh a sample either.
  wire.fail_status = 2;
  wire.fail_address = 0x30;
  wire.fail_reg = VL53L4CD_SYSTEM_INTERRUPT_CLEAR;
  timestamp = bus.devices[0].data.timestamp_us;
  bus.poll();
  assert(bus.devices[0].data.last_error == SensorError::NACK);
  assert(bus.devices[0].data.timestamp_us == timestamp);
  // A disconnected sensor returns without retrying forever.
  FrontToF disconnected;
  disconnected.setBus(&wire);
  wire.fail_address = -1;
  wire.fail_reg = -1;
  bool ready = false;
  uint16_t mm = 123;
  uint8_t status = 0;
  size_t before = wire.transactions.size();
  assert(tof_read_sample(disconnected, ready, mm, status) == 2);
  assert(wire.transactions.size() - before <= 2);
  assert(mm == 123);
  std::cout
      << "VL53L4CD addressing, timing, freshness, and transport tests passed\n";
}
