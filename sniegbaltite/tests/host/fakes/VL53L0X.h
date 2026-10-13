#pragma once
#include "Wire.h"
#include <cassert>
// Owner tests inject result streams; transport itself is tested with the real
// fork.
class VL53L0X {
public:
  enum {
    RESULT_INTERRUPT_STATUS = 0x13,
    RESULT_RANGE_STATUS = 0x14,
    SYSTEM_INTERRUPT_CLEAR = 0x0b
  };
  inline static bool init_ok = true;
  inline static unsigned init_calls = 0, reads = 0;
  inline static bool ready = true;
  inline static uint8_t injected_error = 0, result_status = 11 << 3;
  inline static uint16_t result_mm = 500;
  inline static bool fail_result = false, fail_ack = false;
  uint8_t error = 0, address = 0x29;
  unsigned timeout_ms = 0;
  TwoWire *bus = nullptr;
  void setBus(TwoWire *value) { bus = value; }
  void setTimeout(unsigned value) { timeout_ms = value; }
  void clear_error() { error = 0; }
  uint8_t first_error() const { return error; }
  void set_host_address(uint8_t a) { address = a; }
  void setAddress(uint8_t a) { address = a; }
  bool init() {
    assert(timeout_ms > 0 && bus && bus->timeout_ms > 0);
    ++init_calls;
    delay(30);
    return init_ok;
  }
  bool setMeasurementTimingBudget(unsigned) { return true; }
  void startContinuous() {}
  uint8_t readReg(uint8_t) {
    ++reads;
    error = injected_error;
    return ready ? 1 : 0;
  }
  bool read_checked(uint8_t, uint8_t *out, unsigned) {
    if (fail_result) {
      error = 7;
      return false;
    }
    out[0] = result_status;
    out[10] = result_mm >> 8;
    out[11] = result_mm;
    return !error;
  }
  void writeReg(uint8_t, uint8_t) {
    if (fail_ack && !error)
      error = 2;
  }
};
