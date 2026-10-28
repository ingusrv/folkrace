#pragma once
#include "Arduino.h"
#include <array>
#include <map>
#include <vector>

class TwoWire {
public:
  std::map<unsigned, uint8_t> registers;
  std::vector<uint8_t> written;
  std::vector<std::pair<uint8_t, uint8_t>> transactions;
  uint8_t address = 0, fail_status = 0;
  uint16_t reg = 0;
  bool register16 = false;
  int fail_address = -1, fail_reg = -1, short_reg = -1;
  int short_address = -1;
  unsigned requested = 0, position = 0, begins = 0, ends = 0, timeout_ms = 0;
  bool begin(int = -1, int = -1, unsigned = 400000) {
    ++begins;
    return true;
  }
  void end() { ++ends; }
  void setTimeOut(unsigned value) { timeout_ms = value; }
  void setClock(unsigned) {}
  void beginTransmission(uint8_t a) {
    address = a;
    written.clear();
  }
  size_t write(uint8_t value) {
    written.push_back(value);
    return 1;
  }
  size_t write(const uint8_t *data, size_t size) {
    written.insert(written.end(), data, data + size);
    return size;
  }
  uint8_t endTransmission(bool = true) {
    if (!written.empty())
      reg = register16 && written.size() >= 2
                ? (uint16_t(written[0]) << 8) | written[1]
                : written.front();
    transactions.emplace_back(address, reg);
    if (fail_status && (fail_address < 0 || fail_address == address) &&
        (fail_reg < 0 || fail_reg == reg))
      return fail_status;
    unsigned prefix = register16 ? 2 : 1;
    for (unsigned i = prefix; i < written.size(); ++i)
      registers[(address << (register16 ? 16 : 8)) | (reg + i - prefix)] =
          written[i];
    return 0;
  }
  uint8_t requestFrom(uint8_t a, uint8_t n) {
    address = a;
    position = 0;
    requested = short_reg == reg && (short_address < 0 || short_address == a)
                    ? n - 1
                    : n;
    return requested;
  }
  unsigned available() { return requested - position; }
  int read() {
    return position < requested ? registers[(address << (register16 ? 16 : 8)) |
                                            (reg + position++)]
                                : -1;
  }
};
inline TwoWire Wire, Wire1;
