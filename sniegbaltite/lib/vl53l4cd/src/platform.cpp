// Checked transport for STM32duino VL53L4CD 1.0.5 (BSD-3-Clause).
// Replaces upstream's unbounded NACK retry and checks every transferred byte.
#include "vl53l4cd_class.h"
#include <cstring>

uint8_t VL53L4CD::VL53L4CD_I2CRead(uint8_t address, uint16_t reg,
                                   uint8_t *values, uint32_t size) {
  std::memset(values, 0, size);
  dev_i2c->beginTransmission(address >> 1);
  if (dev_i2c->write(static_cast<uint8_t>(reg >> 8)) != 1 ||
      dev_i2c->write(static_cast<uint8_t>(reg)) != 1) {
    dev_i2c->endTransmission();
    return 4;
  }
  uint8_t error = dev_i2c->endTransmission(false);
  if (error)
    return error;
  // ULD reads at most four bytes per call; no retry on a short transfer.
  if (size > 4)
    return 4;
  if (dev_i2c->requestFrom(static_cast<uint8_t>(address >> 1),
                           static_cast<uint8_t>(size)) != size)
    return 7;
  for (uint32_t i = 0; i < size; ++i) {
    if (!dev_i2c->available())
      return 7;
    int value = dev_i2c->read();
    if (value < 0)
      return 7;
    values[i] = static_cast<uint8_t>(value);
  }
  return 0;
}

uint8_t VL53L4CD::VL53L4CD_I2CWrite(uint8_t address, uint16_t reg,
                                    uint8_t *values, uint32_t size) {
  // ULD writes at most four bytes per call, within every supported Wire buffer.
  if (size > 4)
    return 4;
  dev_i2c->beginTransmission(address >> 1);
  bool ok = dev_i2c->write(static_cast<uint8_t>(reg >> 8)) == 1 &&
            dev_i2c->write(static_cast<uint8_t>(reg)) == 1 &&
            dev_i2c->write(values, size) == size;
  uint8_t error = dev_i2c->endTransmission();
  return error ? error : (ok ? 0 : 4);
}

uint8_t VL53L4CD::VL53L4CD_RdByte(uint16_t address, uint16_t reg,
                                  uint8_t *value) {
  return VL53L4CD_I2CRead(address, reg, value, 1);
}
uint8_t VL53L4CD::VL53L4CD_RdWord(uint16_t address, uint16_t reg,
                                  uint16_t *value) {
  uint8_t bytes[2] = {};
  uint8_t error = VL53L4CD_I2CRead(address, reg, bytes, sizeof(bytes));
  *value = (static_cast<uint16_t>(bytes[0]) << 8) | bytes[1];
  return error;
}
uint8_t VL53L4CD::VL53L4CD_RdDWord(uint16_t address, uint16_t reg,
                                   uint32_t *value) {
  uint8_t bytes[4] = {};
  uint8_t error = VL53L4CD_I2CRead(address, reg, bytes, sizeof(bytes));
  *value = (static_cast<uint32_t>(bytes[0]) << 24) |
           (static_cast<uint32_t>(bytes[1]) << 16) |
           (static_cast<uint32_t>(bytes[2]) << 8) | bytes[3];
  return error;
}
uint8_t VL53L4CD::VL53L4CD_WrByte(uint16_t address, uint16_t reg,
                                  uint8_t value) {
  return VL53L4CD_I2CWrite(address, reg, &value, 1);
}
uint8_t VL53L4CD::VL53L4CD_WrWord(uint16_t address, uint16_t reg,
                                  uint16_t value) {
  uint8_t bytes[] = {static_cast<uint8_t>(value >> 8),
                     static_cast<uint8_t>(value)};
  return VL53L4CD_I2CWrite(address, reg, bytes, sizeof(bytes));
}
uint8_t VL53L4CD::VL53L4CD_WrDWord(uint16_t address, uint16_t reg,
                                   uint32_t value) {
  uint8_t bytes[] = {
      static_cast<uint8_t>(value >> 24), static_cast<uint8_t>(value >> 16),
      static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value)};
  return VL53L4CD_I2CWrite(address, reg, bytes, sizeof(bytes));
}
void VL53L4CD::WaitMs(uint32_t duration_ms) { delay(duration_ms); }
