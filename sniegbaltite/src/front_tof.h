#pragma once
#include "sensors/tof_bus.h"
#include <vl53l4cd_class.h>

// Adapter for the bus owner's Pololu-style initialization interface. The owner
// controls XSHUT, so the ST driver must never toggle it during InitSensor().
class FrontToF : public VL53L4CD {
public:
  FrontToF() : VL53L4CD(nullptr, -1) {}
  uint8_t last_status = 0;
  void setBus(TwoWire *bus) { dev_i2c = bus; }
  // ST bounds its initialization waits internally; I2C transactions use the
  // owner's Wire timeout. There is no equivalent per-device timeout setting.
  void setTimeout(uint16_t) {}
  void setAddress(uint8_t address) {
    // ST uses 8-bit addresses; the bus owner and Wire use 7-bit addresses.
    last_status = VL53L4CD_SetI2CAddress(address << 1);
  }
  bool init() {
    // InitSensor() defaults to 0x52, which would undo address assignment.
    last_status = InitSensor(static_cast<uint8_t>(dev));
    return last_status == 0;
  }
  bool setMeasurementTimingBudget(uint32_t budget_us) {
    if (budget_us % 1000 != 0) {
      last_status = 1;
      return false;
    }
    last_status = VL53L4CD_SetRangeTiming(budget_us / 1000, 0);
    return last_status == 0;
  }
  void startContinuous() { last_status = VL53L4CD_StartRanging(); }
};

// ADL selects this overload for the front bus; side sensors use VL53L0X.
inline uint8_t tof_read_sample(FrontToF &driver, bool &ready,
                               uint16_t &range_mm, uint8_t &status) {
  uint8_t data_ready = 0;
  uint8_t error = driver.VL53L4CD_CheckForDataReady(&data_ready);
  ready = data_ready != 0;
  if (error || !ready)
    return error;
  VL53L4CD_Result_t result = {};
  error = driver.VL53L4CD_GetResult(&result);
  if (error)
    return error;
  error = driver.VL53L4CD_ClearInterrupt();
  if (error)
    return error;
  range_mm = result.distance_mm;
  status = result.range_status;
  return 0;
}

/** Interpret VL53L4CD status before exposing a distance to driving logic.
 * Weak/invalid returns are fresh no-return samples, not close obstacles or
 * transport failures. Preserve the original distance/status for diagnostics.
 */
inline void tof_accept_range(FrontToF &, SensorData &out, uint16_t raw,
                             uint8_t status, uint32_t now, uint16_t minimum,
                             uint16_t maximum) {
  sensor_policy::accept_range(out, raw, status, now, minimum, maximum);
  if (status == 3) {
    // Below the sensor's detection threshold: treat as a very close obstacle.
    out.distance_mm = minimum;
    out.health = SensorHealth::OK;
  } else if (status != 0) {
    out.distance_mm = maximum;
    out.health = SensorHealth::OUT_OF_RANGE;
  }
}
