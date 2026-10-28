# VL53L4CD driver

Vendored from [STM32duino VL53L4CD 1.0.5](https://github.com/stm32duino/VL53L4CD/tree/1.0.5), under the included BSD-3-Clause license.
Calibration sources are unchanged. `vl53l4cd_api.cpp` returns early on
initialization I2C errors and guards timing arithmetic against failed reads
and zero divisors. `platform.cpp` replaces the
upstream transport: one bounded Wire transaction per operation, explicit NACK
and short-read errors, and initialized outputs on failure. Upstream retries
NACKs indefinitely and can hang the sensor owner task.

The three front sensors use existing XSHUT pins and 7-bit addresses:
front `0x30`, right `0x31`, left `0x32`. The adapter converts these to ST's
8-bit API addresses. Front sensors use a 10 ms timing budget in continuous
mode; side sensors continue using Pololu VL53L0X at 20 ms.
Front `raw_status` reports ST's `range_status` (0 means valid), while side
`raw_status` remains the VL53L0X RESULT_RANGE_STATUS register. BLE reports the status byte without applying model-specific bit extraction.

Build with `pio run` before running the host tests to install other dependencies.

`FrontToF` uses ULD status 0 as a valid range and status 3 (below detection
threshold) as minimum distance. Other statuses, including weak-signal status 2,
produce a fresh maximum-distance/no-return sample for navigation. Raw ranges
and statuses remain in telemetry; communication failures still trigger bus
recovery. This avoids false near readings repeatedly cutting throttle.
