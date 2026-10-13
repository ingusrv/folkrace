# VL53L0X checked transport fork

Pololu vl53l0x-arduino 1.3.1, commit
`9f3773cb48d4e4e844d689cfc529a06f96d1d264` (MIT, LICENSE.txt).
https://github.com/pololu/vl53l0x-arduino/tree/9f3773cb48d4e4e844d689cfc529a06f96d1d264

Local changes: sticky first transaction error, checked byte counts, host-only
address adoption, finite/yielding polling, and checked address commit. Call
clear_error() once before a complete operation, then inspect first_error().
Error 7 denotes a short read; other values are Arduino Wire status codes.

Internal range-status mapping reference: ST implementation distributed by
Adafruit at 99a130ca7c7ee930fa428244dbe16d84703e4756,
src/core/src/vl53l0x_api_core.cpp (VL53L0X_get_pal_range_status).
https://github.com/adafruit/Adafruit_VL53L0X/tree/99a130ca7c7ee930fa428244dbe16d84703e4756
This adapter does not implement ST's additional sigma limit calculations.
