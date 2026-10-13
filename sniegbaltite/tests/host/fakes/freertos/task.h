#pragma once
#include "Arduino.h"
#include "FreeRTOS.h"
inline void vTaskDelay(TickType_t ticks) { fake_us += ticks * 1000; }
