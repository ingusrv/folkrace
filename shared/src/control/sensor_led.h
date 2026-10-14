#pragma once
#include "state.h"
#include <Arduino.h>
#include <freertos/queue.h>
#include <freertos/task.h>

// All RGB writes belong to this independent task, including off/idle
// indication.
class SensorLED {
public:
  bool begin(uint8_t pin, uint8_t brightness, uint8_t offline_r,
             uint8_t offline_g, uint8_t offline_b, uint32_t interval_ms,
             uint32_t battery_interval_ms = 500) {
    m_pin = pin;
    m_brightness = brightness;
    m_offline_r = offline_r;
    m_offline_g = offline_g;
    m_offline_b = offline_b;
    m_interval_ms = interval_ms;
    m_battery_interval_ms = battery_interval_ms;
    m_queue = xQueueCreate(1, sizeof(uint8_t));
    return m_queue && xTaskCreatePinnedToCore(run, "sensor_led", 2048, this, 1,
                                              nullptr, 0) == pdPASS;
  }
  void update(SensorHealth imu, bool countdown, bool ready,
              bool battery_low = false) {
    uint8_t mode = battery_low                    ? 4
                   : imu == SensorHealth::OFFLINE ? 3
                   : countdown                    ? 2
                   : !ready                       ? 1
                                                  : 0;
    if (m_queue)
      xQueueOverwrite(m_queue, &mode);
  }

private:
  static void run(void *context) {
    auto &self = *static_cast<SensorLED *>(context);
    uint8_t mode = 0, previous = 255;
    uint32_t changed_ms = 0;
    bool on = false;
    for (;;) {
      xQueuePeek(self.m_queue, &mode, 0);
      uint32_t now = millis();
      uint32_t interval = mode == 4   ? self.m_battery_interval_ms
                          : mode == 3 ? self.m_interval_ms
                                      : 150;
      if (mode != previous) {
        previous = mode;
        changed_ms = now;
        on = true;
      } else if (now - changed_ms >= interval) {
        changed_ms = now;
        on = !on;
      }
      uint8_t r = 0, g = 0, b = 0;
      if (on) {
        if (mode == 4) {
          r = self.m_brightness;
        } else if (mode == 3) {
          r = self.m_offline_r;
          g = self.m_offline_g;
          b = self.m_offline_b;
        } else if (mode == 2) {
          r = 128;
          g = 64;
        } else if (mode == 1) {
          r = self.m_brightness;
          b = self.m_brightness;
        }
      }
      rgbLedWrite(self.m_pin, r, g, b);
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
  uint8_t m_pin = 0, m_brightness = 32;
  uint8_t m_offline_r = 128, m_offline_g = 32, m_offline_b = 64;
  uint32_t m_interval_ms = 250, m_battery_interval_ms = 500;
  QueueHandle_t m_queue = nullptr;
};
