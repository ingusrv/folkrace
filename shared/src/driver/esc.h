#pragma once
#include <stdint.h>

class ESC {
public:
  explicit ESC(uint8_t pin, uint16_t min_us = 1000, uint16_t neutral_us = 1500, uint16_t max_us = 2000)
      : m_pin(pin), m_min_us(min_us), m_neutral_us(neutral_us), m_max_us(max_us) {}

  void init();
  void writeMicroseconds(uint16_t us);
  void write(int percent); // -100 (full reverse) to +100 (full forward), 0 is neutral
  void stop();

private:
  uint8_t m_pin;
  uint16_t m_min_us;
  uint16_t m_neutral_us;
  uint16_t m_max_us;

  static constexpr uint32_t PWM_FREQ = 50;
  static constexpr uint8_t PWM_BITS = 14;
  static constexpr uint32_t MAX_DUTY = (1 << PWM_BITS) - 1; // 16383
};
