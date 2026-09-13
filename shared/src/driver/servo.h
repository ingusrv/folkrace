#pragma once
#include <stdint.h>

class Servo {
private:
  uint8_t m_pin;

public:
  Servo(uint8_t pin) : m_pin(pin) {}

  void init();
  void write(int degrees);
};
