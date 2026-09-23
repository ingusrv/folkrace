#pragma once
#include <stdint.h>

class RZ7886Driver {
public:
  RZ7886Driver(uint8_t bi, uint8_t fi) : m_bi(bi), m_fi(fi) {}

  void init();
  void write(float throttle); // -1.0f (full reverse) to +1.0f (full forward), 0.0f is stop
  void stop();
  void brake();

private:
  uint8_t m_bi;
  uint8_t m_fi;

  static constexpr uint32_t PWM_FREQ = 1000;
  static constexpr uint8_t PWM_BITS = 14;
  static constexpr uint32_t MAX_DUTY = (1 << PWM_BITS) - 1;
};

