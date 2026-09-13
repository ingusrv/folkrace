#include "esc.h"
#include <Arduino.h>

void ESC::init() {
  ledcAttach(m_pin, PWM_FREQ, PWM_BITS);
  stop(); // Output 1500us neutral pulse immediately for arming
}

void ESC::writeMicroseconds(uint16_t us) {
  us = constrain(us, m_min_us, m_max_us);
  // Period is 20,000 us (50 Hz). duty = (us * MAX_DUTY) / 20000
  uint32_t duty = (static_cast<uint32_t>(us) * MAX_DUTY) / 20000;
  ledcWrite(m_pin, duty);
}

void ESC::write(int percent) {
  percent = constrain(percent, -100, 100);

  uint16_t us;
  if (percent >= 0) {
    us = map(percent, 0, 100, m_neutral_us, m_max_us);
  } else {
    us = map(percent, -100, 0, m_min_us, m_neutral_us);
  }

  writeMicroseconds(us);
}

void ESC::stop() {
  writeMicroseconds(m_neutral_us);
}
