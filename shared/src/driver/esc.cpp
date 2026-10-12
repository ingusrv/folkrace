#include "esc.h"
#include <Arduino.h>

void ESC::init() {
  ledcAttach(m_pin, PWM_FREQ, PWM_BITS);
  stop(); // Output 1500us neutral pulse immediately for arming
}

void ESC::write_microseconds(uint16_t us) {
  us = constrain(us, m_min_us, m_max_us);
  // Period is 20,000 us (50 Hz). duty = (us * MAX_DUTY) / 20000
  uint32_t duty = (static_cast<uint32_t>(us) * MAX_DUTY) / 20000;
  ledcWrite(m_pin, duty);
}

void ESC::write(float throttle) {
  throttle = constrain(throttle, -1.0f, 1.0f);

  uint16_t us;
  if (throttle >= 0.0f) {
    us = static_cast<uint16_t>(m_neutral_us + throttle * (m_max_us - m_neutral_us));
  } else {
    us = static_cast<uint16_t>(m_neutral_us + throttle * (m_neutral_us - m_min_us));
  }

  write_microseconds(us);
}

void ESC::stop() {
  write_microseconds(m_neutral_us);
}
