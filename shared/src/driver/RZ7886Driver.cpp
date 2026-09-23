#include "RZ7886Driver.h"
#include <Arduino.h>
#include <math.h>

void RZ7886Driver::init() {
  ledcAttach(m_fi, PWM_FREQ, PWM_BITS);
  ledcAttach(m_bi, PWM_FREQ, PWM_BITS);
  stop();
}

void RZ7886Driver::write(float throttle) {
  throttle = constrain(throttle, -1.0f, 1.0f);

  uint32_t duty = static_cast<uint32_t>(roundf(fabsf(throttle) * MAX_DUTY));

  if (throttle > 0.0f) {
    ledcWrite(m_fi, duty);
    ledcWrite(m_bi, 0);
  } else if (throttle < 0.0f) {
    ledcWrite(m_fi, 0);
    ledcWrite(m_bi, duty);
  } else {
    ledcWrite(m_fi, 0);
    ledcWrite(m_bi, 0);
  }
}

void RZ7886Driver::stop() {
  ledcWrite(m_fi, 0);
  ledcWrite(m_bi, 0);
}

void RZ7886Driver::brake() {
  ledcWrite(m_bi, MAX_DUTY);
  ledcWrite(m_fi, MAX_DUTY);
}