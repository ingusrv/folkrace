#include "RZ7886Driver.h"
#include <Arduino.h>

void RZ7886Driver::init() {
  ledcAttach(m_bi, 10000, 8);
  ledcAttach(m_fi, 10000, 8);
}

void RZ7886Driver::write(int16_t speed) {
  speed = constrain(speed, -255, 255);

  if (speed >= 0) {
    ledcWrite(m_bi, abs(speed));
    ledcWrite(m_fi, 0);
  } else {
    ledcWrite(m_bi, 0);
    ledcWrite(m_fi, abs(speed));
  }
}

void RZ7886Driver::brake() {
  ledcWrite(m_bi, 255);
  ledcWrite(m_fi, 255);
}