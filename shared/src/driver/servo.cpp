#include "servo.h"
#include <Arduino.h>

void Servo::init() {
  ledcAttach(m_pin, 50, 10);
}

void Servo::write(int degrees) {
  degrees = constrain(degrees, 0, 180);
  long pulse_width_us = map(degrees, 0, 180, 500, 2500);
  float period_us = 1000000.0 / 50.0;
  uint16_t max_duty = 1023;
  long duty = (pulse_width_us * max_duty) / period_us;
  ledcWrite(m_pin, duty);
}
