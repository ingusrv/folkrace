#pragma once
#include <stdint.h>

class RZ7886Driver {
public:
  RZ7886Driver(uint8_t bi, uint8_t fi) : m_bi(bi), m_fi(fi) {}
  void init();
  void write(int16_t speed);
  void brake();

private:
  uint8_t m_bi;
  uint8_t m_fi;
};
