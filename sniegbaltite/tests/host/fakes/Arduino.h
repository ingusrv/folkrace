#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <map>
using boolean = bool;
constexpr float PI = 3.14159265358979323846f;
constexpr int LOW = 0, HIGH = 1, INPUT = 0, OUTPUT = 1, OUTPUT_OPEN_DRAIN = 3;
inline uint32_t fake_us = 1000;
inline uint32_t micros() { return fake_us; }
inline uint32_t millis() { return fake_us / 1000; }
inline void delay(unsigned ms) { fake_us += ms * 1000; }
inline void delayMicroseconds(unsigned us) { fake_us += us; }
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
inline std::map<int, int> fake_pin_levels;
inline int digitalRead(int pin) {
  auto it = fake_pin_levels.find(pin);
  return it == fake_pin_levels.end() ? HIGH : it->second;
}

using byte = uint8_t;
using std::min;
inline uint8_t highByte(uint16_t value) { return value >> 8; }
inline uint8_t lowByte(uint16_t value) { return value & 255; }

#define PROGMEM
