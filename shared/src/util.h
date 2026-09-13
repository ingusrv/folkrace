#pragma once
#include <cstdint>

constexpr uint32_t make_magic(const char (&s)[5]) {
  return uint32_t(s[0]) | (uint32_t(s[1]) << 8) | (uint32_t(s[2]) << 16) |
         (uint32_t(s[3]) << 24);
}

constexpr uint32_t message_magic = make_magic("FOLK");

constexpr uint8_t broadcast_address[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
constexpr uint8_t espnow_wifi_channel = 11;
