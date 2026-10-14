#pragma once
#include "protocol.h"
#include "util.h"
#include <stddef.h>
#include <string.h>

namespace serial_telemetry {
constexpr size_t frame_size = sizeof(PacketHeader) + sizeof(TelemetryPacket);
constexpr size_t decoded_size = 15 + frame_size + 4;
constexpr size_t encoded_size = decoded_size + decoded_size / 254 + 2;
constexpr uint8_t version = 1;
static_assert(frame_size == 196 && decoded_size == 215 && encoded_size == 217,
              "Update serial bandwidth budget and host decoder");
struct Frame {
  uint32_t sequence, arrival_us;
  uint8_t bytes[frame_size];
};
static_assert(sizeof(Frame) == 204, "Receiver queue item size");
/** Validate an exact supported radio frame before copying or enqueueing it. */
inline bool valid(const uint8_t *data, size_t length) {
  if (length != frame_size)
    return false;
  PacketHeader header;
  memcpy(&header, data, sizeof(header));
  return header.magic == message_magic &&
         header.type == PacketType::TELEMETRY &&
         header.length == sizeof(TelemetryPacket) &&
         data[sizeof(header)] == TELEMETRY_PROTOCOL_VERSION;
}
inline void put_u32(uint8_t *out, uint32_t value) {
  for (unsigned i = 0; i < 4; ++i)
    out[i] = value >> (8 * i);
}
/** CRC-32/ISO-HDLC, compatible with zlib.crc32. */
inline uint32_t crc32(const uint8_t *data, size_t size) {
  uint32_t crc = 0xffffffff;
  for (size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (unsigned bit = 0; bit < 8; ++bit)
      crc = (crc >> 1) ^ ((crc & 1) ? 0xedb88320 : 0);
  }
  return ~crc;
}
/** Encode a validated frame into an encoded_size buffer, including delimiter.
 * @return Number of bytes to write. */
inline size_t encode(const Frame &frame, uint32_t drops, uint8_t *out) {
  uint8_t raw[decoded_size] = {};
  raw[0] = version;
  put_u32(raw + 1, frame.sequence);
  put_u32(raw + 5, frame.arrival_us);
  put_u32(raw + 9, drops);
  raw[13] = frame_size & 0xff;
  raw[14] = frame_size >> 8;
  memcpy(raw + 15, frame.bytes, frame_size);
  put_u32(raw + decoded_size - 4, crc32(raw, decoded_size - 4));
  size_t write = 1, code_at = 0;
  uint8_t code = 1;
  for (uint8_t value : raw) {
    if (!value) {
      out[code_at] = code;
      code_at = write++;
      code = 1;
    } else {
      out[write++] = value;
      if (++code == 0xff) {
        out[code_at] = code;
        code_at = write++;
        code = 1;
      }
    }
  }
  out[code_at] = code;
  out[write++] = 0;
  return write;
}
} // namespace serial_telemetry
