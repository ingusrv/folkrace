#include "serial_telemetry.h"
#include <cassert>
#include <cstdio>

int main() {
  using namespace serial_telemetry;
  assert(crc32(reinterpret_cast<const uint8_t *>("123456789"), 9) ==
         0xcbf43926);
  PacketHeader header{message_magic, PacketType::TELEMETRY, RobotId::ROSEPINK,
                      sizeof(TelemetryPacket)};
  TelemetryPacket packet{};
  packet.protocol_version = TELEMETRY_PROTOCOL_VERSION;
  packet.timestamp_ms = 0x12345678;
  packet.running_state = RunningState::REVERSE_TURN;
  packet.flags = 0x1d;
  packet.dist_front = 123;
  packet.dist_right = 456;
  packet.dist_left = 789;
  packet.target_steering = -0.75f;
  packet.current_throttle = 0.25f;
  packet.accel_x = 9.5f;
  packet.gyro_z = -3.25f;
  packet.yaw = 179.5f;
  packet.battery_voltage_v = 7.625f;
  for (unsigned i = 0; i < 8; ++i)
    packet.sensors[i] = {uint8_t(i), uint16_t(i * 13), uint16_t(i * 17),
                         uint16_t(i), uint8_t(i)};
  packet.sensors[0].age_ms = 65535;
  for (unsigned i = 0; i < 7; ++i)
    packet.raw_tof[i] = {uint16_t(100 + i), uint8_t(0x58 + i),
                         uint16_t(i + 20)};
  packet.raw_tof[1] = {8190, 0x40, 22};
  packet.raw_tof[2] = {8191, 0x50, 23};
  packet.raw_tof[3] = {0, 0, 65535};
  Frame frame{0xfffffffe, 0xfffffff0, {}};
  memcpy(frame.bytes, &header, sizeof(header));
  memcpy(frame.bytes + sizeof(header), &packet, sizeof(packet));
  assert(valid(frame.bytes, frame_size));
  assert(!valid(frame.bytes, frame_size - 1));
  assert(!valid(frame.bytes, frame_size + 1));
  frame.bytes[8] = 1;
  assert(!valid(frame.bytes, frame_size));
  frame.bytes[8] = TELEMETRY_PROTOCOL_VERSION;
  frame.bytes[6] = 0;
  assert(!valid(frame.bytes, frame_size));
  frame.bytes[6] = sizeof(TelemetryPacket);
  uint8_t out[encoded_size];
  size_t size = encode(frame, 7, out);
  assert(size <= encoded_size);
  assert(fwrite(out, 1, size, stdout) == size);
}
