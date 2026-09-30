#include "protocol.h"
#include "state.h"
#include "util.h"
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

void on_data_received(const esp_now_recv_info_t *recv_info, const uint8_t *data,
                      int length) {
  if (length < static_cast<int>(sizeof(PacketHeader))) {
    return;
  }

  PacketHeader header = {};
  memcpy(&header, data, sizeof(header));

  if (header.magic != message_magic) {
    return;
  }

  if (header.type == PacketType::TELEMETRY) {
    if (length <
        static_cast<int>(sizeof(PacketHeader) + sizeof(TelemetryPacket))) {
      return;
    }

    TelemetryPacket pkt = {};
    memcpy(&pkt, data + sizeof(PacketHeader), sizeof(pkt));

    // Print JSON line for Python script consumption
    Serial.printf(
        "{\"robot\":\"%s\",\"robot_id\":%u,\"time\":%lu,\"state\":\"%s\",\"state_id\":%u,\"flags\":%u,"
        "\"dist_front\":%u,\"dist_right\":%u,\"dist_left\":%u,"
        "\"dist_side_fl\":%u,\"dist_side_rl\":%u,\"dist_side_fr\":%u,\"dist_side_rr\":%u,"
        "\"target_steering\":%.3f,\"current_steering\":%.3f,"
        "\"target_throttle\":%.3f,\"current_throttle\":%.3f,"
        "\"pid_error\":%.4f,\"pid_integral\":%.4f,\"pid_derivative\":%.4f,"
        "\"accel\":[%.3f,%.3f,%.3f],\"gyro\":[%.3f,%.3f,%.3f],\"euler\":[%.2f,%"
        ".2f,%.2f]}\n",
        robot_id_to_str(header.robot_id),
        static_cast<uint8_t>(header.robot_id),
        (unsigned long)pkt.timestamp_ms,
        running_state_to_str(pkt.running_state),
        static_cast<uint8_t>(pkt.running_state), pkt.flags, pkt.dist_front,
        pkt.dist_right, pkt.dist_left,
        pkt.dist_side_front_left, pkt.dist_side_rear_left,
        pkt.dist_side_front_right, pkt.dist_side_rear_right,
        pkt.target_steering,
        pkt.current_steering, pkt.target_throttle, pkt.current_throttle,
        pkt.pid_error, pkt.pid_integral, pkt.pid_derivative, pkt.accel_x,
        pkt.accel_y, pkt.accel_z, pkt.gyro_x, pkt.gyro_y, pkt.gyro_z, pkt.roll,
        pkt.pitch, pkt.yaw);
  }
}

void setup() {
  Serial.begin(460800);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(espnow_wifi_channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("{\"error\":\"ESP-NOW initialization failed\"}");
    return;
  }

  esp_now_register_recv_cb(on_data_received);
}

void loop() { delay(10); }
