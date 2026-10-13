#include "serial_telemetry.h"
#include <Arduino.h>
#include <WiFi.h>
#include <esp_arduino_version.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <freertos/queue.h>

namespace {
constexpr unsigned queue_depth = 32;
StaticQueue_t queue_control;
uint8_t queue_storage[queue_depth * sizeof(serial_telemetry::Frame)];
QueueHandle_t frames = nullptr;
portMUX_TYPE counter_lock = portMUX_INITIALIZER_UNLOCKED;
uint32_t sequence = 0, queue_drops = 0;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void on_data_received(const esp_now_recv_info_t *, const uint8_t *data,
                      int length) {
#else
void on_data_received(const uint8_t *, const uint8_t *data, int length) {
#endif
  if (length < 0 || !serial_telemetry::valid(data, length))
    return;
  serial_telemetry::Frame frame;
  frame.arrival_us = micros();
  portENTER_CRITICAL(&counter_lock);
  frame.sequence = sequence++;
  portEXIT_CRITICAL(&counter_lock);
  memcpy(frame.bytes, data, sizeof(frame.bytes));
  if (xQueueSend(frames, &frame, 0) != pdTRUE) {
    portENTER_CRITICAL(&counter_lock);
    ++queue_drops;
    portEXIT_CRITICAL(&counter_lock);
  }
}
} // namespace

void setup() {
  Serial.begin(460800);
  frames = xQueueCreateStatic(queue_depth, sizeof(serial_telemetry::Frame),
                              queue_storage, &queue_control);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(espnow_wifi_channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  if (frames && esp_now_init() == ESP_OK)
    esp_now_register_recv_cb(on_data_received);
}

void loop() {
  serial_telemetry::Frame frame;
  if (!frames) {
    delay(1000);
    return;
  }
  if (xQueueReceive(frames, &frame, portMAX_DELAY) != pdTRUE)
    return;
  portENTER_CRITICAL(&counter_lock);
  uint32_t drops = queue_drops;
  portEXIT_CRITICAL(&counter_lock);
  uint8_t encoded[serial_telemetry::encoded_size];
  size_t length = serial_telemetry::encode(frame, drops, encoded);
  for (size_t sent = 0; sent < length;) {
    size_t count = Serial.write(encoded + sent, length - sent);
    sent += count;
    if (!count)
      vTaskDelay(1);
  }
}
