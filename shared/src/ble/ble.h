#pragma once
#include <stddef.h>
#include <stdint.h>

void init_ble(const char *device_name);
bool ble_is_connected();
void notify(const char *fmt, ...);
void ble_send(const uint8_t *data, size_t length);
