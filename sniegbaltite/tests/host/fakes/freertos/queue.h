#pragma once
#include "FreeRTOS.h"
#include <cstring>
#include <deque>
#include <vector>
struct FakeQueue {
  unsigned capacity, size;
  std::deque<std::vector<uint8_t>> items;
};
using QueueHandle_t = FakeQueue *;
inline QueueHandle_t xQueueCreate(unsigned n, unsigned size) {
  return new FakeQueue{n, size, {}};
}
inline int xQueueSend(QueueHandle_t q, const void *p, unsigned) {
  if (q->items.size() >= q->capacity)
    return pdFALSE;
  const auto *bytes = static_cast<const uint8_t *>(p);
  q->items.emplace_back(bytes, bytes + q->size);
  return pdTRUE;
}
inline int xQueueOverwrite(QueueHandle_t q, const void *p) {
  q->items.clear();
  return xQueueSend(q, p, 0);
}
inline int xQueuePeek(QueueHandle_t q, void *p, unsigned) {
  if (q->items.empty())
    return pdFALSE;
  memcpy(p, q->items.front().data(), q->size);
  return pdTRUE;
}
inline int xQueueReceive(QueueHandle_t q, void *p, unsigned wait) {
  if (!xQueuePeek(q, p, wait))
    return pdFALSE;
  q->items.pop_front();
  return pdTRUE;
}
