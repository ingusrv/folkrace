#pragma once
#include "state.h"
#include <algorithm>
#include <stdint.h>

namespace sensor_policy {
constexpr uint32_t tof_age_us = 100000;
constexpr uint32_t imu_age_us = 50000;
constexpr uint32_t healthy_us = 2000000;
struct Settings {
  uint32_t tof_freshness_us = tof_age_us;
  uint32_t imu_freshness_us = imu_age_us;
  uint16_t max_consecutive_errors = 3;
  uint32_t recovery_initial_ms = 500;
  uint32_t recovery_max_ms = 5000;
  uint32_t recovery_healthy_ms = 2000;
  uint32_t bus_timeout_ms = 25;
  uint16_t tof_timeout_ms = 100;
  uint32_t tof_resume_grace_us = 100000;
};
inline void fail(SensorStatus &s, SensorError error) {
  s.health = SensorHealth::READ_ERROR;
  s.last_error = error;
  if (s.error_count != UINT32_MAX)
    ++s.error_count;
  if (s.consecutive_errors != UINT16_MAX)
    ++s.consecutive_errors;
}
inline void success(SensorStatus &s, uint32_t now) {
  s.timestamp_us = now;
  s.health = SensorHealth::OK;
  s.consecutive_errors = 0;
}
inline void expire(SensorStatus &s, uint32_t now, uint32_t age) {
  if ((s.health == SensorHealth::OK || s.health == SensorHealth::OUT_OF_RANGE) &&
      !is_sensor_fresh(s.timestamp_us, now, age))
    s.health = SensorHealth::STALE;
}
inline SensorData *distances(RobotState &s, unsigned i) {
  SensorData *slots[] = {&s.dist_front,          &s.dist_right,
                         &s.dist_left,           &s.dist_side_front_left,
                         &s.dist_side_rear_left, &s.dist_side_front_right,
                         &s.dist_side_rear_right};
  return slots[i];
}
inline const SensorData *distances(const RobotState &s, unsigned i) {
  return distances(const_cast<RobotState &>(s), i);
}
inline void expire(SensorData &s, uint32_t now, uint32_t age) {
  if ((s.health == SensorHealth::OK ||
       s.health == SensorHealth::OUT_OF_RANGE) &&
      !is_sensor_fresh(s.timestamp_us, now, age))
    s.health = SensorHealth::READ_ERROR;
}
inline void expire_all(RobotState &s, uint32_t now) {
  for (unsigned i = 0; i < 7; ++i)
    expire(*distances(s, i), now, tof_age_us);
  expire(s.imu, now, imu_age_us);
}
inline bool available(const SensorData &s, uint32_t now) {
  return is_sensor_available(s, now, tof_age_us);
}
inline bool required(const RobotState &s, uint32_t now, bool diagonals = true) {
  return available(s.dist_front, now) &&
         (!diagonals || available(s.dist_left, now) ||
          available(s.dist_right, now));
}
inline uint16_t age_ms(uint32_t timestamp, uint32_t now) {
  return timestamp == 0
             ? UINT16_MAX
             : std::min<uint32_t>((now - timestamp) / 1000, UINT16_MAX - 1);
}
// Recovery history is separate from driver objects and survives reconstruction.
struct Backoff {
  uint32_t delay_ms = 500, retry_at_ms = 0, healthy_since_us = 0;
  bool attempted = false;
  uint32_t initial_ms = 500, maximum_ms = 5000, healthy_window_us = healthy_us;
  void configure(const Settings &settings) {
    initial_ms = delay_ms = settings.recovery_initial_ms;
    maximum_ms = settings.recovery_max_ms;
    healthy_window_us = settings.recovery_healthy_ms * 1000;
  }
  bool due(uint32_t now_ms) const {
    return static_cast<int32_t>(now_ms - retry_at_ms) >= 0;
  }
  void fault() { healthy_since_us = 0; }
  void defer(uint32_t now_ms) {
    if (attempted)
      delay_ms = std::min<uint32_t>(delay_ms * 2, maximum_ms);
    attempted = true;
    retry_at_ms = now_ms + delay_ms;
    healthy_since_us = 0;
  }
  void acquired(uint32_t now_us) {
    if (!healthy_since_us)
      healthy_since_us = now_us;
    if (now_us - healthy_since_us >= healthy_window_us) {
      delay_ms = initial_ms;
      attempted = false;
    }
  }
};
} // namespace sensor_policy
