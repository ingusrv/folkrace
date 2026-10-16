#pragma once
#include "config.h"
#include "sensors/health.h"
#include "state.h"
#include <cmath>

// Compare average ranges in the two halves of a fixed observation window.
// One noisy beam, intermittent no-returns, or chassis vibration cannot keep
// refreshing the timer. IMU yaw can establish a sustained turn; acceleration,
// pitch and roll do not establish forward progress while wheels are spinning.
class StallDetector {
public:
  void reset() { m_tracking = false; }

  bool update(const RobotState &s, bool imu_ready, uint32_t generation) {
    if (s.running_state != RunningState::RUNNING ||
        s.current_throttle <= config::stall_min_throttle ||
        s.target_throttle <= config::stall_min_throttle ||
        !sensor_policy::required(s, s.current_time_us)) {
      reset();
      return false;
    }

    // Never bridge a fusion reset or a change in IMU availability. Optional
    // distance sensor changes are handled per beam, without resetting all
    // the other sensors' evidence of a stall.
    if (!m_tracking || imu_ready != m_imu_ready ||
        (imu_ready && generation != m_generation)) {
      m_tracking = true;
      m_started_us = s.current_time_us;
      m_imu_ready = imu_ready;
      m_generation = generation;
      for (unsigned half = 0; half < 2; ++half) {
        m_ticks[half] = 0;
        m_yaw_sum[half] = 0.0f;
        for (unsigned i = 0; i < 7; ++i) {
          m_distance_sum[half][i] = 0.0f;
          m_samples[half][i] = 0;
        }
      }
    }

    const uint32_t timeout_us = config::stall_timeout_ms * 1000U;
    uint32_t elapsed_us = s.current_time_us - m_started_us;
    unsigned half = elapsed_us < timeout_us / 2 ? 0 : 1;
    ++m_ticks[half];
    if (imu_ready)
      m_yaw_sum[half] += s.imu.measurement.yaw;
    for (unsigned i = 0; i < 7; ++i) {
      const auto &distance = *sensor_policy::distances(s, i);
      // OUT_OF_RANGE is usable for driving, but its max-range sentinel must not
      // be interpreted as a displacement from the last real return.
      if (distance.health == SensorHealth::OK &&
          sensor_policy::available(distance, s.current_time_us)) {
        m_distance_sum[half][i] += distance.distance_mm;
        ++m_samples[half][i];
      }
    }
    if (elapsed_us < timeout_us)
      return false;

    unsigned reliable = 0, moved = 0;
    for (unsigned i = 0; i < 7; ++i) {
      if (m_samples[0][i] == 0 || m_samples[1][i] == 0 ||
          m_samples[0][i] * 100U <
              m_ticks[0] * config::stall_min_valid_percent ||
          m_samples[1][i] * 100U < m_ticks[1] * config::stall_min_valid_percent)
        continue;
      ++reliable;
      float change = m_distance_sum[1][i] / m_samples[1][i] -
                     m_distance_sum[0][i] / m_samples[0][i];
      if (std::fabs(change) >= config::stall_distance_change_mm)
        ++moved;
    }
    bool turning =
        imu_ready && m_ticks[0] != 0 &&
        std::fabs(m_yaw_sum[1] / m_ticks[1] - m_yaw_sum[0] / m_ticks[0]) >=
            config::stall_rotation_change_deg;
    reset();
    // Need two real beams to judge translation. Progress must appear in two
    // beams, or in sustained yaw, rather than just a single noisy range.
    return reliable >= 2 && moved < 2 && !turning;
  }

private:
  bool m_tracking = false, m_imu_ready = false;
  uint32_t m_started_us = 0, m_generation = 0;
  uint16_t m_ticks[2] = {}, m_samples[2][7] = {};
  float m_distance_sum[2][7] = {}, m_yaw_sum[2] = {};
};
