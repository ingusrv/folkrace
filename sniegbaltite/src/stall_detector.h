#pragma once
#include "config.h"
#include "sensors/health.h"
#include "state.h"
#include <cmath>

// IMU cannot measure constant speed. Require quiet distances as well as quiet
// IMU signals; when IMU is unavailable, use fresh distances alone.
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

    uint8_t mask = 0;
    for (unsigned i = 0; i < 7; ++i)
      if (sensor_policy::available(*sensor_policy::distances(s, i),
                                   s.current_time_us))
        mask |= static_cast<uint8_t>(1U << i);
    // Restart on sensor loss/return or fusion reset, never bridge blind time.
    if (!m_tracking || mask != m_distance_mask || imu_ready != m_imu_ready ||
        (imu_ready && generation != m_generation)) {
      m_filtered_accel[0] = s.imu.measurement.accel_x;
      m_filtered_accel[1] = s.imu.measurement.accel_y;
      m_filtered_accel[2] = s.imu.measurement.accel_z;
      m_last_imu_us = s.imu.timestamp_us;
      m_distance_mask = mask;
      m_imu_ready = imu_ready;
      m_generation = generation;
      anchor(s);
      return false;
    }

    bool moved = false;
    for (unsigned i = 0; i < 7; ++i) {
      if (!(mask & (1U << i)))
        continue;
      int change =
          static_cast<int>(sensor_policy::distances(s, i)->distance_mm) -
          m_distances_mm[i];
      moved |= std::abs(change) >= config::stall_distance_change_mm;
    }
    if (imu_ready && s.imu.timestamp_us != m_last_imu_us) {
      uint32_t dt_us = s.imu.timestamp_us - m_last_imu_us;
      m_last_imu_us = s.imu.timestamp_us;
      float alpha = static_cast<float>(dt_us) /
                    (config::stall_accel_filter_us + static_cast<float>(dt_us));
      const auto &imu = s.imu.measurement;
      float accel[] = {imu.accel_x, imu.accel_y, imu.accel_z};
      float accel_change_squared = 0.0f;
      for (unsigned i = 0; i < 3; ++i) {
        m_filtered_accel[i] += alpha * (accel[i] - m_filtered_accel[i]);
        float change = m_filtered_accel[i] - m_accel_anchor[i];
        accel_change_squared += change * change;
      }
      // Yaw is continuous; roll/pitch wrap at +/-180 degrees.
      moved |=
          std::fabs(imu.yaw - m_orientation.yaw) >=
              config::stall_rotation_change_deg ||
          std::fabs(std::remainder(imu.pitch - m_orientation.pitch, 360.0f)) >=
              config::stall_rotation_change_deg ||
          std::fabs(std::remainder(imu.roll - m_orientation.roll, 360.0f)) >=
              config::stall_rotation_change_deg ||
          accel_change_squared >=
              config::stall_accel_change_mps2 * config::stall_accel_change_mps2;
    }
    if (moved) {
      anchor(s);
      return false;
    }
    return s.current_time_us - m_stationary_since_us >=
           static_cast<uint32_t>(config::stall_timeout_ms) * 1000;
  }

private:
  void anchor(const RobotState &s) {
    m_tracking = true;
    m_stationary_since_us = s.current_time_us;
    for (unsigned i = 0; i < 7; ++i)
      m_distances_mm[i] = sensor_policy::distances(s, i)->distance_mm;
    m_orientation = s.imu.measurement;
    for (unsigned i = 0; i < 3; ++i)
      m_accel_anchor[i] = m_filtered_accel[i];
  }

  bool m_tracking = false, m_imu_ready = false;
  uint8_t m_distance_mask = 0;
  uint16_t m_distances_mm[7] = {};
  uint32_t m_stationary_since_us = 0, m_generation = 0, m_last_imu_us = 0;
  IMUState m_orientation = {};
  float m_filtered_accel[3] = {}, m_accel_anchor[3] = {};
};
