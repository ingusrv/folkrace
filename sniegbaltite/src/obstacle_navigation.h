#pragma once
#include "config.h"
#include "control/sensor_guidance.h"
#include "sensors/health.h"
#include <algorithm>
#include <cmath>

// Local to Sniegbaltite until the pole/recovery behavior is tested on track.
class ObstacleAvoidance {
public:
  void reset() {
    m_locked = m_spent = false;
    m_urgency = m_wall_steering = 0.0f;
  }

  void update(const RobotState &s, uint32_t now_us, bool imu_ready = false,
              uint32_t generation = 0) {
    uint16_t left =
        wall_distance(s.dist_side_front_left, s.dist_side_rear_left, now_us);
    uint16_t right =
        wall_distance(s.dist_side_front_right, s.dist_side_rear_right, now_us);
    auto pressure = [](uint16_t mm) {
      return std::max(0.0f, (float(config::side_danger_dist) - mm) /
                                config::side_danger_dist);
    };
    m_wall_steering = pressure(left) - pressure(right);
    bool visible = sensor_policy::available(s.dist_front, now_us);
    uint16_t front = s.dist_front.distance_mm;
    m_urgency =
        visible
            ? std::max(0.0f,
                       std::min(1.0f, (float(config::front_turn_dist) - front) /
                                          (config::front_turn_dist -
                                           config::front_critical_dist)))
            : 0.0f;
    // Once the obstacle leaves the front beam, hand steering straight back to
    // wall following. No timed tail or forced turn when forward drive resumes.
    if (!visible || front >= config::front_turn_dist +
                                 config::avoidance_release_margin_mm) {
      m_locked = m_spent = false;
      return;
    }
    if (m_wall_steering != 0.0f) {
      m_locked = false;
      m_spent = true; // A nearby wall overrides the previous passing side.
      return;
    }
    if (front >= config::front_turn_dist) {
      m_spent = m_spent || m_locked;
      m_locked = false;
      return;
    }
    if (!m_locked && !m_spent && front < config::front_turn_dist) {
      m_direction = escape_direction(s, now_us);
      m_locked = true;
      m_started_us = now_us;
      m_start_yaw_deg = s.imu.measurement.yaw;
      m_yaw_available = imu_ready;
      m_generation = generation;
    }
    if (m_locked) {
      if (!imu_ready || generation != m_generation)
        m_yaw_available = false;
      bool turned = m_yaw_available &&
                    std::fabs(s.imu.measurement.yaw - m_start_yaw_deg) >=
                        config::avoidance_max_yaw_deg;
      if (turned ||
          now_us - m_started_us >=
              static_cast<uint32_t>(config::avoidance_max_ms) * 1000) {
        m_locked = false;
        m_spent = true; // Don't renew a pulse on every still-close front frame.
      }
    }
  }

  // Recover with ordinary wall following, not a second forced passing turn.
  void resume(uint32_t) {
    reset();
    m_spent = true;
  }

  float direction() const {
    return m_wall_steering > 0.0f   ? 1.0f
           : m_wall_steering < 0.0f ? -1.0f
                                    : m_direction;
  }
  float urgency() const { return m_urgency; }

  // Baseline guidance deliberately excludes front anticipation: this object's
  // bounded pulse supplies it. Otherwise the shared blend keeps forcing a turn
  // after the pulse has ended and suppresses emergency side-wall repulsion.
  SensorGuidance guidance(const RobotState &s, uint32_t now_us) const {
    RobotState wall_state = s;
    wall_state.dist_front.health = SensorHealth::ABSENT;
    return sensor_guidance(wall_state, now_us, config::side_turn_range,
                           config::side_danger_dist, config::front_turn_dist,
                           config::front_critical_dist, config::sensor_max_raw);
  }

  float steer(float target) const {
    if (m_wall_steering > 0.0f)
      return std::max(target, m_wall_steering);
    if (m_wall_steering < 0.0f)
      return std::min(target, m_wall_steering);
    if (!m_locked)
      return target;
    return m_direction > 0.0f ? std::max(target, m_urgency)
                              : std::min(target, -m_urgency);
  }

  // Recovery re-evaluates current space instead of trusting an old pole latch.
  static float escape_direction(const RobotState &s, uint32_t now_us,
                                float fallback = 0.0f) {
    auto clearance = [now_us](const SensorData &diagonal,
                              const SensorData &front, const SensorData &rear,
                              bool &seen) {
      uint16_t distance = config::sensor_max_raw;
      for (const SensorData *d : {&diagonal, &front, &rear}) {
        if (sensor_policy::available(*d, now_us)) {
          seen = true;
          distance = std::min(distance, d->distance_mm);
        }
      }
      return distance;
    };
    bool left_seen = false, right_seen = false;
    uint16_t left = clearance(s.dist_left, s.dist_side_front_left,
                              s.dist_side_rear_left, left_seen);
    uint16_t right = clearance(s.dist_right, s.dist_side_front_right,
                               s.dist_side_rear_right, right_seen);
    if (left_seen && right_seen &&
        std::abs(static_cast<int>(right) - left) >=
            config::avoidance_direction_margin_mm)
      return right > left ? 1.0f : -1.0f;
    if (left_seen && !right_seen && left < config::side_turn_range)
      return 1.0f;
    if (right_seen && !left_seen && right < config::side_turn_range)
      return -1.0f;
    if (fallback != 0.0f)
      return fallback;
    return s.track_direction == TrackDirection::CLOCKWISE ? 1.0f : -1.0f;
  }

private:
  static uint16_t wall_distance(const SensorData &front, const SensorData &rear,
                                uint32_t now_us) {
    return std::min(
        sensor_policy::available(front, now_us) ? front.distance_mm
                                                : config::sensor_max_raw,
        sensor_policy::available(rear, now_us) ? rear.distance_mm
                                               : config::sensor_max_raw);
  }
  bool m_locked = false, m_spent = false, m_yaw_available = false;
  float m_direction = 1.0f, m_urgency = 0.0f, m_wall_steering = 0.0f;
  float m_start_yaw_deg = 0.0f;
  uint32_t m_started_us = 0, m_generation = 0;
};

// CSV calibration: right steering gives positive gyro_z/yaw, left gives
// negative. CW progress therefore advances the maximum, CCW the minimum.
inline float heading_deviation(TrackDirection direction, float yaw_deg,
                               float &maximum_deg, float &minimum_deg) {
  if (direction == TrackDirection::CLOCKWISE) {
    maximum_deg = std::max(maximum_deg, yaw_deg);
    return maximum_deg - yaw_deg;
  }
  minimum_deg = std::min(minimum_deg, yaw_deg);
  return yaw_deg - minimum_deg;
}

class ReverseRecovery {
public:
  void reset() { m_phase = RunningState::RUNNING; }

  void begin(const RobotState &s, float forward_direction, bool imu_ready,
             uint32_t generation) {
    m_phase = RunningState::REVERSE;
    m_started_us = s.current_time_us;
    m_motion_started = false;
    m_motion_start_us = 0;
    m_turn_start_us = 0;
    m_clear_samples = 0;
    m_last_front_timestamp_us = s.dist_front.timestamp_us;
    m_reverse_steering = -ObstacleAvoidance::escape_direction(
        s, s.current_time_us,
        forward_direction); // Same nose rotation while backing.
    m_start_yaw_deg = s.imu.measurement.yaw;
    m_yaw_available = imu_ready;
    m_generation = generation;
  }

  RunningState update(const RobotState &s, uint16_t reverse_distance_mm,
                      bool imu_ready, uint32_t generation) {
    uint32_t now_us = s.current_time_us;
    // Never compare yaw across a fusion reset or bridge a missing IMU interval.
    if (!imu_ready || generation != m_generation)
      m_yaw_available = false;
    if (now_us - m_started_us >=
            static_cast<uint32_t>(config::reverse_total_timeout_ms) * 1000 ||
        (m_yaw_available &&
         std::fabs(s.imu.measurement.yaw - m_start_yaw_deg) >=
             config::reverse_max_yaw_deg)) {
      reset();
      return m_phase;
    }
    if (m_phase == RunningState::REVERSE) {
      if (!m_motion_started &&
          s.current_throttle <= -config::reverse_motion_throttle) {
        m_motion_started = true;
        m_motion_start_us = now_us;
      }
      uint32_t clearance_mm = std::max<uint32_t>(
          config::front_critical_dist,
          uint32_t(reverse_distance_mm) + config::reverse_clearance_margin_mm);
      if (s.dist_front.timestamp_us != m_last_front_timestamp_us) {
        m_last_front_timestamp_us = s.dist_front.timestamp_us;
        bool clear = sensor_policy::available(s.dist_front, now_us) &&
                     s.dist_front.distance_mm >= clearance_mm;
        m_clear_samples =
            clear ? std::min<unsigned>(m_clear_samples + 1, 2) : 0;
      }
      uint32_t backing_us = now_us - m_motion_start_us;
      if (m_motion_started &&
          (backing_us >=
               static_cast<uint32_t>(config::reverse_duration_ms) * 1000 ||
           (backing_us >=
                static_cast<uint32_t>(config::reverse_min_duration_ms) * 1000 &&
            m_clear_samples >= 2))) {
        m_phase = RunningState::REVERSE_TURN;
        m_turn_start_us = now_us;
      }
    } else if (m_phase == RunningState::REVERSE_TURN &&
               now_us - m_turn_start_us >=
                   static_cast<uint32_t>(config::reverse_turn_duration_ms) *
                       1000) {
      reset();
    }
    return m_phase;
  }

  float steering() const {
    return m_phase == RunningState::REVERSE_TURN ? m_reverse_steering : 0.0f;
  }

private:
  RunningState m_phase = RunningState::RUNNING;
  bool m_motion_started = false, m_yaw_available = false;
  float m_reverse_steering = 0.0f, m_start_yaw_deg = 0.0f;
  uint32_t m_started_us = 0, m_motion_start_us = 0, m_turn_start_us = 0;
  uint32_t m_generation = 0, m_last_front_timestamp_us = 0;
  unsigned m_clear_samples = 0;
};
