#pragma once
#include "config.h"
#include "obstacle_navigation.h"
#include "state.h"

// Clear the attempt count after sustained clear forward driving or as soon as
// the forced rocking maneuver finishes.
class EscapeRecovery {
public:
  void reset() {
    m_phase = Phase::INACTIVE;
    m_attempts = 0;
    m_clear_tracking = false;
    m_alternate = false;
  }

  void observe(const RobotState &s, uint16_t reverse_distance_mm) {
    bool clear =
        !active() && s.running_state == RunningState::RUNNING &&
        s.current_throttle > config::stall_min_throttle &&
        s.target_throttle > config::stall_min_throttle &&
        sensor_policy::available(s.dist_front, s.current_time_us) &&
        s.dist_front.distance_mm >=
            static_cast<uint32_t>(reverse_distance_mm) +
                config::reverse_clearance_margin_mm;
    if (!clear) {
      m_clear_tracking = false;
    } else if (!m_clear_tracking) {
      m_clear_tracking = true;
      m_clear_since_us = s.current_time_us;
    } else if (s.current_time_us - m_clear_since_us >=
               config::escape_clear_forward_ms * 1000U) {
      reset();
    }
  }

  // Return true when this attempt replaces the ordinary short reverse.
  bool begin_attempt(const RobotState &s, float fallback_direction) {
    m_clear_tracking = false;
    if (m_attempts < config::escape_reverse_attempts)
      ++m_attempts;
    if (m_attempts < config::escape_reverse_attempts)
      return false;
    if (!m_alternate)
      m_forward_direction = ObstacleAvoidance::escape_direction(
          s, s.current_time_us, fallback_direction);
    else
      m_forward_direction = -m_forward_direction;
    // Following attempts alternate relative to the last choice, rather than
    // re-reading the same misleading gap and repeating the same maneuver.
    m_alternate = true;
    m_phase = Phase::BACKUP;
    m_started_us = s.current_time_us;
    m_command_started = false;
    return true;
  }

  bool active() const { return m_phase != Phase::INACTIVE; }

  void update(RobotState &s) {
    if (s.current_time_us - m_started_us >= config::escape_timeout_ms * 1000U) {
      finish(s);
      return;
    }
    bool forward = m_phase == Phase::FORWARD;
    bool command_ready = forward
                             ? s.current_throttle > config::stall_min_throttle
                             : s.current_throttle < -config::stall_min_throttle;
    if (!m_command_started && command_ready) {
      m_command_started = true;
      m_command_since_us = s.current_time_us;
    }
    uint32_t duration_ms = forward ? config::escape_forward_ms
                           : m_phase == Phase::BACKUP
                               ? config::escape_backup_ms
                               : config::escape_reverse_turn_ms;
    if (m_command_started &&
        s.current_time_us - m_command_since_us >= duration_ms * 1000U) {
      if (forward) {
        finish(s);
        return;
      }
      m_phase = m_phase == Phase::BACKUP ? Phase::REVERSE_TURN : Phase::FORWARD;
      m_command_started = false;
      forward = m_phase == Phase::FORWARD;
      command_ready = !forward; // Center through the reverse-to-forward ramp.
    }
    s.running_state = forward                    ? RunningState::RUNNING
                      : m_phase == Phase::BACKUP ? RunningState::REVERSE
                                                 : RunningState::REVERSE_TURN;
    s.target_throttle = forward ? s.base_throttle : -s.reverse_throttle;
    s.target_steering = m_phase == Phase::BACKUP || !command_ready ? 0.0f
                        : forward ? m_forward_direction
                                  : -m_forward_direction;
  }

private:
  enum class Phase : uint8_t { INACTIVE, BACKUP, REVERSE_TURN, FORWARD };

  void finish(RobotState &s) {
    m_phase = Phase::INACTIVE;
    m_attempts = 0;
    m_clear_tracking = false;
    s.running_state = RunningState::RUNNING;
    s.target_throttle = s.target_steering = 0.0f;
  }

  Phase m_phase = Phase::INACTIVE;
  uint8_t m_attempts = 0;
  bool m_clear_tracking = false, m_command_started = false, m_alternate = false;
  uint32_t m_clear_since_us = 0, m_started_us = 0, m_command_since_us = 0;
  float m_forward_direction = 1.0f;
};
