#pragma once
#include "sensor_snapshots.h"

// Control-owned calibration handshake. No I2C or RobotState writes in the
// producer.
class SensorStartup {
public:
  using Send = bool (*)(const SensorCommand &);
  void enter(uint32_t now_ms) {
    m_enter_ms = now_ms;
    if (!m_motion_permitted && !m_degraded && !m_abort) {
      m_started = false;
      m_completed = false;
    }
  }
  void cancel() {
    if (m_pending || m_started)
      m_abort = true;
  }
  bool imu_enabled(const MainSensorSnapshot &s) const {
    return !m_degraded && s.imu_calib_valid;
  }
  bool pending() const { return m_pending || m_abort; }
  void permitted_motion() { m_motion_permitted = true; }
  bool tick(const MainSensorSnapshot &s, bool countdown, uint32_t now_ms,
            uint32_t delay_ms, uint32_t extension_ms, uint16_t minimum,
            Send send) {
    if (!countdown)
      cancel();
    if (countdown && !m_motion_permitted && !m_degraded &&
        (!s.imu_calib_valid || m_pending || (m_started && !m_completed)) &&
        (now_ms - m_enter_ms >= delay_ms + extension_ms ||
         s.imu_disabled_until_reboot)) {
      m_abort = true;
      m_disable_after_abort = true;
    }
    if (m_pending && m_sent && s.last_cmd_id_ack == m_command.cmd_id) {
      m_pending = false;
      m_sent = false;
      if (m_command.type == SensorCommandType::START_IMU_CALIBRATION) {
        m_started = s.last_cmd_result == SensorCommandResult::ACCEPTED;
        if (!m_started) {
          m_abort = true;
          m_disable_after_abort = true;
        }
      } else if (m_command.type == SensorCommandType::FINISH_IMU_CALIBRATION) {
        m_completed = s.last_cmd_result == SensorCommandResult::COMPLETED &&
                      s.imu_calib_start_cmd_id == m_start_id &&
                      s.imu_calib_completed;
        if (m_completed)
          m_started = false;
        if (s.last_cmd_result == SensorCommandResult::REJECTED) {
          m_abort = true;
          m_disable_after_abort = true;
        }
      } else if (s.last_cmd_result == SensorCommandResult::ABORTED) {
        m_abort = false;
        m_started = false;
        m_completed = false;
        if (m_disable_after_abort)
          m_degraded = true;
        m_disable_after_abort = false;
      }
    }
    if (!m_pending) {
      if (m_abort)
        issue(SensorCommandType::ABORT_IMU_CALIBRATION);
      else if (countdown && !m_degraded && !m_motion_permitted &&
               !s.imu_calib_valid) {
        if (!m_started) {
          issue(SensorCommandType::START_IMU_CALIBRATION);
          m_start_id = m_command.cmd_id;
        } else if (now_ms - m_enter_ms >= delay_ms &&
                   s.imu_calib_sample_count >= minimum)
          issue(SensorCommandType::FINISH_IMU_CALIBRATION);
      }
    }
    if (m_pending && !m_sent)
      m_sent = send(m_command); // Retry same ID on queue-full.
    bool calibrated = s.imu_calib_valid && (!m_started || m_completed);
    return countdown && now_ms - m_enter_ms >= delay_ms && !pending() &&
           !s.imu_calib_in_progress && (calibrated || m_degraded);
  }

private:
  void issue(SensorCommandType type) {
    m_command = {++m_next_id, type};
    m_pending = true;
    m_sent = false;
  }
  SensorCommand m_command;
  uint32_t m_next_id = 0, m_start_id = 0, m_enter_ms = 0;
  bool m_pending = false, m_sent = false, m_started = false,
       m_completed = false;
  bool m_abort = false, m_disable_after_abort = false, m_degraded = false;
  bool m_motion_permitted = false;
};
