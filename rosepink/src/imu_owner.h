#pragma once
#include "config.h"
#include "driver/bmi088.h"
#include "sensor_snapshots.h"
#include "sensors/health.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <math.h>

#if __has_include(<BMI088.h>)
namespace sensor_policy {
// Rosepink owns a dedicated IMU bus; calibration handshake matches the shared
// owner, without coupling firmware uploads or ToF recovery to IMU attempts.
class RosepinkIMUOwner {
public:
  RosepinkIMUOwner(TwoWire &bus, QueueHandle_t snapshots,
                   QueueHandle_t commands, const Settings &settings)
      : m_bus(bus), m_imu(bus), m_snapshots(snapshots), m_commands(commands),
        m_min_samples(config::imu_min_calib_samples),
        m_max_attempts(config::imu_max_init_attempts_per_boot),
        m_settings(settings) {
    m_backoff.configure(settings);
  }
  MainSensorSnapshot snapshot;
  uint32_t init_duration_us = 0;
  void begin() {
    m_bus.begin(pin::sda, pin::scl, 400000);
    m_bus.setTimeOut(m_settings.bus_timeout_ms);
    publish();
  }
  void tick() {
    process_commands();
    if (m_active && !m_disabled)
      acquire();
    publish();
    process_commands();
    // Give calibration commands a normal cycle after each blocking operation.
    if (m_resume_cycle) {
      m_resume_cycle = false;
      return;
    }
    if (m_need_init && !m_disabled && m_backoff.due(millis()))
      initialize();
  }

private:
  void publish() {
    snapshot.publish_timestamp_us = micros();
    xQueueOverwrite(m_snapshots, &snapshot);
  }
  void disable() {
    snapshot.imu_disabled_until_reboot = true;
    m_disabled = true;
    m_active = false;
    m_need_init = false;
    snapshot.imu.health = SensorHealth::OFFLINE;
    publish();
  }
  void recovery_needed() {
    m_active = false;
    m_need_init = true;
    m_backoff.defer(millis());
    if (m_attempts >= m_max_attempts)
      disable();
    else {
      snapshot.imu.health = SensorHealth::OFFLINE;
      publish();
    }
  }
  void initialize() {
    if (m_attempts >= m_max_attempts) {
      disable();
      return;
    }
    snapshot.imu.health = SensorHealth::RECOVERING;
    publish();
    ++m_attempts; // Lifetime budget, including successful calls and bus
                  // recovery.
    uint32_t start = micros();
    bool ok = m_imu.begin();
    init_duration_us = micros() - start;
    m_bus.setClock(400000);
    m_bus.setTimeOut(m_settings.bus_timeout_ms);
    m_resume_cycle = true;
    if (!ok) {
      fail(snapshot.imu, SensorError::INIT_FAILED);
      publish();
      recovery_needed();
      return;
    }
    if (snapshot.imu_calib_valid)
      m_imu.set_gyro_biases(m_bx, m_by, m_bz);
    m_imu.reset_orientation();
    m_new_frame = true;
    m_active = true;
    m_need_init = false;
    m_wait_first = true;
    m_first_deadline_us = micros();
    snapshot.imu.health = SensorHealth::INITIALIZING;
    publish();
  }
  void acquire() {
    IMUSample sample;
    SensorError error;
    if (!m_imu.read_sample(sample, error)) {
      fail(snapshot.imu, error);
      m_backoff.fault();
      publish();
      if (snapshot.imu.consecutive_errors >=
              m_settings.max_consecutive_errors ||
          (m_wait_first &&
           micros() - m_first_deadline_us >= m_settings.imu_freshness_us))
        recovery_needed();
      return;
    }
    uint32_t now = micros();
    if (m_new_frame || !snapshot.imu.timestamp_us ||
        now - snapshot.imu.timestamp_us > m_settings.imu_freshness_us) {
      m_imu.reset_orientation();
      ++snapshot.orientation_generation;
      m_new_frame = false;
    }
    m_imu.fuse(sample, now);
    snapshot.imu.measurement = m_imu.get_state();
    success(snapshot.imu, now);
    m_wait_first = false;
    m_backoff.acquired(now);
    if (snapshot.imu_calib_in_progress &&
        snapshot.imu_calib_sample_count < UINT16_MAX) {
      m_sum[0] += sample.gx;
      m_sum[1] += sample.gy;
      m_sum[2] += sample.gz;
      m_sum[3] += sample.ax;
      m_sum[4] += sample.ay;
      m_sum[5] += sample.az;
      ++snapshot.imu_calib_sample_count;
    }
  }
  void clear_attempt() {
    for (auto &sum : m_sum)
      sum = 0.0f;
    snapshot.imu_calib_sample_count = 0;
    snapshot.imu_calib_in_progress = false;
    snapshot.imu_calib_completed = false;
  }
  void process_commands() {
    SensorCommand command;
    while (xQueueReceive(m_commands, &command, 0) == pdTRUE) {
      if (command.cmd_id == snapshot.last_cmd_id_ack) {
        publish();
        continue;
      }
      // Queue is ordered and the controller has only one unacknowledged
      // command.
      if (static_cast<int32_t>(command.cmd_id - snapshot.last_cmd_id_ack) <= 0)
        continue;
      snapshot.last_cmd_id_ack = command.cmd_id;
      auto &result = snapshot.last_cmd_result;
      result = SensorCommandResult::REJECTED;
      switch (command.type) {
      case SensorCommandType::START_IMU_CALIBRATION:
        if (!m_disabled && !snapshot.imu_calib_valid &&
            !snapshot.imu_calib_in_progress) {
          clear_attempt();
          snapshot.imu_calib_in_progress = true;
          snapshot.imu_calib_start_cmd_id = command.cmd_id;
          result = SensorCommandResult::ACCEPTED;
        }
        break;
      case SensorCommandType::FINISH_IMU_CALIBRATION:
        if (!snapshot.imu_calib_in_progress || m_disabled)
          break;
        if (snapshot.imu_calib_sample_count < m_min_samples) {
          result = SensorCommandResult::INSUFFICIENT_SAMPLES;
          break;
        }
        {
          float n = snapshot.imu_calib_sample_count;
          m_bx = m_sum[0] / n;
          m_by = m_sum[1] / n;
          m_bz = m_sum[2] / n;
          snapshot.imu_calib_flat_pitch =
              atan2f(m_sum[4],
                     sqrtf(m_sum[3] * m_sum[3] + m_sum[5] * m_sum[5])) *
              180.0f / PI;
          m_imu.set_gyro_biases(m_bx, m_by, m_bz);
          m_imu.reset_orientation();
          // Publish the generation only with the first usable orientation.
          m_new_frame = true;
          snapshot.imu.health = SensorHealth::INITIALIZING;
          snapshot.imu_calib_valid = true;
          snapshot.imu_calib_completed = true;
          snapshot.imu_calib_in_progress = false;
          result = SensorCommandResult::COMPLETED;
        }
        break;
      case SensorCommandType::ABORT_IMU_CALIBRATION:
        clear_attempt();
        result = SensorCommandResult::ABORTED;
        break;
      }
      publish();
    }
  }
  TwoWire &m_bus;
  BMI088Driver m_imu;
  QueueHandle_t m_snapshots, m_commands;
  uint16_t m_min_samples;
  uint8_t m_max_attempts, m_attempts = 0;
  Settings m_settings;
  bool m_active = false, m_need_init = true, m_disabled = false;
  bool m_new_frame = true, m_wait_first = false, m_resume_cycle = false;
  uint32_t m_first_deadline_us = 0;
  Backoff m_backoff;
  float m_sum[6] = {}, m_bx = 0.0f, m_by = 0.0f, m_bz = 0.0f;
};
} // namespace sensor_policy
#endif
