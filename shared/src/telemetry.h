#pragma once
#include "protocol.h"
#include "sensors/health.h"

/** Build protocol v1 diagnostics from the control tick's copied state. */
inline TelemetryPacket make_telemetry(const RobotState &s, uint8_t flags,
                                      bool diagonals = true) {
  TelemetryPacket p = {};
  p.protocol_version = TELEMETRY_PROTOCOL_VERSION;
  p.timestamp_ms = s.current_time_ms;
  p.running_state = s.running_state;
  p.flags = flags |
            (sensor_policy::required(s, s.current_time_us, diagonals) ? 8 : 0);
  p.dist_front = s.dist_front.distance_mm;
  p.dist_right = s.dist_right.distance_mm;
  p.dist_left = s.dist_left.distance_mm;
  p.dist_side_front_left = s.dist_side_front_left.distance_mm;
  p.dist_side_rear_left = s.dist_side_rear_left.distance_mm;
  p.dist_side_front_right = s.dist_side_front_right.distance_mm;
  p.dist_side_rear_right = s.dist_side_rear_right.distance_mm;
  p.target_steering = s.target_steering;
  p.current_steering = s.current_steering;
  p.target_throttle = s.target_throttle;
  p.current_throttle = s.current_throttle;
  p.pid_error = s.steering_pid.error;
  p.pid_integral = s.steering_pid.integral;
  p.pid_derivative = s.steering_pid.derivative;
  p.accel_x = s.imu.measurement.accel_x;
  p.accel_y = s.imu.measurement.accel_y;
  p.accel_z = s.imu.measurement.accel_z;
  p.gyro_x = s.imu.measurement.gyro_x;
  p.gyro_y = s.imu.measurement.gyro_y;
  p.gyro_z = s.imu.measurement.gyro_z;
  p.roll = s.imu.measurement.roll;
  p.pitch = s.imu.measurement.pitch;
  p.yaw = s.imu.measurement.yaw;
  for (unsigned i = 0; i < 8; ++i) {
    const SensorStatus &d = i == 7 ? static_cast<const SensorStatus &>(s.imu)
                                   : static_cast<const SensorStatus &>(
                                         *sensor_policy::distances(s, i));
    p.sensors[i] = {
        static_cast<uint8_t>(d.health),
        sensor_policy::age_ms(d.timestamp_us, s.current_time_us),
        static_cast<uint16_t>(std::min<uint32_t>(d.error_count, UINT16_MAX)),
        d.consecutive_errors, static_cast<uint8_t>(d.last_error)};
    if (i < 7) {
      const auto &raw = *sensor_policy::distances(s, i);
      p.raw_tof[i] = {
          raw.raw_range_mm, raw.raw_status,
          sensor_policy::age_ms(raw.raw_timestamp_us, s.current_time_us)};
    }
  }
  return p;
}
