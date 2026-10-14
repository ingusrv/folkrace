#include "stall_detector.h"
#include <cassert>
#include <cstdio>

static RobotState running(uint32_t now_us = 1000) {
  RobotState s{};
  s.running_state = RunningState::RUNNING;
  s.current_throttle = s.target_throttle = 0.3f;
  s.imu.measurement.accel_z = 9.807f;
  for (unsigned i = 0; i < 7; ++i)
    sensor_policy::distances(s, i)->distance_mm = 400;
  s.current_time_us = now_us;
  return s;
}

static bool tick(StallDetector &detector, RobotState &s, uint32_t now_us,
                 bool imu_ready = true, uint32_t generation = 1) {
  s.current_time_us = now_us;
  for (unsigned i = 0; i < 7; ++i)
    sensor_policy::success(*sensor_policy::distances(s, i), now_us);
  sensor_policy::success(s.imu, now_us);
  return detector.update(s, imu_ready, generation);
}

static void quiet_and_sensor_only_stalls() {
  for (bool imu_ready : {false, true}) {
    StallDetector detector;
    auto s = running();
    for (uint32_t t = 1000; t < 2001000; t += 10000) {
      // Sub-threshold sensor noise and vibration must not refresh the timer.
      s.dist_front.distance_mm = (t / 10000 % 2) ? 410 : 390;
      s.imu.measurement.accel_x = (t / 10000 % 2) ? 0.2f : -0.2f;
      assert(!tick(detector, s, t, imu_ready));
    }
    assert(tick(detector, s, 2001000, imu_ready));
    detector.reset(); // Recovery/inhibit starts a new full observation window.
    assert(!tick(detector, s, 2011000, imu_ready));
    assert(!tick(detector, s, 4001000, imu_ready));
    assert(tick(detector, s, 4011000, imu_ready));
  }
}

static void cumulative_movement_and_imu_motion() {
  for (unsigned moving_slot = 0; moving_slot < 7; ++moving_slot) {
    StallDetector detector;
    auto s = running();
    for (uint32_t t = 1000; t <= 5001000; t += 10000) {
      sensor_policy::distances(s, moving_slot)->distance_mm =
          400 + (t - 1000) / 20000; // Slow drift, small frame-to-frame changes.
      assert(!tick(detector, s, t, false));
    }
  }
  for (unsigned axis = 0; axis < 4; ++axis) {
    StallDetector detector;
    auto s = running();
    for (uint32_t t = 1000; t <= 5001000; t += 10000) {
      float elapsed = (t - 1000) * 1e-6f;
      if (axis == 0)
        s.imu.measurement.yaw = elapsed * 5.0f;
      else if (axis == 1)
        s.imu.measurement.pitch = elapsed * 5.0f;
      else if (axis == 2)
        s.imu.measurement.roll = elapsed * 5.0f;
      else
        s.imu.measurement.accel_x = elapsed * 2.0f;
      assert(!tick(detector, s, t));
    }
  }
}

static void modes_freshness_and_wrap() {
  StallDetector detector;
  auto s = running();
  assert(!tick(detector, s, 1000));
  // An unavailable IMU's arbitrary values cannot postpone sensor-only stall.
  s.imu.measurement.yaw = 1000.0f;
  assert(!tick(detector, s, 1901000, false));
  s.imu.measurement.yaw = -1000.0f;
  assert(!tick(detector, s, 3900000, false));
  assert(tick(detector, s, 3901000, false));
  assert(!tick(detector, s, 3911000, true));    // IMU returned: restart.
  assert(!tick(detector, s, 5901000, true, 2)); // Fusion reset: restart.
  assert(!tick(detector, s, 7900000, true, 2));
  assert(tick(detector, s, 7901000, true, 2));

  for (RunningState mode :
       {RunningState::IDLE, RunningState::COUNTDOWN, RunningState::REVERSE,
        RunningState::REVERSE_TURN, RunningState::WRONG_WAY_RECOVERY}) {
    s.running_state = mode;
    assert(!tick(detector, s, 8001000));
    s.running_state = RunningState::RUNNING;
    assert(!tick(detector, s, 8011000));
  }
  s.current_throttle = -0.1f;
  assert(!tick(detector, s, 11001000));
  s.current_throttle = 0.3f;
  s.target_throttle = 0.0f;
  assert(!tick(detector, s, 13001000));
  s.target_throttle = 0.3f;
  assert(!tick(detector, s, 13011000));
  s.current_time_us = 13211000; // No new frames: stale sensors cannot stall.
  assert(!detector.update(s, false, 1));
  assert(!tick(detector, s, 15011000));

  detector.reset();
  uint32_t start_us = UINT32_MAX - 1000000;
  assert(!tick(detector, s, start_us, false));
  assert(!tick(detector, s, start_us + 1990000U, false));
  assert(tick(detector, s, start_us + 2000000U, false));
}

int main() {
  quiet_and_sensor_only_stalls();
  cumulative_movement_and_imu_motion();
  modes_freshness_and_wrap();
  puts("stall detection tests passed");
}
