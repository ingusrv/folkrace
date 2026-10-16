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

static void sustained_progress() {
  for (unsigned moving_slot = 1; moving_slot < 7; ++moving_slot) {
    StallDetector detector;
    auto s = running();
    for (uint32_t t = 1000; t <= 6001000; t += 10000) {
      // Translation is corroborated by another beam, even at constant speed.
      s.dist_front.distance_mm = 400 + (t - 1000) / 20000;
      sensor_policy::distances(s, moving_slot)->distance_mm =
          400 + (t - 1000) / 20000;
      assert(!tick(detector, s, t, false));
    }
  }
  StallDetector detector;
  auto s = running();
  for (uint32_t t = 1000; t <= 6001000; t += 10000) {
    s.imu.measurement.yaw = (t - 1000) * 30.0f * 1e-6f;
    assert(!tick(detector, s, t));
  }
}

static void vibration_and_outliers() {
  StallDetector detector;
  auto s = running();
  for (uint32_t t = 1000; t <= 2001000; t += 10000) {
    bool alternate = (t / 10000) % 2;
    s.dist_front.distance_mm = alternate ? 550 : 400;
    s.dist_side_front_right.distance_mm = alternate ? 1000 : 700;
    s.imu.measurement.roll = alternate ? 40.0f : -40.0f;
    s.imu.measurement.pitch = alternate ? 8.0f : -8.0f;
    s.imu.measurement.accel_x = alternate ? 12.0f : -12.0f;
    s.imu.measurement.yaw = alternate ? 2.0f : -2.0f;
    s.current_time_us = t;
    for (unsigned i = 0; i < 7; ++i)
      sensor_policy::success(*sensor_policy::distances(s, i), t);
    s.dist_side_front_right.health =
        alternate ? SensorHealth::OUT_OF_RANGE : SensorHealth::OK;
    sensor_policy::success(s.imu, t);
    assert(detector.update(s, true, 1) == (t == 2001000));
  }
  // A single sustained range change also cannot veto four quiet wall beams.
  detector.reset();
  s = running();
  for (uint32_t t = 1000; t <= 2001000; t += 10000) {
    s.dist_front.distance_mm = 400 + (t - 1000) / 20000;
    assert(tick(detector, s, t) == (t == 2001000));
  }
}

static void optional_sensor_changes_and_blind_ranges() {
  for (bool blind : {false, true}) {
    StallDetector detector;
    auto s = running();
    for (uint32_t t = 1000; t <= 2001000; t += 10000) {
      s.current_time_us = t;
      for (unsigned i = 0; i < 7; ++i) {
        auto &beam = *sensor_policy::distances(s, i);
        sensor_policy::success(beam, t);
        if (blind) {
          beam.health = SensorHealth::OUT_OF_RANGE;
          beam.distance_mm = 1000;
        }
      }
      if (!blind && (t / 10000) % 2)
        s.dist_side_rear_right.health = SensorHealth::READ_ERROR;
      assert(detector.update(s, false, 1) == (!blind && t == 2001000));
    }
  }
}

static void modes_freshness_and_wrap() {
  StallDetector detector;
  auto s = running();
  for (RunningState mode :
       {RunningState::IDLE, RunningState::COUNTDOWN, RunningState::REVERSE,
        RunningState::REVERSE_TURN, RunningState::WRONG_WAY_RECOVERY}) {
    s.running_state = mode;
    assert(!tick(detector, s, 1000));
    s.running_state = RunningState::RUNNING;
    assert(!tick(detector, s, 11000));
  }
  s.current_throttle = -0.1f;
  assert(!tick(detector, s, 1001000));
  s.current_throttle = 0.3f;
  s.target_throttle = 0.0f;
  assert(!tick(detector, s, 2001000));
  s.target_throttle = 0.3f;
  assert(!tick(detector, s, 2011000));
  s.current_time_us = 2211000; // No new frames: stale sensors cannot stall.
  assert(!detector.update(s, false, 1));

  detector.reset();
  uint32_t start_us = UINT32_MAX - 1000000;
  for (uint32_t elapsed = 0; elapsed <= 2000000; elapsed += 10000)
    assert(tick(detector, s, start_us + elapsed, false) ==
           (elapsed == 2000000));

  for (unsigned change = 0; change < 2; ++change) {
    detector.reset();
    s = running();
    for (uint32_t t = 1000; t <= 3001000; t += 10000) {
      bool returned = t >= 1001000;
      bool imu_ready = change == 0 ? returned : true;
      uint32_t generation = change == 1 && returned ? 2 : 1;
      assert(tick(detector, s, t, imu_ready, generation) == (t == 3001000));
    }
  }
}

int main() {
  quiet_and_sensor_only_stalls();
  sustained_progress();
  vibration_and_outliers();
  optional_sensor_changes_and_blind_ranges();
  modes_freshness_and_wrap();
  puts("stall detection tests passed");
}
