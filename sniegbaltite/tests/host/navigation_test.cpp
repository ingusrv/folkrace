#include "obstacle_navigation.h"
#include <cassert>
#include <cstdio>

static SensorData distance(uint16_t mm, uint32_t now_us) {
  SensorData d;
  d.distance_mm = mm;
  sensor_policy::success(d, now_us);
  return d;
}
static RobotState encounter(uint32_t now_us) {
  RobotState s{};
  s.current_time_us = now_us;
  s.dist_front = distance(350, now_us);
  s.dist_left = distance(280, now_us);
  s.dist_right = distance(600, now_us);
  s.track_direction = TrackDirection::CLOCKWISE;
  return s;
}
static void tick(RobotState &s, uint32_t now_us, uint16_t front_mm) {
  s.current_time_us = now_us;
  s.dist_front = distance(front_mm, now_us);
}

static void bounded_avoidance_returns_to_wall_following() {
  ObstacleAvoidance avoid;
  auto s = encounter(1000);
  avoid.update(s, 1000, true, 1);
  assert(avoid.direction() > 0.0f && avoid.urgency() > 0.0f);
  s.dist_front = distance(150, 2000);
  s.dist_left = distance(700, 2000);
  s.dist_right = distance(250, 2000);
  avoid.update(s, 2000, true, 1);
  assert(avoid.direction() >
         0.0f); // Brief diagonal noise doesn't swap the pulse.
  assert(avoid.steer(-1.0f) == 1.0f);
  s.imu.measurement.yaw = config::avoidance_max_yaw_deg;
  avoid.update(s, 3000, true, 1);
  assert(avoid.steer(-0.2f) ==
         -0.2f); // Enough rotation: normal wall following.
  avoid.update(s, 4000, true, 1);
  assert(avoid.steer(-0.2f) == -0.2f); // A close front cannot renew the pulse.

  s.dist_front = distance(1000, 5000);
  s.dist_front.health = SensorHealth::OUT_OF_RANGE;
  avoid.update(s, 5000, true, 1);
  assert(avoid.steer(-0.5f) ==
         -0.5f); // No timed tail after the pole disappears.
  s = encounter(6000);
  avoid.update(s, 6000);
  tick(s, 186000, 300);
  s.dist_left = distance(280, s.current_time_us);
  s.dist_right = distance(600, s.current_time_us);
  avoid.update(s, 186000);
  assert(avoid.steer(-0.2f) == -0.2f); // No IMU: time bound still releases it.
  auto walls = avoid.guidance(s, s.current_time_us);
  assert(walls.steering >
         0.0f); // Shared front blend doesn't outlive the pulse.
  avoid.resume(187000);
  avoid.update(s, 187000);
  assert(avoid.steer(-0.2f) == -0.2f); // Recovery adds no second forced turn.

  avoid.reset();
  s = encounter(400000);
  s.track_direction = TrackDirection::COUNTER_CLOCKWISE;
  s.dist_left = distance(500, 400000);
  s.dist_right = distance(510, 400000);
  avoid.update(s, 400000);
  assert(avoid.direction() <
         0.0f); // Near tie follows track, not single-mm noise.
  avoid.reset();
  s = encounter(500000);
  s.dist_side_front_right = distance(80, 300000);
  avoid.update(s, 500000);
  assert(avoid.direction() > 0.0f); // Ignore stale side readings.
  avoid.reset();
  s.dist_front =
      distance(577, 500000); // CSV t=7003: old 650 mm onset was early.
  avoid.update(s, 500000);
  assert(avoid.urgency() == 0.0f && avoid.steer(0.0f) == 0.0f);
}

static void recorded_wall_conflict_and_reverse_direction() {
  ObstacleAvoidance avoid;
  auto s = encounter(15000000);
  s.dist_left.distance_mm = 700;
  s.dist_right.distance_mm = 300;
  avoid.update(s, s.current_time_us, true, 1); // Previously chose left.
  assert(avoid.direction() < 0.0f);
  // CSV at 16003 ms: front 239, left wall 31, right side no-return/open.
  tick(s, 16003000, 239);
  s.dist_left = distance(121, s.current_time_us);
  s.dist_right = distance(446, s.current_time_us);
  s.dist_side_front_left = distance(31, s.current_time_us);
  s.dist_side_rear_left = distance(336, s.current_time_us);
  s.dist_side_front_right = distance(1000, s.current_time_us);
  s.dist_side_front_right.health = SensorHealth::OUT_OF_RANGE;
  s.dist_side_rear_right = distance(1000, s.current_time_us);
  avoid.update(s, s.current_time_us, true, 1);
  assert(avoid.steer(-0.98445f) >
         0.8f); // Old output in the CSV pointed at wall.
  assert(avoid.steer(avoid.guidance(s, s.current_time_us).steering) > 0.0f);

  // CSV at 16533 ms: fresh space requires rightward nose rotation, not old
  // left.
  tick(s, 16533000, 219);
  s.dist_left = distance(94, s.current_time_us);
  s.dist_right = distance(373, s.current_time_us);
  s.dist_side_front_left = distance(40, s.current_time_us);
  s.dist_side_rear_left = distance(428, s.current_time_us);
  s.dist_side_front_right = distance(1000, s.current_time_us);
  s.dist_side_rear_right = distance(1000, s.current_time_us);
  ReverseRecovery reverse;
  s.current_throttle = -0.2f;
  reverse.begin(s, -1.0f, false,
                1); // Supply the stale leftward latch deliberately.
  assert(reverse.update(s, 220, false, 1) == RunningState::REVERSE);
  tick(s, 16733000, 219);
  assert(reverse.update(s, 220, false, 1) == RunningState::REVERSE_TURN);
  assert(reverse.steering() < 0.0f); // Reverse left rotates nose right, away.

  avoid.reset();
  s = encounter(17000000);
  s.dist_side_front_right = distance(30, s.current_time_us);
  avoid.update(s, s.current_time_us);
  assert(avoid.steer(1.0f) <
         -0.8f); // Mirror: right wall wins over rightward assist.
}

static void recorded_yaw_sign_and_real_wrong_way() {
  float maximum = 0.0f, minimum = 0.0f;
  // CSV 5433..11433 ms: normal left/CCW turns wrongly triggered recovery
  // before.
  for (float yaw : {-0.1f, -37.0f, -68.4f, -118.1f, -227.5f})
    assert(heading_deviation(TrackDirection::COUNTER_CLOCKWISE, yaw, maximum,
                             minimum) == 0.0f);
  assert(heading_deviation(TrackDirection::COUNTER_CLOCKWISE, 42.5f, maximum,
                           minimum) > config::wrong_way_yaw_threshold_deg);
  maximum = minimum = 0.0f;
  for (float yaw : {10.0f, 37.0f, 100.0f, 227.5f})
    assert(heading_deviation(TrackDirection::CLOCKWISE, yaw, maximum,
                             minimum) == 0.0f);
  assert(heading_deviation(TrackDirection::CLOCKWISE, -42.5f, maximum,
                           minimum) > config::wrong_way_yaw_threshold_deg);
}

static void short_backup_waits_for_command_and_distinct_clear_frames() {
  ReverseRecovery reverse;
  auto s = encounter(1000);
  s.dist_front.distance_mm = 80;
  s.current_throttle = 0.3f;
  reverse.begin(s, 1.0f, true, 1);
  tick(s, 201000, 80);
  assert(reverse.update(s, 110, true, 1) == RunningState::REVERSE);
  s.current_throttle = -0.05f;
  tick(s, 211000, 400);
  assert(reverse.update(s, 110, true, 1) == RunningState::REVERSE);
  s.current_time_us = 311000; // Same front frame must not count twice.
  assert(reverse.update(s, 110, true, 1) == RunningState::REVERSE);
  tick(s, 321000, 400);
  assert(reverse.update(s, 110, true, 1) == RunningState::REVERSE_TURN);
  assert(reverse.steering() <
         0.0f); // Backward turn preserves rightward nose yaw.
  s.imu.measurement.yaw = -config::reverse_max_yaw_deg;
  tick(s, 331000, 400);
  assert(reverse.update(s, 110, true, 1) == RunningState::RUNNING);
  assert(reverse.steering() == 0.0f);
}

static void bounded_without_imu_and_across_orientation_reset() {
  ReverseRecovery reverse;
  auto s = encounter(1000);
  s.current_throttle = -0.2f;
  s.dist_left.distance_mm = 600;
  s.dist_right.distance_mm = 280;
  reverse.begin(s, -1.0f, false, 1);
  assert(reverse.update(s, 110, false, 1) == RunningState::REVERSE);
  tick(s, 201000, 80); // No front clearance: still finish maximum backup time.
  assert(reverse.update(s, 110, false, 1) == RunningState::REVERSE_TURN);
  assert(reverse.steering() > 0.0f);
  tick(s, 451000, 80);
  assert(reverse.update(s, 110, false, 1) == RunningState::RUNNING);

  s = encounter(1000);
  s.imu.measurement.yaw = 170.0f;
  s.current_throttle = -0.2f;
  reverse.begin(s, 1.0f, true, 2);
  assert(reverse.update(s, 110, true, 2) == RunningState::REVERSE);
  tick(s, 11000, 80);
  s.imu.measurement.yaw = 0.0f; // Sensor reset, not a 170-degree rotation.
  assert(reverse.update(s, 110, true, 3) == RunningState::REVERSE);
  tick(s, 201000, 80);
  assert(reverse.update(s, 110, true, 3) == RunningState::REVERSE_TURN);

  s = encounter(1000);
  s.current_throttle = -0.2f;
  reverse.begin(s, 1.0f, true, 4);
  assert(reverse.update(s, 110, false, 4) == RunningState::REVERSE);
  tick(s, 11000, 80);
  s.imu.measurement.yaw = 90.0f;
  assert(reverse.update(s, 110, true, 4) == RunningState::REVERSE);

  // Zero reverse throttle or actuator ramp cannot leave recovery stuck forever.
  s = encounter(1000);
  s.current_throttle = 0.0f;
  reverse.begin(s, 1.0f, false, 1);
  tick(s, 901000, 80);
  assert(reverse.update(s, 110, false, 1) == RunningState::RUNNING);

  s = encounter(UINT32_MAX - 100000);
  s.current_throttle = -0.2f;
  reverse.begin(s, 1.0f, false, 1);
  assert(reverse.update(s, 110, false, 1) == RunningState::REVERSE);
  tick(s, 99999, 80); // 200 ms across micros wrap.
  assert(reverse.update(s, 110, false, 1) == RunningState::REVERSE_TURN);
}

int main() {
  bounded_avoidance_returns_to_wall_following();
  recorded_wall_conflict_and_reverse_direction();
  recorded_yaw_sign_and_real_wrong_way();
  short_backup_waits_for_command_and_distinct_clear_frames();
  bounded_without_imu_and_across_orientation_reset();
  puts("pole avoidance and adaptive reverse recovery tests passed");
}
