#include "escape_recovery.h"
#include <cassert>
#include <cstdio>

static RobotState trapped(uint32_t now_us) {
  RobotState s{};
  s.current_time_us = now_us;
  s.running_state = RunningState::RUNNING;
  s.base_throttle = 0.35f;
  s.reverse_throttle = 0.2f;
  s.dist_front.distance_mm = 216;
  s.dist_left.distance_mm = 280;
  s.dist_right.distance_mm = 277;
  s.dist_side_front_left.distance_mm = 89;
  s.dist_side_front_right.distance_mm = 580;
  for (unsigned i = 0; i < 7; ++i)
    sensor_policy::success(*sensor_policy::distances(s, i), now_us);
  return s;
}

static void repeated_attempts_rock_and_alternate() {
  EscapeRecovery escape;
  auto s = trapped(817757000);
  assert(!escape.begin_attempt(s, 1.0f));
  s.current_time_us = 818327000;
  assert(!escape.begin_attempt(s, 1.0f));
  s.current_time_us = 818887000;
  assert(!escape.begin_attempt(s, 1.0f));
  s.current_time_us = 819457000;
  assert(!escape.begin_attempt(s, 1.0f));
  s = trapped(820027000); // Fifth reverse in the recorded repeating sequence.
  assert(escape.begin_attempt(s, 1.0f));
  s.current_throttle = 0.1f;
  escape.update(s);
  assert(s.target_throttle == -0.2f && s.target_steering == 0.0f);
  s.current_time_us += 200000;
  s.current_throttle = -0.2f;
  escape.update(s); // Start backup timer after reverse command is established.
  s.current_time_us += 699000;
  escape.update(s);
  assert(s.running_state == RunningState::REVERSE);
  s.current_time_us += 1000;
  escape.update(s);
  assert(s.running_state == RunningState::REVERSE_TURN);
  assert(s.target_steering ==
         -1.0f); // Reverse left rotates away from left wall.
  s.current_time_us += 10000;
  escape.update(s);
  s.current_time_us += 650000;
  escape.update(s);
  assert(s.running_state == RunningState::RUNNING && escape.active());
  assert(s.target_throttle == s.base_throttle);
  assert(s.target_steering == 0.0f); // Center while still commanding reverse.
  s.current_time_us += 300000;
  escape.update(s);
  assert(s.target_steering == 0.0f);
  s.current_throttle = 0.1f;
  s.current_time_us += 10000;
  escape.update(s);
  assert(s.target_steering == 1.0f);
  // The close front reading deliberately does not abort the forward rock.
  s.current_time_us += 599000;
  escape.update(s);
  assert(escape.active());
  s.current_time_us += 1000;
  escape.update(s);
  assert(!escape.active() && s.target_throttle == 0.0f);
  assert(escape.begin_attempt(s, 1.0f));
  s.current_throttle = -0.2f;
  escape.update(s);
  s.current_time_us += 700000;
  escape.update(s);
  assert(s.target_steering == 1.0f); // Same sensor picture, opposite attempt.
  escape.reset(); // Stop/inhibit cancels the maneuver and its history.
  assert(!escape.active());
  assert(!escape.begin_attempt(s, 1.0f));
}

static void clear_drive_timeout_and_wrap() {
  EscapeRecovery escape;
  auto s = trapped(1000);
  for (unsigned i = 1; i < config::escape_reverse_attempts; ++i)
    assert(!escape.begin_attempt(s, 1.0f));
  s.current_throttle = s.target_throttle = 0.3f;
  escape.observe(s, 220); // Close front cannot clear history.
  s.current_time_us += 4000000;
  escape.observe(s, 220);
  assert(escape.begin_attempt(s, 1.0f));
  s.current_throttle = 0.0f; // No reverse command: total timeout still exits.
  s.current_time_us += config::escape_timeout_ms * 1000U;
  escape.update(s);
  assert(!escape.active());

  s = trapped(10000000);
  s.dist_front.distance_mm = 600;
  s.current_throttle = s.target_throttle = 0.3f;
  escape.observe(s, 220);
  s.current_time_us += 2000000;
  sensor_policy::success(s.dist_front, s.current_time_us);
  escape.observe(s, 220);
  s.current_throttle = -0.2f;
  escape.observe(s, 220); // Interrupted drive must restart the success timer.
  s.current_throttle = 0.3f;
  escape.observe(s, 220);
  s.current_time_us += 2990000;
  sensor_policy::success(s.dist_front, s.current_time_us);
  escape.observe(s, 220);
  assert(escape.begin_attempt(s, 1.0f)); // Still remembers failed attempts.
  s.current_time_us += config::escape_timeout_ms * 1000U;
  escape.update(s);
  s.current_throttle = s.target_throttle = 0.3f;
  sensor_policy::success(s.dist_front, s.current_time_us);
  escape.observe(s, 220);
  s.current_time_us += 3000000;
  sensor_policy::success(s.dist_front, s.current_time_us);
  escape.observe(s, 220);
  assert(
      !escape.begin_attempt(s, 1.0f)); // Sustained clear drive clears history.

  escape.reset();
  s = trapped(UINT32_MAX - 1000000);
  for (unsigned i = 1; i < config::escape_reverse_attempts; ++i)
    assert(!escape.begin_attempt(s, 1.0f));
  assert(escape.begin_attempt(s, 1.0f));
  s.current_throttle = -0.2f;
  escape.update(s);
  s.current_time_us += 700000U;
  escape.update(s);
  assert(s.running_state == RunningState::REVERSE_TURN);
  s.current_time_us += 10000U;
  escape.update(s);
  s.current_time_us += 650000U;
  escape.update(s);
  assert(s.running_state == RunningState::RUNNING && escape.active());
}

int main() {
  repeated_attempts_rock_and_alternate();
  clear_drive_timeout_and_wrap();
  puts("repeated reverse escape tests passed");
}
