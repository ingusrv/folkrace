#include "escape_recovery.h"
#include <cassert>
#include <cstdio>
#include <initializer_list>

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
  for (unsigned i = 1; i < config::escape_reverse_attempts; ++i) {
    assert(!escape.begin_attempt(s, 1.0f));
    s = trapped(s.current_time_us + 570000U);
  }
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
  for (unsigned i = 1; i < config::escape_reverse_attempts; ++i)
    assert(!escape.begin_attempt(s, 1.0f));
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

static void timeout_resets_attempts_and_wrap() {
  EscapeRecovery escape;
  auto s = trapped(1000);
  for (unsigned i = 1; i < config::escape_reverse_attempts; ++i) {
    assert(!escape.begin_attempt(s, 1.0f));
    // Elapsed time between ordinary reverses does not clear the count.
    s.current_time_us += 10000000U;
  }
  assert(escape.begin_attempt(s, 1.0f));
  s.current_throttle = 0.0f; // No reverse command: total timeout still exits.
  s.current_time_us += config::escape_timeout_ms * 1000U;
  escape.update(s);
  assert(!escape.active());
  for (unsigned i = 1; i < config::escape_reverse_attempts; ++i)
    assert(!escape.begin_attempt(s, 1.0f));
  assert(escape.begin_attempt(s, 1.0f));

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

static void clear_forward_resets_after_500ms() {
  for (uint32_t start_us : {1000U, UINT32_MAX - 200000U}) {
    EscapeRecovery escape;
    auto s = trapped(start_us);
    for (unsigned i = 1; i < config::escape_reverse_attempts; ++i)
      assert(!escape.begin_attempt(s, 1.0f));
    s.current_throttle = s.target_throttle = 0.3f;
    escape.observe(s, 220); // Close front does not start the timer.
    s.current_time_us += 500000U;
    escape.observe(s, 220);
    s.dist_front.distance_mm = 600;
    sensor_policy::success(s.dist_front, s.current_time_us);
    escape.observe(s, 220);
    s.current_time_us += 499000U;
    sensor_policy::success(s.dist_front, s.current_time_us);
    escape.observe(s, 220);
    assert(escape.begin_attempt(s, 1.0f)); // 499 ms retains the count.

    s.current_time_us += config::escape_timeout_ms * 1000U;
    escape.update(s);
    for (unsigned i = 1; i < config::escape_reverse_attempts; ++i)
      assert(!escape.begin_attempt(s, 1.0f));
    s.running_state = RunningState::RUNNING;
    s.current_throttle = s.target_throttle = 0.3f;
    sensor_policy::success(s.dist_front, s.current_time_us);
    escape.observe(s, 220);
    s.current_time_us += 499000U;
    sensor_policy::success(s.dist_front, s.current_time_us);
    escape.observe(s, 220);
    s.current_throttle = -0.2f;
    escape.observe(s, 220); // Interrupted forward drive restarts the timer.
    s.current_throttle = 0.3f;
    escape.observe(s, 220);
    s.current_time_us += 500000U;
    sensor_policy::success(s.dist_front, s.current_time_us);
    escape.observe(s, 220);
    for (unsigned i = 1; i < config::escape_reverse_attempts; ++i)
      assert(!escape.begin_attempt(s, 1.0f));
    assert(escape.begin_attempt(s, 1.0f));
  }
}

int main() {
  repeated_attempts_rock_and_alternate();
  timeout_resets_attempts_and_wrap();
  clear_forward_resets_after_500ms();
  puts("repeated reverse escape tests passed");
}
