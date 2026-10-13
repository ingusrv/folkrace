#pragma once
#include "control/pid.h"

/** Cancel timed driving on sensor loss, preserving the actual actuator output.
 * @return true when an active state must skip its maneuver/PID update this
 * tick. IDLE and COUNTDOWN retain their own entry actions and acknowledgement
 * gates. */
inline bool sensor_motion_inhibit(RobotState &state, bool inhibited) {
  if (!inhibited || state.running_state == RunningState::IDLE ||
      state.running_state == RunningState::COUNTDOWN)
    return false;
  state.running_state = RunningState::RUNNING;
  state.maneuver_start_us = 0;
  state.target_throttle = 0.0f;
  state.target_steering = state.current_steering;
  reset_pid(state.steering_pid);
  return true;
}
