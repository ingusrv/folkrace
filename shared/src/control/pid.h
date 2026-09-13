#pragma once
#include "../state.h"
#include <algorithm>

/**
 * Reset PID state accumulated terms.
 */
inline void reset_pid(PIDState &pid) {
  pid.error = 0.0f;
  pid.last_error = 0.0f;
  pid.integral = 0.0f;
  pid.derivative = 0.0f;
}

/**
 * Update PID controller.
 * @param dt Delta time in seconds since last update.
 * @param pid PIDState reference containing gains and state history.
 * @return Clamped control output in [-1.0, 1.0].
 */
inline float update_pid(float dt, PIDState &pid) {
  float error = pid.error;

  float derivative = 0.0f;
  if (dt > 0.0f) {
    pid.integral += error * dt;
    // Anti-windup clamping on integral
    pid.integral = std::max(-1.0f, std::min(1.0f, pid.integral));
    derivative = (error - pid.last_error) / dt;
    pid.derivative = derivative;
    pid.last_error = error;
  }

  float output =
      (pid.Kp * error) + (pid.Ki * pid.integral) + (pid.Kd * derivative);
  return std::max(-1.0f, std::min(1.0f, output));
}
