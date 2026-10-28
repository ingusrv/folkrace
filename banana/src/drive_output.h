#pragma once
#include "control/slew_rate_limiter.h"
#include <algorithm>
#include <cmath>

struct WheelCommands {
  float left, right;
  bool stopped() const { return left == 0.0f && right == 0.0f; }
};

// Limit physical wheel commands after mixing and deadband. A zero throttle
// alone does not stop a pivot; inhibition must drive both final targets to
// zero.
class DriveOutput {
public:
  DriveOutput(float acceleration, float deceleration, float minimum)
      : m_left(acceleration, deceleration), m_right(acceleration, deceleration),
        m_minimum(minimum) {}
  WheelCommands update(float throttle, float steering, bool allow_reverse,
                       bool inhibited, float dt) {
    float low = allow_reverse ? -1.0f : 0.0f;
    float left_mix = throttle + steering;
    float right_mix = throttle - steering;
    if (!allow_reverse) {
      // Forward driving: steer only by slowing the inner wheel so turns never
      // speed the robot up. Wheel speed difference (2 * steering) is unchanged.
      float shift = std::fabs(steering);
      left_mix -= shift;
      right_mix -= shift;
    }
    float left = deadband(std::max(low, std::min(1.0f, left_mix)));
    float right = deadband(std::max(low, std::min(1.0f, right_mix)));
    if (inhibited)
      left = right = 0.0f;
    return {m_left.update(left, dt), m_right.update(right, dt)};
  }
  void reset() {
    m_left.reset();
    m_right.reset();
  }

private:
  float deadband(float value) const {
    if (std::fabs(value) < 0.02f)
      return 0.0f;
    return std::copysign(m_minimum + (1.0f - m_minimum) * std::fabs(value),
                         value);
  }
  SlewRateLimiter m_left, m_right;
  float m_minimum;
};
