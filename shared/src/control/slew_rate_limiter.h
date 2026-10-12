#pragma once
#include <algorithm>

class SlewRateLimiter {
public:
  /**
   * @param max_accel Rate limit when increasing magnitude / accelerating (units per second)
   *                  e.g., 2.0f means 0.0 to 1.0 takes 0.5s.
   * @param max_decel Rate limit when decreasing magnitude / braking (units per second)
   *                  e.g., 6.0f means 1.0 to 0.0 takes ~0.17s.
   * @param initial_value Starting output value (defaults to 0.0f).
   */
  constexpr SlewRateLimiter(float max_accel, float max_decel, float initial_value = 0.0f)
      : m_max_accel(max_accel), m_max_decel(max_decel), m_current(initial_value) {}

  /**
   * Update the limiter towards target.
   * @param target Desired target value.
   * @param dt_seconds Elapsed time since last update in seconds (e.g. 0.01f for 100 Hz).
   * @return Rate-limited current value.
   */
  float update(float target, float dt_seconds) {
    if (dt_seconds <= 0.0f) {
      return m_current;
    }

    float rate_limit;
    if (target > m_current) {
      // Increasing value: if current >= 0, this is forward acceleration;
      // if current < 0, this is recovery from reverse (braking/decelerating).
      rate_limit = (m_current >= 0.0f) ? m_max_accel : m_max_decel;
      float max_step = rate_limit * dt_seconds;
      m_current = std::min(m_current + max_step, target);
    } else {
      // Decreasing value: if current > 0, this is braking/decelerating;
      // if current <= 0, this is accelerating into reverse.
      rate_limit = (m_current > 0.0f) ? m_max_decel : m_max_accel;
      float max_step = rate_limit * dt_seconds;
      m_current = std::max(m_current - max_step, target);
    }

    return m_current;
  }

  void reset(float value = 0.0f) { m_current = value; }
  float get() const { return m_current; }

  void set_rates(float max_accel, float max_decel) {
    m_max_accel = max_accel;
    m_max_decel = max_decel;
  }

private:
  float m_max_accel;
  float m_max_decel;
  float m_current;
};
