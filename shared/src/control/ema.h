#pragma once

/**
 * Exponential Moving Average (EMA) filter.
 * Output update: y[k] = y[k-1] + alpha * (x[k] - y[k-1])
 */
class EMAFilter {
public:
  /**
   * @param alpha Smoothing factor between 0.0f and 1.0f.
   *              Higher alpha = faster response / less smoothing.
   *              Lower alpha = slower response / more smoothing.
   * @param initial_value Starting output value if pre-initialized.
   */
  constexpr EMAFilter(float alpha = 0.25f, float initial_value = 0.0f)
      : m_alpha(alpha), m_value(initial_value), m_initialized(false) {}

  /**
   * Update the filter with a new sample.
   * Initializes automatically to the first sample value if uninitialized.
   * @param sample New input sample.
   * @return Filtered output value.
   */
  float update(float sample) {
    if (!m_initialized) {
      m_value = sample;
      m_initialized = true;
      return m_value;
    }
    m_value += m_alpha * (sample - m_value);
    return m_value;
  }

  /**
   * Reset filter value and initialization state.
   */
  void reset(float value = 0.0f, bool initialized = false) {
    m_value = value;
    m_initialized = initialized;
  }

  float get() const { return m_value; }

  void setAlpha(float alpha) { m_alpha = alpha; }
  float getAlpha() const { return m_alpha; }

  bool isInitialized() const { return m_initialized; }

private:
  float m_alpha;
  float m_value;
  bool m_initialized;
};
