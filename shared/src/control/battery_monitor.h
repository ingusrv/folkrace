#pragma once
#include <Arduino.h>

/** Calibrated divider measurement with a debounced, boot-latched cutoff. */
class BatteryMonitor {
public:
  /** Configure the ADC and pack threshold.
   * @param pin ADC input connected to the divider midpoint.
   * @param cutoff_mv Minimum pack voltage in mV.
   * @param divider_ratio Pack voltage divided by ADC voltage.
   * @param sample_interval_ms Time between ADC conversions.
   * @param confirm_ms Continuous low-voltage time before latching.
   */
  void begin(uint8_t pin, uint32_t cutoff_mv, float divider_ratio,
             uint32_t sample_interval_ms, uint32_t confirm_ms) {
    m_pin = pin;
    m_cutoff_mv = cutoff_mv;
    m_divider_ratio = divider_ratio;
    m_sample_interval_ms = sample_interval_ms;
    m_confirm_ms = confirm_ms;
    pinMode(m_pin, INPUT);
    analogSetPinAttenuation(m_pin, ADC_11db);
    // Create the ADC/calibration handles during setup, outside the control
    // loop.
    m_voltage_mv = analogReadMilliVolts(m_pin) * m_divider_ratio;
  }

  /** Sample when due; return true only when the cutoff first latches.
   * @param now_ms Current wrap-safe millis() timestamp.
   * @return Whether this call newly latched the cutoff.
   */
  bool update(uint32_t now_ms) {
    if (m_sampled && now_ms - m_last_sample_ms < m_sample_interval_ms)
      return false;
    m_sampled = true;
    m_last_sample_ms = now_ms;
    m_voltage_mv = analogReadMilliVolts(m_pin) * m_divider_ratio;
    if (m_cutoff)
      return false;
    if (m_voltage_mv > m_cutoff_mv) {
      m_low = false;
      return false;
    }
    if (!m_low) {
      m_low = true;
      m_low_since_ms = now_ms;
    }
    if (now_ms - m_low_since_ms < m_confirm_ms)
      return false;
    m_cutoff = true;
    return true;
  }

  /** @return Whether driving is inhibited until reboot. */
  bool cutoff() const { return m_cutoff; }
  /** @return Latest measured pack voltage in mV. */
  float voltage_mv() const { return m_voltage_mv; }

private:
  uint8_t m_pin = 0;
  uint32_t m_cutoff_mv = 0, m_sample_interval_ms = 50, m_confirm_ms = 500;
  uint32_t m_last_sample_ms = 0, m_low_since_ms = 0;
  float m_divider_ratio = 1.0f, m_voltage_mv = 0.0f;
  bool m_sampled = false, m_low = false, m_cutoff = false;
};
