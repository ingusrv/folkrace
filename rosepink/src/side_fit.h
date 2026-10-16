#pragma once
#include "config.h"
#include <SparkFun_VL53L5CX_Library.h>
#include <algorithm>
#include <cmath>

struct SideWallEstimate {
  float distance_mm = config::sensor_max_raw;
  float min_distance_mm = config::sensor_max_raw;
  float front_dist_mm = config::sensor_max_raw;
  float rear_dist_mm = config::sensor_max_raw;
  float heading_error_rad = 0.0f;
  float heading_error_deg = 0.0f;
  float confidence = 0.0f;
  bool valid = false;
};

// Existing mounting calibration: columns run rear to front after flip_x.
static constexpr float col_sin[4] = {-0.290285f, -0.098017f, 0.098017f,
                                     0.290285f};
static constexpr float col_cos[4] = {0.956940f, 0.995185f, 0.995185f,
                                     0.956940f};

/** Fit a wall from all 16 zones while retaining independent near clearances.
 * Column medians reject isolated floor/top-edge returns. Three-column line
 * consensus rejects a pole or an opening from the heading estimate. Sparse
 * frames still provide clearance, but cannot invent a wall heading.
 */
inline SideWallEstimate process_vl53l5cx(const VL53L5CX_ResultsData &data,
                                         bool flip_x, bool flip_y,
                                         bool is_left) {
  SideWallEstimate est;
  float x[4] = {}, y[4] = {};
  unsigned count = 0, zones = 0;
  float front = INFINITY, rear = INFINITY;
  for (unsigned c = 0; c < 4; ++c) {
    float ranges[4];
    unsigned n = 0;
    for (unsigned r = 0; r < 4; ++r) {
      unsigned index = (flip_y ? 3 - r : r) * 4 + (flip_x ? 3 - c : c);
      unsigned target = index * VL53L5CX_NB_TARGET_PER_ZONE;
      uint8_t status = data.target_status[target];
      float range = data.distance_mm[target];
      if ((status != 5 && status != 9) || range <= 0.0f)
        continue;
      ranges[n++] = range;
      ++zones;
      // All valid zones contribute to collision clearance, including points
      // rejected by the wall model. Do not smooth away a nearby competitor.
      float lateral = range * col_cos[c];
      float &sector = c < 2 ? rear : front;
      sector = std::min(sector, lateral);
    }
    if (!n)
      continue;
    std::sort(ranges, ranges + n);
    float median = (ranges[(n - 1) / 2] + ranges[n / 2]) * 0.5f;
    x[count] = median * col_sin[c];
    y[count++] = median * col_cos[c];
  }
  if (!count)
    return est;
  est.valid = true;
  est.min_distance_mm = std::min(front, rear);
  est.distance_mm = est.min_distance_mm;
  // A missing sector is open/unknown, not a copy of the opposite sector.
  est.front_dist_mm = std::isfinite(front) ? front : config::sensor_max_raw;
  est.rear_dist_mm = std::isfinite(rear) ? rear : config::sensor_max_raw;

  unsigned best_count = 0, best_mask = 0;
  float best_error = INFINITY;
  for (unsigned a = 0; a < count; ++a) {
    for (unsigned b = a + 1; b < count; ++b) {
      float dx = x[b] - x[a];
      if (std::fabs(dx) < 1.0f)
        continue;
      float slope = (y[b] - y[a]) / dx;
      float intercept = y[a] - slope * x[a];
      float normal = std::sqrt(1.0f + slope * slope);
      unsigned inliers = 0, mask = 0;
      float error = 0.0f;
      for (unsigned k = 0; k < count; ++k) {
        float residual = std::fabs(y[k] - slope * x[k] - intercept) / normal;
        if (residual <= config::side_fit_tolerance_mm) {
          ++inliers;
          mask |= 1U << k;
          error += residual;
        }
      }
      if (inliers > best_count ||
          (inliers == best_count && error < best_error)) {
        best_count = inliers;
        best_mask = mask;
        best_error = error;
      }
    }
  }
  if (best_count < 3)
    return est;
  float mx = 0.0f, my = 0.0f;
  for (unsigned k = 0; k < count; ++k) {
    if (best_mask & (1U << k)) {
      mx += x[k];
      my += y[k];
    }
  }
  mx /= best_count;
  my /= best_count;
  float numerator = 0.0f, denominator = 0.0f;
  for (unsigned k = 0; k < count; ++k) {
    if (best_mask & (1U << k)) {
      numerator += (x[k] - mx) * (y[k] - my);
      denominator += (x[k] - mx) * (x[k] - mx);
    }
  }
  if (denominator < 1.0f)
    return est;
  float slope = numerator / denominator;
  float intercept = my - slope * mx;
  if (intercept <= 0.0f)
    return est;
  est.heading_error_rad = (is_left ? -1.0f : 1.0f) * std::atan(slope);
  est.heading_error_deg = est.heading_error_rad * (180.0f / 3.14159265f);
  float normal = std::sqrt(1.0f + slope * slope);
  est.distance_mm = intercept / normal;
  float half = config::side_sensor_baseline_mm * 0.5f;
  // Conservative footprint clearances combine fitted wall and real sectors.
  est.front_dist_mm = std::min(
      est.front_dist_mm, std::max(0.0f, (intercept + slope * half) / normal));
  est.rear_dist_mm = std::min(
      est.rear_dist_mm, std::max(0.0f, (intercept - slope * half) / normal));
  est.confidence =
      (best_count / 4.0f) * (zones / 16.0f) *
      std::max(0.0f, 1.0f - best_error /
                                (best_count * config::side_fit_tolerance_mm));
  return est;
}
