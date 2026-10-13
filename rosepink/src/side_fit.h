#pragma once
#include "config.h"
#include <SparkFun_VL53L5CX_Library.h>
#include <algorithm>
#include <cmath>
struct SideWallEstimate {
  float distance_mm = 0.0f;
  float min_distance_mm = 0.0f;
  float front_dist_mm = 0.0f;
  float rear_dist_mm = 0.0f;
  float heading_error_rad = 0.0f;
  float heading_error_deg = 0.0f;
  bool valid = false;
};
static constexpr float col_sin[4] = {-0.290285f, -0.098017f, 0.098017f,
                                     0.290285f};
static constexpr float col_cos[4] = {0.956940f, 0.995185f, 0.995185f,
                                     0.956940f};

static SideWallEstimate process_vl53l5cx(const VL53L5CX_ResultsData &data,
                                         bool flip_x, bool flip_y,
                                         bool is_left) {
  SideWallEstimate est = {};
  float x_pts[4];
  float y_pts[4];
  int valid_count = 0;
  float min_observed_dist = 10000.0f;

  for (int c = 0; c < 4; c++) {
    int eff_c = flip_x ? (3 - c) : c;
    float sum_d = 0.0f;
    int row_valid = 0;

    for (int r = 0; r < 4; r++) {
      int eff_r = flip_y ? (3 - r) : r;
      int raw_idx = eff_r * 4 + eff_c;
      uint8_t status = data.target_status[raw_idx];
      int16_t d = data.distance_mm[raw_idx];

      // Accept valid targets; wrap-around status 6 is ambiguous.
      if ((status == 5 || status == 9) && d > 0) {
        sum_d += static_cast<float>(d);
        row_valid++;
        if (static_cast<float>(d) < min_observed_dist) {
          min_observed_dist = static_cast<float>(d);
        }
      }
    }

    if (row_valid > 0) {
      float r_avg = sum_d / static_cast<float>(row_valid);
      x_pts[valid_count] = r_avg * col_sin[c];
      y_pts[valid_count] = r_avg * col_cos[c];
      valid_count++;
    }
  }

  if (valid_count >= 2) {
    float sum_x = 0.0f, sum_y = 0.0f;
    for (int i = 0; i < valid_count; i++) {
      sum_x += x_pts[i];
      sum_y += y_pts[i];
    }
    float mean_x = sum_x / static_cast<float>(valid_count);
    float mean_y = sum_y / static_cast<float>(valid_count);

    float num = 0.0f, den = 0.0f;
    for (int i = 0; i < valid_count; i++) {
      float dx = x_pts[i] - mean_x;
      num += dx * (y_pts[i] - mean_y);
      den += dx * dx;
    }

    float m = (den > 1e-4f) ? (num / den) : 0.0f;
    float y0 = mean_y - m * mean_x;

    // Heading error: positive = vehicle turned into wall (demands opposite
    // steer) Left side: turned into left wall -> m < 0 -> theta = -atan(m) > 0
    // (steer right) Right side: turned into right wall -> m < 0 -> theta =
    // +atan(m) < 0 (steer left)
    float theta_rad = is_left ? -std::atan(m) : std::atan(m);
    float dist = std::max(0.0f, y0 * std::cos(theta_rad));

    est.heading_error_rad = theta_rad;
    est.heading_error_deg = theta_rad * (180.0f / 3.14159265f);
    est.distance_mm = dist;
    est.min_distance_mm =
        (min_observed_dist < 10000.0f) ? min_observed_dist : dist;
    est.valid = std::isfinite(dist);

    float half_baseline = config::side_sensor_baseline_mm * 0.5f;
    float cos_m = std::cos(std::atan(m));
    est.front_dist_mm = std::max(0.0f, (y0 + m * half_baseline) * cos_m);
    est.rear_dist_mm = std::max(0.0f, (y0 - m * half_baseline) * cos_m);
  } else if (valid_count == 1) {
    float dist = y_pts[0];
    est.heading_error_rad = 0.0f;
    est.heading_error_deg = 0.0f;
    est.distance_mm = dist;
    est.min_distance_mm =
        (min_observed_dist < 10000.0f) ? min_observed_dist : dist;
    est.front_dist_mm = dist;
    est.rear_dist_mm = dist;
    est.valid = std::isfinite(dist);
  } else {
    est.distance_mm = static_cast<float>(config::sensor_max_raw);
    est.min_distance_mm = static_cast<float>(config::sensor_max_raw);
    est.front_dist_mm = static_cast<float>(config::sensor_max_raw);
    est.rear_dist_mm = static_cast<float>(config::sensor_max_raw);
    est.heading_error_rad = 0.0f;
    est.heading_error_deg = 0.0f;
    est.valid = false;
  }

  return est;
}
