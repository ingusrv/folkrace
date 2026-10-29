#pragma once
#include "config.h"
#include <SparkFun_VL53L5CX_Library.h>
#include <algorithm>
#include <cmath>

struct SideWallEstimate {
  float distance_mm = config::sensor_max_raw;
  float min_distance_mm = config::sensor_max_raw;
  float heading_error_rad = 0.0f;
  float heading_error_deg = 0.0f;
  float confidence = 0.0f;
  bool valid = false;
};

struct ForwardFrame {
  SideWallEstimate wall;
  uint16_t distance_mm = config::sensor_max_raw;
  bool valid = false;
  bool open = true;
  uint16_t valid_zones = 0;
  uint16_t floor_zones = 0;
  uint16_t clearance_zones = 0;
  uint8_t nearest_zone = 255; // Logical top-left-first zone, 255 if open.
};

/** Project calibrated zone rays into robot coordinates: x forward, y right.
 * Clearance uses every valid above-floor point; heading needs three columns
 * agreeing on a wall. No rear-side readings are synthesized.
 */
inline ForwardFrame process_forward_vl53l5cx(const VL53L5CX_ResultsData &data,
                                             unsigned sensor, float pitch_deg) {
  constexpr float radians = 3.14159265f / 180.0f;
  ForwardFrame frame;
  auto &est = frame.wall;
  float x[4] = {}, y[4] = {};
  unsigned count = 0, zones = 0;
  float yaw = config::tof_yaw_deg[sensor] * radians;
  float pitch =
      (config::tof_pitch_deg[sensor] + config::tof_imu_pitch_sign * pitch_deg) *
      radians;
  float nearest = config::sensor_max_raw;
  float min_lateral = config::sensor_max_raw;
  bool evidence = false;
  for (unsigned c = 0; c < 4; ++c) {
    float column_x[4], column_y[4];
    unsigned n = 0;
    float h =
        std::atan(((c + 0.5f) * 0.5f - 1.0f) *
                  std::tan(config::tof_horizontal_fov_deg * radians * 0.5f));
    for (unsigned r = 0; r < 4; ++r) {
      unsigned logical_zone = r * 4 + c;
      if (config::tof_ignore_zones[sensor] & (1U << logical_zone))
        continue;
      unsigned index = (config::tof_flip_y[sensor] ? 3 - r : r) * 4 +
                       (config::tof_flip_x[sensor] ? 3 - c : c);
      unsigned target = index * VL53L5CX_NB_TARGET_PER_ZONE;
      uint8_t status = data.target_status[target];
      if (data.nb_target_detected[index] == 0 && status == 255) {
        evidence = true; // A completed no-target zone, not a transport failure.
        continue;
      }
      float range = data.distance_mm[target];
      if (!data.nb_target_detected[index] || (status != 5 && status != 9) ||
          range <= 0.0f)
        continue;
      evidence = true;
      frame.valid_zones |= 1U << logical_zone;
      float v =
          std::atan((1.0f - (r + 0.5f) * 0.5f) *
                    std::tan(config::tof_vertical_fov_deg * radians * 0.5f));
      float forward = range * std::cos(v) * std::cos(h);
      float right = range * std::cos(v) * std::sin(h);
      float up = range * std::sin(v);
      float leveled_forward = forward * std::cos(pitch) - up * std::sin(pitch);
      float height = config::tof_height_mm[sensor] + forward * std::sin(pitch) +
                     up * std::cos(pitch);
      // A 4x4 zone covers an angular area, not just its center ray. Floor
      // returns within its lower edge can otherwise project above the floor
      // (e.g. the recorded 417-460 mm returns in the lower-middle row).
      // Preserve close objects whose whole vertical footprint is above floor.
      float lower_v =
          std::atan((1.0f - (r + 1.0f) * 0.5f) *
                    std::tan(config::tof_vertical_fov_deg * radians * 0.5f));
      float lower_height =
          config::tof_height_mm[sensor] +
          range * (std::cos(lower_v) * std::cos(h) * std::sin(pitch) +
                   std::sin(lower_v) * std::cos(pitch));
      if (std::min(height, lower_height) < config::tof_floor_reject_height_mm) {
        frame.floor_zones |= 1U << logical_zone;
        continue;
      }
      float px = config::tof_forward_mm[sensor] +
                 leveled_forward * std::cos(yaw) - right * std::sin(yaw);
      float py = config::tof_right_mm[sensor] +
                 leveled_forward * std::sin(yaw) + right * std::cos(yaw);
      if (px <= 0.0f)
        continue;
      if (sensor == 0) {
        if (std::fabs(py) <= config::tof_corridor_half_width_mm) {
          frame.clearance_zones |= 1U << logical_zone;
          if (px < nearest) {
            nearest = px;
            frame.nearest_zone = logical_zone;
          }
        }
      } else {
        // Keep diagonal ranges compatible with avoidance/stall thresholds.
        frame.clearance_zones |= 1U << logical_zone;
        if (range < nearest) {
          nearest = range;
          frame.nearest_zone = logical_zone;
        }
        float lateral = sensor == 1 ? -py : py;
        if (lateral <= 0.0f)
          continue;
        min_lateral = std::min(min_lateral, lateral);
        column_x[n] = px;
        column_y[n++] = lateral;
        ++zones;
      }
    }
    if (!n)
      continue;
    // Select the median lateral point, preserving its matching forward value.
    for (unsigned a = 1; a < n; ++a)
      for (unsigned b = a; b > 0 && column_y[b] < column_y[b - 1]; --b) {
        std::swap(column_y[b], column_y[b - 1]);
        std::swap(column_x[b], column_x[b - 1]);
      }
    x[count] = column_x[n / 2];
    y[count++] = column_y[n / 2];
  }
  frame.valid = evidence;
  frame.open = nearest >= config::sensor_max_raw;
  frame.distance_mm = static_cast<uint16_t>(std::max(0.0f, nearest));
  if (!count)
    return frame;
  est.valid = true;
  est.min_distance_mm = min_lateral;
  est.distance_mm = min_lateral;

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
    return frame;
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
    return frame;
  float slope = numerator / denominator;
  float intercept = my - slope * mx;
  if (intercept <= 0.0f)
    return frame;
  est.heading_error_rad = (sensor == 1 ? -1.0f : 1.0f) * std::atan(slope);
  est.heading_error_deg = est.heading_error_rad * (180.0f / 3.14159265f);
  float normal = std::sqrt(1.0f + slope * slope);
  est.distance_mm = intercept / normal;
  est.confidence =
      (best_count / 4.0f) * (zones / 16.0f) *
      std::max(0.0f, 1.0f - best_error /
                                (best_count * config::side_fit_tolerance_mm));
  return frame;
}
