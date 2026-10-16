#pragma once
#include "control/sensor_guidance.h"
#include "sensors.h"
#include <algorithm>

/** Blend fresh multizone wall geometry with the front diagonal beams.
 * Heading uses consensus confidence and favors the nearer wall. Clearance
 * remains independent of confidence so a pole cannot disappear in the fit.
 */
inline SensorGuidance multizone_guidance(const RobotState &s,
                                         const RosepinkSideSnapshot &side,
                                         uint32_t now_us) {
  auto clamp = [](float value) {
    return std::max(-1.0f, std::min(1.0f, value));
  };
  RobotState diagonal = s;
  diagonal.dist_front.health = SensorHealth::ABSENT;
  diagonal.dist_side_front_left.health = SensorHealth::ABSENT;
  diagonal.dist_side_rear_left.health = SensorHealth::ABSENT;
  diagonal.dist_side_front_right.health = SensorHealth::ABSENT;
  diagonal.dist_side_rear_right.health = SensorHealth::ABSENT;
  float diag =
      sensor_guidance(diagonal, now_us, config::side_turn_range,
                      config::side_danger_dist, config::front_turn_dist,
                      config::front_critical_dist, config::sensor_max_raw)
          .steering;

  auto fresh = [now_us](const SideWallEstimate &wall, const SensorData &front,
                        const SensorData &rear) {
    return wall.valid && sensor_policy::available(front, now_us) &&
           sensor_policy::available(rear, now_us);
  };
  bool left =
      fresh(side.left_wall, s.dist_side_front_left, s.dist_side_rear_left);
  bool right =
      fresh(side.right_wall, s.dist_side_front_right, s.dist_side_rear_right);
  float dl = left ? side.left_wall.distance_mm : config::sensor_max_raw;
  float dr = right ? side.right_wall.distance_mm : config::sensor_max_raw;
  left = left && dl < config::sensor_max_raw;
  right = right && dr < config::sensor_max_raw;
  auto pressure = [](float distance, float range) {
    return std::max(0.0f, (range - distance) / range);
  };
  float center = left && right ? clamp((dr - dl) / config::side_turn_range)
                 : left        ? pressure(dl, config::side_turn_range)
                 : right       ? -pressure(dr, config::side_turn_range)
                               : 0.0f;
  float heading = 0.0f, weight = 0.0f;
  auto add_heading = [&](const SideWallEstimate &wall, bool available) {
    if (!available)
      return;
    float w =
        wall.confidence * pressure(wall.distance_mm, config::sensor_max_raw);
    heading +=
        w * clamp(wall.heading_error_rad / config::side_heading_full_scale_rad);
    weight += w;
  };
  add_heading(side.left_wall, left);
  add_heading(side.right_wall, right);
  // Only normalize when both walls together exceed full confidence; weak
  // fits keep a proportionally weak influence instead of regaining full gain.
  heading /= std::max(1.0f, weight);
  float repel =
      (left ? pressure(side.left_wall.min_distance_mm, config::side_danger_dist)
            : 0.0f) -
      (right
           ? pressure(side.right_wall.min_distance_mm, config::side_danger_dist)
           : 0.0f);
  float steering = left || right
                       ? 0.5f * diag + 0.5f * center +
                             config::side_heading_weight * heading + repel
                       : diag;
  return {clamp(steering), 0.0f};
}
