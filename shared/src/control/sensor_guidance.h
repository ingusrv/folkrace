#pragma once
#include "sensors/health.h"
#include <algorithm>

struct SensorGuidance {
  float steering, urgency;
};
/** Choose an avoidance direction using only fresh diagonals, then track
 * direction. */
inline float sensor_turn_direction(const RobotState &s, uint32_t now,
                                   float turn_range) {
  bool left = sensor_policy::available(s.dist_left, now);
  bool right = sensor_policy::available(s.dist_right, now);
  float direction =
      s.track_direction == TrackDirection::CLOCKWISE ? 1.0f : -1.0f;
  if (left && right)
    direction =
        s.dist_right.distance_mm >= s.dist_left.distance_mm ? 1.0f : -1.0f;
  else if (left && s.dist_left.distance_mm < turn_range)
    direction = 1.0f;
  else if (right && s.dist_right.distance_mm < turn_range)
    direction = -1.0f;
  return direction;
}
/** Compute wall guidance using only available distances.
 * @param now Freshness time captured after snapshot copies.
 * @return Normalized steering and front urgency; caller gates required sensors.
 */
inline SensorGuidance sensor_guidance(const RobotState &s, uint32_t now,
                                      float turn_range, uint16_t danger,
                                      uint16_t front_turn,
                                      uint16_t front_critical,
                                      uint16_t maximum) {
  auto clamp = [](float v, float lo, float hi) {
    return std::max(lo, std::min(hi, v));
  };
  auto available = [now](const SensorData &d) {
    return sensor_policy::available(d, now);
  };
  auto repulsion = [turn_range](uint16_t d) {
    return std::max(0.0f, (turn_range - d) / turn_range);
  };
  bool left = available(s.dist_left), right = available(s.dist_right);
  float diag =
      left && right
          ? clamp((float(s.dist_right.distance_mm) - s.dist_left.distance_mm) /
                      turn_range,
                  -1.0f, 1.0f)
      : left  ? repulsion(s.dist_left.distance_mm)
      : right ? -repulsion(s.dist_right.distance_mm)
              : 0.0f;
  bool fl = available(s.dist_side_front_left),
       rl = available(s.dist_side_rear_left);
  bool fr = available(s.dist_side_front_right),
       rr = available(s.dist_side_rear_right);
  uint16_t dl = std::min(fl ? s.dist_side_front_left.distance_mm : maximum,
                         rl ? s.dist_side_rear_left.distance_mm : maximum);
  uint16_t dr = std::min(fr ? s.dist_side_front_right.distance_mm : maximum,
                         rr ? s.dist_side_rear_right.distance_mm : maximum);
  // Available maximum-range samples mean no visible side wall.
  bool sl = (fl || rl) && dl < maximum, sr = (fr || rr) && dr < maximum;
  float side = sl && sr ? clamp((float(dr) - dl) / turn_range, -1.0f, 1.0f)
               : sl     ? repulsion(dl)
               : sr     ? -repulsion(dr)
                        : 0.0f;
  float angle = 0.0f;
  if (fl && rl && s.dist_side_front_left.distance_mm < maximum &&
      s.dist_side_rear_left.distance_mm < maximum)
    angle += clamp((float(s.dist_side_rear_left.distance_mm) -
                    s.dist_side_front_left.distance_mm) /
                       150.0f,
                   -0.5f, 0.5f);
  if (fr && rr && s.dist_side_front_right.distance_mm < maximum &&
      s.dist_side_rear_right.distance_mm < maximum)
    angle += clamp((float(s.dist_side_front_right.distance_mm) -
                    s.dist_side_rear_right.distance_mm) /
                       150.0f,
                   -0.5f, 0.5f);
  float guidance = sl || sr ? 0.5f * diag + 0.5f * (side + angle) : diag;
  uint16_t ml = std::min(dl, left ? s.dist_left.distance_mm : maximum);
  uint16_t mr = std::min(dr, right ? s.dist_right.distance_mm : maximum);
  float repel = (ml < danger ? float(danger - ml) / danger : 0.0f) -
                (mr < danger ? float(danger - mr) / danger : 0.0f);
  float direction = sensor_turn_direction(s, now, turn_range);
  float urgency = available(s.dist_front)
                      ? clamp((float(front_turn) - s.dist_front.distance_mm) /
                                  (front_turn - front_critical),
                              0.0f, 1.0f)
                      : 0.0f;
  return {clamp((1.0f - urgency) * clamp(guidance + repel, -1.0f, 1.0f) +
                    urgency * direction,
                -1.0f, 1.0f),
          urgency};
}
