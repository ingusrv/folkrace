#include "stanley.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>

constexpr float PI_F = 3.14159265358979323846f;
constexpr float DEG_TO_RAD_F = PI_F / 180.0f;
constexpr float RAD_TO_DEG_F = 180.0f / PI_F;

StanleyController::StanleyController() : m_config(StanleyConfig()) {}

StanleyController::StanleyController(const StanleyConfig &config)
    : m_config(config) {}

void StanleyController::reset() {
  m_filtered_heading_rad = 0.0f;
  m_heading_initialized = false;
}

StanleyOutput StanleyController::update(
    const SideWallEstimate &left_wall,
    const SideWallEstimate &right_wall,
    float speed,
    float gyro_z_dps,
    float dt
) {
  bool left_wall_valid = left_wall.valid;
  bool right_wall_valid = right_wall.valid;

  // 1. Heading error from side sensors (positive = robot turned into left wall, demanding RIGHT steer)
  float raw_heading_rad = 0.0f;
  if (left_wall_valid && right_wall_valid) {
    raw_heading_rad = 0.5f * (left_wall.heading_error_rad + right_wall.heading_error_rad);
  } else if (left_wall_valid) {
    raw_heading_rad = left_wall.heading_error_rad;
  } else if (right_wall_valid) {
    raw_heading_rad = right_wall.heading_error_rad;
  }

  // Low-pass filter heading error to attenuate sensor measurement noise
  if (left_wall_valid || right_wall_valid) {
    if (!m_heading_initialized) {
      m_filtered_heading_rad = raw_heading_rad;
      m_heading_initialized = true;
    } else {
      m_filtered_heading_rad +=
          m_config.heading_filter_alpha * (raw_heading_rad - m_filtered_heading_rad);
    }
  } else {
    // When walls are momentarily lost, smoothly decay the heading angle towards 0
    m_filtered_heading_rad *= 0.85f;
  }

  // 2. Cross-track error (in meters, positive = closer to left wall / right of target, demanding RIGHT steer)
  float cte_m = 0.0f;
  if (left_wall_valid && right_wall_valid) {
    cte_m = 0.5f * (right_wall.distance_mm - left_wall.distance_mm) * 0.001f;
  } else if (left_wall_valid) {
    cte_m = (m_config.target_wall_dist_mm - left_wall.distance_mm) * 0.001f;
  } else if (right_wall_valid) {
    cte_m = (right_wall.distance_mm - m_config.target_wall_dist_mm) * 0.001f;
  }

  // 3. Emergency side wall repulsion
  float repel_rad = 0.0f;
  float min_l = (left_wall.min_distance_mm > 0.0f) ? left_wall.min_distance_mm : left_wall.distance_mm;
  float min_r = (right_wall.min_distance_mm > 0.0f) ? right_wall.min_distance_mm : right_wall.distance_mm;

  if (left_wall_valid && min_l < m_config.side_danger_dist_mm) {
    float ratio = (m_config.side_danger_dist_mm - min_l) / m_config.side_danger_dist_mm;
    repel_rad += ratio * m_config.max_steer_rad;
  }
  if (right_wall_valid && min_r < m_config.side_danger_dist_mm) {
    float ratio = (m_config.side_danger_dist_mm - min_r) / m_config.side_danger_dist_mm;
    repel_rad -= ratio * m_config.max_steer_rad;
  }

  // 4. Stanley control law terms
  float v = std::max(speed, 0.0f);
  float cte_term_rad = std::atan2(m_config.k_cte * cte_m, v + m_config.k_soft);
  float heading_term_rad = m_config.k_heading * m_filtered_heading_rad;

  // Gyro yaw rate damping opposes fast turning to suppress oscillations
  float gyro_z_rads = gyro_z_dps * DEG_TO_RAD_F;
  float damping_term_rad = m_config.k_d * gyro_z_rads;

  // Total steering angle in radians
  float total_steer_rad =
      heading_term_rad + cte_term_rad - damping_term_rad + repel_rad;

  // Normalized steering command [-1.0f, 1.0f]
  float norm_cmd = total_steer_rad / m_config.max_steer_rad;
  norm_cmd = std::max(-1.0f, std::min(1.0f, norm_cmd));

  // Package diagnostic output
  StanleyOutput out;
  out.steering_cmd = norm_cmd;
  out.heading_error_rad = m_filtered_heading_rad;
  out.heading_error_deg = m_filtered_heading_rad * RAD_TO_DEG_F;
  out.cte_m = cte_m;
  out.delta_heading_term = heading_term_rad;
  out.delta_cte_term = cte_term_rad;
  out.delta_damping_term = damping_term_rad;
  out.delta_repel_term = repel_rad;
  out.left_wall_valid = left_wall_valid;
  out.right_wall_valid = right_wall_valid;

  return out;
}

StanleyOutput StanleyController::update(
    uint16_t dist_front_left,
    uint16_t dist_rear_left,
    uint16_t dist_front_right,
    uint16_t dist_rear_right,
    float speed,
    float gyro_z_dps,
    float dt
) {
  SideWallEstimate left_wall;
  left_wall.valid =
      (dist_front_left >= m_config.min_valid_dist_mm &&
       dist_front_left <= m_config.max_valid_dist_mm &&
       dist_rear_left >= m_config.min_valid_dist_mm &&
       dist_rear_left <= m_config.max_valid_dist_mm &&
       std::abs(static_cast<int>(dist_front_left) -
                static_cast<int>(dist_rear_left)) <=
           static_cast<int>(m_config.max_delta_mm));
  if (left_wall.valid) {
    left_wall.distance_mm = static_cast<float>(dist_front_left);
    left_wall.min_distance_mm = static_cast<float>(std::min(dist_front_left, dist_rear_left));
    left_wall.front_dist_mm = static_cast<float>(dist_front_left);
    left_wall.rear_dist_mm = static_cast<float>(dist_rear_left);
    left_wall.heading_error_rad = std::atan2(
        static_cast<float>(dist_rear_left) - static_cast<float>(dist_front_left),
        m_config.sensor_baseline_mm);
    left_wall.heading_error_deg = left_wall.heading_error_rad * RAD_TO_DEG_F;
  }

  SideWallEstimate right_wall;
  right_wall.valid =
      (dist_front_right >= m_config.min_valid_dist_mm &&
       dist_front_right <= m_config.max_valid_dist_mm &&
       dist_rear_right >= m_config.min_valid_dist_mm &&
       dist_rear_right <= m_config.max_valid_dist_mm &&
       std::abs(static_cast<int>(dist_front_right) -
                static_cast<int>(dist_rear_right)) <=
           static_cast<int>(m_config.max_delta_mm));
  if (right_wall.valid) {
    right_wall.distance_mm = static_cast<float>(dist_front_right);
    right_wall.min_distance_mm = static_cast<float>(std::min(dist_front_right, dist_rear_right));
    right_wall.front_dist_mm = static_cast<float>(dist_front_right);
    right_wall.rear_dist_mm = static_cast<float>(dist_rear_right);
    right_wall.heading_error_rad = std::atan2(
        static_cast<float>(dist_front_right) - static_cast<float>(dist_rear_right),
        m_config.sensor_baseline_mm);
    right_wall.heading_error_deg = right_wall.heading_error_rad * RAD_TO_DEG_F;
  }

  return update(left_wall, right_wall, speed, gyro_z_dps, dt);
}
