#pragma once
#include <stdint.h>

/**
 * Configuration parameters for the Stanley Path-Tracking Controller.
 */
struct StanleyConfig {
  float sensor_baseline_mm = 110.0f;      // Distance between front and rear side sensors (110 mm)
  float k_heading = 1.0f;                 // Gain for heading error term
  float k_cte = 2.2f;                     // Cross-track error gain (1/m)
  float k_soft = 0.25f;                   // Velocity softening constant (avoids div by zero, controls low-speed behavior)
  float k_d = 0.04f;                      // Gyro yaw rate damping gain (rad / (rad/s))
  float max_steer_rad = 0.69813f;         // Physical max front wheel steering angle in radians (40 deg = ~0.698 rad)

  float max_valid_dist_mm = 600.0f;       // Maximum distance to consider side wall valid
  float min_valid_dist_mm = 30.0f;        // Minimum distance to consider side wall valid
  float max_delta_mm = 110.0f;            // Maximum |d_front - d_rear| to accept as a continuous flat wall

  float target_wall_dist_mm = 200.0f;     // Target distance when single-wall tracking (e.g. 200 mm)
  float side_danger_dist_mm = 140.0f;     // Threshold for emergency repulsion
  float heading_filter_alpha = 0.35f;     // Low-pass EMA filter smoothing factor for heading
};

/**
 * Side wall estimate directly computed from sensor (e.g. multi-zone ToF).
 */
struct SideWallEstimate {
  float distance_mm = 0.0f;        // Perpendicular distance to wall in mm
  float min_distance_mm = 0.0f;    // Minimum observed distance to wall (for repulsion)
  float front_dist_mm = 0.0f;      // Observed or projected front distance in mm
  float rear_dist_mm = 0.0f;       // Observed or projected rear distance in mm
  float heading_error_rad = 0.0f;  // Heading error in radians (positive = vehicle turned into wall)
  float heading_error_deg = 0.0f;  // Heading error in degrees
  bool valid = false;              // True if wall detection is reliable
};

/**
 * Diagnostic outputs produced by each Stanley controller update step.
 */
struct StanleyOutput {
  float steering_cmd = 0.0f;        // Normalized steering command in [-1.0f, 1.0f]
  float heading_error_rad = 0.0f;   // Filtered heading error in radians
  float heading_error_deg = 0.0f;   // Filtered heading error in degrees
  float cte_m = 0.0f;               // Cross-track error in meters
  float delta_heading_term = 0.0f;  // Steering contribution from heading error (rad)
  float delta_cte_term = 0.0f;      // Steering contribution from cross-track error (rad)
  float delta_damping_term = 0.0f;  // Steering contribution from gyro damping (rad)
  float delta_repel_term = 0.0f;    // Steering contribution from emergency repulsion (rad)
  bool left_wall_valid = false;     // True if left side wall pair is valid
  bool right_wall_valid = false;    // True if right side wall pair is valid
};

/**
 * Stanley Path-Tracking Controller for wall-following autonomous vehicles.
 */
class StanleyController {
public:
  StanleyController();
  explicit StanleyController(const StanleyConfig &config);

  /**
   * Update the Stanley controller with direct side wall estimates (e.g. from VL53L5CX).
   *
   * @param left_wall Side wall estimate for left side.
   * @param right_wall Side wall estimate for right side.
   * @param speed Forward speed proxy (e.g. normalized current throttle in [0.0, 1.0]).
   * @param gyro_z_dps Yaw rate from IMU in degrees per second.
   * @param dt Delta time in seconds since last update.
   * @return Computed StanleyOutput containing normalized steering and telemetry data.
   */
  StanleyOutput update(
      const SideWallEstimate &left_wall,
      const SideWallEstimate &right_wall,
      float speed,
      float gyro_z_dps,
      float dt
  );

  /**
   * Legacy update with 4 discrete distance measurements (for backwards compatibility).
   */
  StanleyOutput update(
      uint16_t dist_front_left,
      uint16_t dist_rear_left,
      uint16_t dist_front_right,
      uint16_t dist_rear_right,
      float speed,
      float gyro_z_dps,
      float dt
  );

  /**
   * Reset filter states and accumulated history.
   */
  void reset();

  void setConfig(const StanleyConfig &config) { m_config = config; }
  const StanleyConfig &getConfig() const { return m_config; }
  StanleyConfig &getConfig() { return m_config; }

private:
  StanleyConfig m_config;
  float m_filtered_heading_rad = 0.0f;
  bool m_heading_initialized = false;
};
