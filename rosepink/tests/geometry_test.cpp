#include "control/sensor_startup.h"
#include "side_guidance.h"
#include <cassert>
#include <cmath>
#include <cstdio>

constexpr float rad = 3.14159265f / 180.0f;
unsigned zone(unsigned sensor, unsigned r, unsigned c) {
  return (config::tof_flip_y[sensor] ? 3 - r : r) * 4 +
         (config::tof_flip_x[sensor] ? 3 - c : c);
}
float angle(unsigned column, float fov) {
  return std::atan(((column + 0.5f) * 0.5f - 1.0f) *
                   std::tan(fov * rad * 0.5f));
}
VL53L5CX_ResultsData empty_frame() {
  VL53L5CX_ResultsData data = {};
  for (unsigned i = 0; i < 16; ++i)
    data.target_status[i] = 255;
  return data;
}
void hit(VL53L5CX_ResultsData &data, unsigned sensor, unsigned r, unsigned c,
         float range) {
  unsigned i = zone(sensor, r, c);
  data.nb_target_detected[i] = 1;
  data.target_status[i] = 5;
  data.distance_mm[i] = static_cast<int16_t>(std::lround(range));
}
VL53L5CX_ResultsData wall_frame(unsigned sensor, float intercept, float slope) {
  auto data = empty_frame();
  float sign = sensor == 1 ? -1.0f : 1.0f;
  for (unsigned r = 0; r < 2; ++r)
    for (unsigned c = 0; c < 4; ++c) {
      float yaw = config::tof_yaw_deg[sensor] * rad +
                  angle(c, config::tof_horizontal_fov_deg);
      float cv = std::cos(angle(r, config::tof_vertical_fov_deg));
      float dx = cv * std::cos(yaw), dy = sign * cv * std::sin(yaw);
      float origin = sign * config::tof_right_mm[sensor];
      float range =
          (intercept - origin + slope * config::tof_forward_mm[sensor]) /
          (dy - slope * dx);
      assert(range > 0.0f && range < 2000.0f);
      hit(data, sensor, r, c, range);
    }
  return data;
}
SensorCommand sent;
bool send_command(const SensorCommand &command) {
  sent = command;
  return true;
}
int main() {
  VL53L5CX_ResultsData invalid = {};
  assert(!process_forward_vl53l5cx(invalid, 0, 0).valid);
  for (unsigned i = 0; i < 3; ++i) {
    auto open = process_forward_vl53l5cx(empty_frame(), i, 0);
    assert(open.valid && open.open && !open.wall.valid);
  }
  auto data = empty_frame();
  for (unsigned r = 0; r < 2; ++r)
    for (unsigned c = 0; c < 4; ++c)
      hit(data, 0, r, c,
          400.0f / (std::cos(angle(r, config::tof_vertical_fov_deg)) *
                    std::cos(angle(c, config::tof_horizontal_fov_deg))));
  auto front = process_forward_vl53l5cx(data, 0, 0);
  assert(front.valid && !front.open &&
         std::abs(int(front.distance_mm) - 400) <= 1);
  hit(data, 0, 1, 1, 60);
  assert(process_forward_vl53l5cx(data, 0, 0).distance_mm < 65);
  // Floor points and far off-corridor returns must not trigger a front
  // obstacle.
  data = empty_frame();
  float bottom = -angle(3, config::tof_vertical_fov_deg);
  hit(data, 0, 3, 1, -config::tof_height_mm[0] / std::sin(bottom));
  assert(process_forward_vl53l5cx(data, 0, 0).open);
  data = empty_frame();
  hit(data, 0, 0, 0, 900);
  assert(process_forward_vl53l5cx(data, 0, 0).open);
  // Recorded center matrix, in its original unflipped physical order. Short
  // floor returns must move to the bottom and not masquerade as front walls.
  data = empty_frame();
  const int16_t captured_ranges[16] = {243, 254,  269,  263,  429,  453,
                                       460, 417,  1258, 1627, 2003, 2152,
                                       946, 3899, 3995, 4008};
  const uint8_t captured_status[16] = {5, 5, 5, 5,   5,   5,   5,   5,
                                       5, 5, 5, 255, 255, 255, 255, 2};
  for (unsigned i = 0; i < 16; ++i) {
    data.distance_mm[i] = captured_ranges[i];
    data.target_status[i] = captured_status[i];
    data.nb_target_detected[i] = captured_status[i] == 255 ? 0 : 1;
  }
  auto recorded = process_forward_vl53l5cx(data, 0, 0);
  assert(recorded.valid && recorded.open && recorded.nearest_zone == 255);
  assert((recorded.floor_zones & 0xff00) == 0xff00);
  // A real close obstacle in a downward-looking zone remains an obstacle.
  hit(data, 0, 2, 1, 100);
  auto close = process_forward_vl53l5cx(data, 0, 0);
  assert(!close.open && close.distance_mm < 105 && close.nearest_zone == 9);
  // A lower-middle floor return was missed by the old center-ray-only filter.
  data = empty_frame();
  hit(data, 0, 2, 1, 430);
  assert(process_forward_vl53l5cx(data, 0, 0).open);
  // Check mounting flips, robot-frame distance and steering signs on both
  // sides.
  for (unsigned sensor = 1; sensor < 3; ++sensor) {
    auto parallel =
        process_forward_vl53l5cx(wall_frame(sensor, 240, 0), sensor, 0);
    assert(parallel.valid && parallel.wall.valid &&
           parallel.wall.confidence > 0.3f);
    assert(std::fabs(parallel.wall.distance_mm - 240) < 3);
    assert(std::fabs(parallel.wall.heading_error_deg) < 1);
    auto angled =
        process_forward_vl53l5cx(wall_frame(sensor, 240, 0.2f), sensor, 0);
    float expected = (sensor == 1 ? -1.0f : 1.0f) * std::atan(0.2f);
    assert(angled.wall.confidence > 0.3f);
    assert(std::fabs(angled.wall.heading_error_rad - expected) < 0.03f);
    // Sparse close obstacles still contribute when no heading can be fitted.
    data = empty_frame();
    hit(data, sensor, 1, 1, 70);
    auto sparse = process_forward_vl53l5cx(data, sensor, 0);
    assert(sparse.valid && sparse.distance_mm == 70 &&
           sparse.wall.confidence == 0);
  }
  // Right matrix captured with no flips on a straight. Reversing both axes
  // must recover a near-parallel wall instead of the erroneous -82 degree fit.
  data = empty_frame();
  const int16_t right_ranges[16] = {121, 249, 250, 238, 479, 548, 633, 785,
                                    483, 559, 658, 835, 454, 536, 518, 429};
  for (unsigned i = 0; i < 16; ++i) {
    data.distance_mm[i] = right_ranges[i];
    data.target_status[i] = i < 12 ? 5 : 255;
    data.nb_target_detected[i] = i < 12 ? 1 : 0;
  }
  auto right = process_forward_vl53l5cx(data, 2, 0);
  assert(right.valid && right.wall.valid && right.wall.confidence > 0.2f);
  assert(std::fabs(right.wall.heading_error_deg) < 10.0f);
  assert(right.wall.distance_mm > 400 && right.wall.distance_mm < 550);
  // The trial mask removes only the suspect corner from clearance/fitting.
  assert(right.nearest_zone == 7 && right.distance_mm == 483);
  assert(!(right.clearance_zones & (1U << 15)));
  // A near obstacle in the neighboring bottom zone is still observed.
  hit(data, 2, 3, 2, 100);
  auto neighboring = process_forward_vl53l5cx(data, 2, 0);
  assert(neighboring.nearest_zone == 14 && neighboring.distance_mm == 100);
  // The same logical corner on the left remains enabled.
  data = empty_frame();
  hit(data, 1, 3, 3, 100);
  auto left_corner = process_forward_vl53l5cx(data, 1, 0);
  assert(left_corner.nearest_zone == 15 && left_corner.distance_mm == 100);
  std::printf("recorded right: wall=%.0f mm heading=%.1f deg confidence=%.2f "
              "nearest=%u mm zone=%u\n",
              right.wall.distance_mm, right.wall.heading_error_deg,
              right.wall.confidence, right.distance_mm, right.nearest_zone);
  // Calibration can finish before ranging initialization; launch stays gated.
  SensorStartup startup;
  MainSensorSnapshot imu;
  startup.enter(0);
  assert(!startup.tick(imu, true, 0, 5000, 5000, 50, send_command));
  imu.last_cmd_id_ack = sent.cmd_id;
  imu.last_cmd_result = SensorCommandResult::ACCEPTED;
  imu.imu_calib_start_cmd_id = sent.cmd_id;
  imu.imu_calib_in_progress = true;
  imu.imu_calib_sample_count = 100;
  assert(!startup.tick(imu, true, 5000, 5000, 5000, 50, send_command));
  assert(sent.type == SensorCommandType::FINISH_IMU_CALIBRATION);
  imu.last_cmd_id_ack = sent.cmd_id;
  imu.last_cmd_result = SensorCommandResult::COMPLETED;
  imu.imu_calib_in_progress = false;
  imu.imu_calib_valid = imu.imu_calib_completed = true;
  assert(startup.tick(imu, true, 5010, 5000, 5000, 50, send_command));
  RosepinkToFSnapshot tof;
  assert(!sensors::ranging_ready(tof, 5010000));
  tof.recovering = false;
  sensor_policy::success(tof.dist_front, 5010000);
  sensor_policy::success(tof.dist_left, 5010000);
  assert(!sensors::ranging_ready(tof, 5010000));
  sensor_policy::success(tof.dist_right, 5010000);
  assert(sensors::ranging_ready(tof, 5010000));
  assert(!sensors::ranging_ready(tof, 5120000));
  tof.recovering = true;
  assert(!sensors::ranging_ready(tof, 5010000));
  std::puts("geometry and startup checks passed");
}
