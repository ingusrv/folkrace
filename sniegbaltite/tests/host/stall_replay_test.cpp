#include "stall_detector.h"
#include <cassert>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

int main(int argc, char **argv) {
  assert(argc == 2);
  std::ifstream fixture(argv[1]);
  assert(fixture.is_open());
  StallDetector detector;
  RobotState s{};
  s.running_state = RunningState::RUNNING;
  std::string line;
  uint32_t first_us = 0, detected_us = 0;
  while (std::getline(fixture, line)) {
    if (line.empty() || line[0] == '#')
      continue;
    std::istringstream row(line);
    uint32_t time_ms;
    auto &imu = s.imu.measurement;
    row >> time_ms >> s.current_throttle >> s.target_throttle >> imu.yaw >>
        imu.pitch >> imu.roll >> imu.accel_x >> imu.accel_y >> imu.accel_z;
    s.current_time_us = time_ms * 1000U;
    if (!first_us)
      first_us = s.current_time_us;
    sensor_policy::success(s.imu, s.current_time_us);
    for (unsigned i = 0; i < 7; ++i) {
      unsigned health, age_ms;
      auto &beam = *sensor_policy::distances(s, i);
      row >> beam.distance_mm >> health >> age_ms;
      beam.health = static_cast<SensorHealth>(health);
      beam.timestamp_us = s.current_time_us - age_ms * 1000U;
    }
    assert(!row.fail());
    if (detector.update(s, true, 1) && !detected_us)
      detected_us = s.current_time_us;
  }
  assert(detected_us != 0);
  // Recorded at ~30 Hz: allow one telemetry interval past the timeout.
  assert(detected_us - first_us >= config::stall_timeout_ms * 1000U);
  assert(detected_us - first_us <= config::stall_timeout_ms * 1000U + 50000U);
  printf("recorded stall detected after %u ms\n",
         (detected_us - first_us) / 1000);
}
