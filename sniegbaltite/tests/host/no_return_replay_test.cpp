#include "control/sensor_motion_gate.h"
#include "sensors/tof_bus.h"
#include <cassert>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

int main(int argc, char **argv) {
  assert(argc == 2);
  std::ifstream fixture(argv[1]);
  assert(fixture.is_open());
  std::string line;
  unsigned cuts = 0;
  while (std::getline(fixture, line)) {
    if (line.empty() || line[0] == '#')
      continue;
    std::istringstream row(line);
    RobotState s{};
    s.running_state = RunningState::RUNNING;
    s.target_throttle = s.current_throttle = 0.3f;
    uint32_t time_ms;
    row >> time_ms;
    s.current_time_us = time_ms * 1000U;
    for (unsigned i = 0; i < 3; ++i) {
      auto &beam = *sensor_policy::distances(s, i);
      unsigned health, age_ms, raw_status, raw_age_ms;
      row >> beam.distance_mm >> health >> age_ms >> beam.raw_range_mm >>
          raw_status >> raw_age_ms;
      beam.health = static_cast<SensorHealth>(health);
      beam.timestamp_us = s.current_time_us - age_ms * 1000U;
      beam.raw_status = raw_status;
      beam.raw_timestamp_us = s.current_time_us - raw_age_ms * 1000U;
    }
    assert(!row.fail());
    assert(!sensor_policy::required(s, s.current_time_us));
    ++cuts;
    for (unsigned i = 0; i < 3; ++i) {
      auto &beam = *sensor_policy::distances(s, i);
      uint16_t raw = beam.raw_range_mm;
      uint8_t status = beam.raw_status;
      uint32_t sampled_us = beam.raw_timestamp_us;
      sensor_policy::accept_range(beam, raw, status, sampled_us, 30, 1000,
                                  true);
    }
    assert(s.dist_front.health == SensorHealth::NO_RETURN);
    assert(sensor_policy::required(s, s.current_time_us));
    assert(!sensor_motion_inhibit(s, false));
    assert(s.target_throttle == 0.3f);
  }
  assert(cuts == 42);
  printf("recorded no-return replay: %u sensor-related throttle cuts removed\n",
         cuts);
}
