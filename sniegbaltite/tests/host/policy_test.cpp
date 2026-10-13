#include "control/sensor_guidance.h"
#include "control/sensor_motion_gate.h"
#include "control/sensor_startup.h"
#include "control/slew_rate_limiter.h"
#include "sensors/main_owner.h"
#include "telemetry.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

static std::vector<SensorCommand> sent;
static bool queue_full = false;
static bool send(const SensorCommand &cmd) {
  sent.push_back(cmd);
  return !queue_full;
}
static void ack(MainSensorSnapshot &s, SensorCommandResult result) {
  s.last_cmd_id_ack = sent.back().cmd_id;
  s.last_cmd_result = result;
}
static SensorData distance(uint16_t mm, uint32_t now) {
  SensorData s;
  s.distance_mm = mm;
  sensor_policy::success(s, now);
  return s;
}
static void calibration_handshake() {
  SensorStartup startup;
  MainSensorSnapshot s;
  sent.clear();
  startup.enter(0);
  queue_full = true;
  assert(!startup.tick(s, true, 0, 5000, 5000, 50, send));
  auto id = sent.back().cmd_id;
  assert(!startup.tick(s, true, 10, 5000, 5000, 50, send));
  assert(sent.back().cmd_id == id); // Full queue does not advance command ID.
  queue_full = false;
  startup.tick(s, true, 20, 5000, 5000, 50, send);
  ack(s, SensorCommandResult::ACCEPTED);
  s.imu_calib_in_progress = true;
  s.imu_calib_start_cmd_id = id;
  s.imu_calib_sample_count = 49;
  startup.tick(s, true, 5000, 5000, 5000, 50, send);
  assert(sent.back().type == SensorCommandType::START_IMU_CALIBRATION);
  s.imu_calib_sample_count = 50;
  assert(!startup.tick(s, true, 5010, 5000, 5000, 50, send));
  assert(sent.back().type == SensorCommandType::FINISH_IMU_CALIBRATION);
  ack(s, SensorCommandResult::INSUFFICIENT_SAMPLES);
  startup.tick(s, true, 5020, 5000, 5000, 50, send);
  assert(sent.back().cmd_id != s.last_cmd_id_ack);
  ack(s, SensorCommandResult::COMPLETED);
  s.imu_calib_valid = true;
  s.imu_calib_completed = true;
  s.imu_calib_in_progress = false;
  assert(startup.tick(s, true, 5030, 5000, 5000, 50, send));
  startup.permitted_motion();
  auto commands = sent.size();
  startup.tick(s, false, 5100, 5000, 5000, 50, send);
  startup.enter(6000);
  assert(startup.tick(s, true, 11000, 5000, 5000, 50, send));
  assert(sent.size() == commands); // Never recalibrate after motion.
}
static void abort_gate() {
  SensorStartup startup;
  MainSensorSnapshot s;
  sent.clear();
  startup.enter(0);
  startup.tick(s, true, 0, 5000, 5000, 50, send);
  auto start = sent.back();
  // Deadline cannot manufacture an acknowledgement or discard an outstanding
  // start.
  assert(!startup.tick(s, true, 10001, 5000, 5000, 50, send));
  assert(sent.back().cmd_id == start.cmd_id);
  ack(s, SensorCommandResult::ACCEPTED);
  s.imu_calib_in_progress = true;
  assert(!startup.tick(s, true, 10002, 5000, 5000, 50, send));
  assert(sent.back().type == SensorCommandType::ABORT_IMU_CALIBRATION);
  assert(!startup.tick(s, true, 20000, 5000, 5000, 50, send));
  ack(s, SensorCommandResult::ABORTED);
  s.imu_calib_in_progress = false;
  assert(startup.tick(s, true, 20001, 5000, 5000, 50, send));
  startup.permitted_motion();
  assert(!startup.imu_enabled(s));
  auto count = sent.size();
  startup.enter(25000);
  assert(startup.tick(s, true, 30000, 5000, 5000, 50, send));
  assert(sent.size() == count);
  // A late calibration result from another attempt cannot unlock the countdown.
  SensorStartup mismatched;
  MainSensorSnapshot bad;
  sent.clear();
  mismatched.enter(0);
  mismatched.tick(bad, true, 0, 5000, 5000, 50, send);
  ack(bad, SensorCommandResult::ACCEPTED);
  bad.imu_calib_sample_count = 50;
  mismatched.tick(bad, true, 5000, 5000, 5000, 50, send);
  ack(bad, SensorCommandResult::COMPLETED);
  bad.imu_calib_start_cmd_id = 999;
  bad.imu_calib_completed = bad.imu_calib_valid = true;
  assert(!mismatched.tick(bad, true, 5001, 5000, 5000, 50, send));
  assert(!mismatched.tick(bad, true, 10001, 5000, 5000, 50, send));
  assert(sent.back().type == SensorCommandType::ABORT_IMU_CALIBRATION);
}
static void motion_and_guidance() {
  constexpr uint32_t now = 1000000;
  RobotState s{};
  s.dist_front = distance(1000, now);
  s.dist_left = distance(100, now);
  s.dist_right.distance_mm = 1;
  s.dist_right.health = SensorHealth::READ_ERROR;
  assert(sensor_policy::required(s, now));
  auto guide = sensor_guidance(s, now, 350.0f, 160, 500, 180, 1000);
  assert(guide.steering >
         0.0f); // Failed right reading cannot steer into left wall.
  auto without_sides = guide.steering;
  s.dist_side_front_left = s.dist_side_rear_left = distance(1000, now);
  s.dist_side_front_right = s.dist_side_rear_right = distance(1000, now);
  assert(sensor_guidance(s, now, 350.0f, 160, 500, 180, 1000).steering ==
         without_sides);
  s.dist_left.health = SensorHealth::READ_ERROR;
  assert(!sensor_policy::required(s, now));
  s.dist_right = distance(100, now);
  assert(sensor_guidance(s, now, 350.0f, 160, 500, 180, 1000).steering < 0.0f);
  s.dist_front.timestamp_us -= 100001;
  assert(!sensor_policy::required(s, now));
  sensor_policy::expire_all(s, now);
  assert(s.dist_front.health == SensorHealth::STALE);
  assert(s.dist_front.distance_mm == 1000);
  for (RunningState mode :
       {RunningState::RUNNING, RunningState::REVERSE,
        RunningState::REVERSE_TURN, RunningState::WRONG_WAY_RECOVERY}) {
    for (float output : {-0.6f, 0.6f}) {
      s.running_state = mode;
      s.current_throttle = output;
      s.target_throttle = output;
      s.current_steering = -0.3f;
      s.steering_pid.integral = 0.8f;
      s.maneuver_start_us = 123;
      SlewRateLimiter limiter(1.0f, 3.0f, output);
      assert(sensor_motion_inhibit(s, true));
      assert(s.target_throttle == 0.0f && s.current_throttle == output);
      assert(s.target_steering == -0.3f && s.maneuver_start_us == 0);
      assert(s.running_state == RunningState::RUNNING &&
             s.steering_pid.integral == 0.0f);
      float next = limiter.update(s.target_throttle, 0.01f);
      assert(std::abs(next) > 0.0f && std::abs(next) < std::abs(output));
      for (int i = 0; i < 25; ++i)
        next = limiter.update(0.0f, 0.01f);
      assert(next == 0.0f);
      assert(limiter.update(0.6f, 0.01f) <= 0.010001f);
    }
  }
  s.running_state = RunningState::IDLE;
  assert(!sensor_motion_inhibit(s, true));
  assert(s.running_state == RunningState::IDLE);
}
static void quality_and_telemetry() {
  SensorData s = distance(120, 100);
  for (uint8_t code : {0, 4, 5, 6, 7, 8, 9, 10, 12, 13, 14, 15}) {
    assert(sensor_policy::accept_range(s, 900, code << 3, 200, 30, 1000));
    assert(s.health == SensorHealth::RANGE_INVALID && s.error_count == 0);
    assert(s.timestamp_us == 100 && s.distance_mm == 120 &&
           s.raw_timestamp_us == 200);
  }
  for (uint16_t raw : {8190, 8191}) {
    assert(sensor_policy::accept_range(s, raw, 11 << 3, 300, 30, 1000));
    assert(s.health == SensorHealth::RANGE_INVALID && s.raw_range_mm == raw);
  }
  for (uint8_t code : {1, 2, 3}) {
    assert(!sensor_policy::accept_range(s, 12, code << 3, 400, 30, 1000));
    assert(s.health == SensorHealth::READ_ERROR &&
           s.last_error == SensorError::HARDWARE_ERROR);
  }
  assert(s.error_count == 3 && s.consecutive_errors == 3);
  sensor_policy::accept_range(s, 1500, 11 << 3, 500, 30, 1000);
  assert(s.health == SensorHealth::OK && s.distance_mm == 1000 &&
         s.consecutive_errors == 0);
  s.error_count = UINT32_MAX;
  s.consecutive_errors = UINT16_MAX;
  sensor_policy::fail(s, SensorError::NACK);
  assert(s.error_count == UINT32_MAX && s.consecutive_errors == UINT16_MAX);
  RobotState state{};
  state.current_time_us = 1000000;
  state.dist_left = s;
  state.dist_right = distance(432, 990000);
  state.dist_right.raw_range_mm = 1234;
  state.dist_right.raw_status = 0x58;
  state.dist_right.raw_timestamp_us = 995000;
  auto p = make_telemetry(state, 0);
  assert(sizeof(p) == 184 && p.protocol_version == 1);
  assert(p.raw_tof[1].range_mm == 1234 && p.raw_tof[1].age_ms == 5);
  assert(p.sensors[1].age_ms == 10 && p.sensors[2].error_count == UINT16_MAX);
  assert(p.raw_tof[0].age_ms == UINT16_MAX);
  assert(is_sensor_fresh(UINT32_MAX - 10, 10, 100));
  assert(!is_sensor_fresh(0, 10, 100));
}
static void open_space_racing_fallback() {
  for (uint8_t code : {4, 6}) {
    for (uint16_t raw : {8190, 8191}) {
      SensorData strict = distance(120, 100);
      sensor_policy::accept_range(strict, raw, code << 3, 200, 30, 1000);
      assert(strict.health == SensorHealth::RANGE_INVALID);
      assert(
          !sensor_policy::available(strict, 200)); // Other robots keep policy.
      SensorData racing = strict;
      assert(sensor_policy::accept_range(racing, raw, code << 3, 200, 30, 1000,
                                         true));
      assert(racing.health == SensorHealth::NO_RETURN);
      assert(racing.distance_mm == 1000 && racing.timestamp_us == 200);
      assert(racing.raw_range_mm == raw && racing.raw_timestamp_us == 200);
      assert(racing.error_count == 0 && sensor_policy::available(racing, 200));
      assert(!sensor_policy::available(racing, 100201));
      sensor_policy::expire(racing, 100201, sensor_policy::tof_age_us);
      assert(racing.health == SensorHealth::STALE);
    }
  }
  for (uint8_t code : {0, 1, 2, 3, 5, 7, 8, 9, 10, 11, 12, 13, 14, 15}) {
    SensorData sample;
    sensor_policy::accept_range(sample, 8191, code << 3, 200, 30, 1000, true);
    assert(!sensor_policy::available(sample, 200));
  }
  SensorData near;
  sensor_policy::accept_range(near, 80, 4 << 3, 200, 30, 1000, true);
  assert(
      !sensor_policy::available(near, 200)); // Weak non-sentinel isn't clear.

  TwoWire bus;
  sensor_policy::Settings settings;
  settings.allow_no_return = true;
  sensor_policy::ToFBus tof(bus, 4, 5, 30, 1000, 20000, settings);
  tof.count = 3;
  tof.devices[1].xshut = 41;
  tof.devices[2].xshut = 42;
  tof.begin();
  tof.recover();
  auto calls = VL53L0X::init_calls;
  VL53L0X::result_mm = 8191;
  VL53L0X::result_status = 4 << 3;
  RobotState state{};
  state.running_state = RunningState::RUNNING;
  state.target_throttle = 0.4f;
  state.current_throttle = 0.25f;
  for (unsigned i = 0; i < 200; ++i) {
    delay(10);
    tof.poll();
    tof.recover();
    state.dist_front = tof.devices[0].data;
    state.dist_right = tof.devices[1].data;
    state.dist_left = tof.devices[2].data;
    state.current_time_us = micros();
    assert(sensor_policy::required(state, state.current_time_us));
    assert(!sensor_motion_inhibit(
        state, !sensor_policy::required(state, state.current_time_us)));
    assert(state.target_throttle == 0.4f);
    auto guidance = sensor_guidance(state, state.current_time_us, 350.0f, 160,
                                    500, 180, 1000);
    assert(guidance.urgency == 0.0f && guidance.steering == 0.0f);
    auto telemetry = make_telemetry(state, 0);
    assert(telemetry.flags & 8);
    assert(telemetry.sensors[0].health ==
           static_cast<uint8_t>(SensorHealth::NO_RETURN));
    assert(telemetry.dist_front == 1000 &&
           telemetry.raw_tof[0].range_mm == 8191);
  }
  assert(VL53L0X::init_calls ==
         calls); // Continuous weak results don't reset bus.
  VL53L0X::ready = false;
  delay(101);
  tof.poll();
  state.dist_front = tof.devices[0].data;
  assert(
      !sensor_policy::required(state, micros())); // Silent device still stops.
  VL53L0X::ready = true;
  delay(500);
  tof.recover();
  tof.poll();
  VL53L0X::injected_error = 2;
  tof.poll();
  assert(!sensor_policy::available(tof.devices[0].data, micros()));
  VL53L0X::injected_error = 0;
  VL53L0X::result_mm = 500;
  VL53L0X::result_status = 11 << 3;
}
static void recovery_backoff_and_grace() {
  sensor_policy::Backoff b;
  b.defer(100);
  assert(b.delay_ms == 500 && !b.due(599) && b.due(600));
  b.acquired(1000);
  b.fault();
  b.defer(600);
  assert(b.delay_ms == 1000);
  b.defer(1600);
  assert(b.delay_ms == 2000);
  b.defer(3600);
  b.defer(7600);
  assert(b.delay_ms == 5000);
  b.acquired(10000);
  b.acquired(2010000);
  b.defer(2010);
  assert(b.delay_ms == 500);
  TwoWire bus;
  sensor_policy::ToFBus tof(bus, 4, 5, 30, 1000);
  tof.count = 1;
  tof.begin();
  assert(tof.recovery_due());
  tof.recover();
  tof.poll();
  assert(!tof.recovery_due());
  auto old = tof.devices[0].data.timestamp_us;
  tof.pause_begin();
  delay(450);
  tof.pause_end();
  VL53L0X::ready = false;
  tof.poll();
  assert(tof.devices[0].data.timestamp_us == old && tof.grace_active());
  delay(99);
  tof.poll();
  assert(!tof.devices[0].pending);
  delay(1);
  tof.poll();
  assert(tof.devices[0].pending);
  assert(tof.devices[0].data.last_error == SensorError::MEASUREMENT_TIMEOUT);
  auto calls = VL53L0X::init_calls;
  assert(!tof.recovery_due()); // C3 recovery must wait for the retry deadline.
  tof.recover();
  assert(VL53L0X::init_calls == calls);
  delay(500);
  assert(tof.recovery_due());
  tof.recover();
  VL53L0X::ready = true;
  tof.poll();
  assert(tof.devices[0].data.health == SensorHealth::OK);
  // Ready results are consumed first even after a known 450 ms owner pause.
  tof.pause_begin();
  delay(450);
  tof.pause_end();
  tof.poll();
  assert(!tof.grace_active() && tof.devices[0].data.health == SensorHealth::OK);
  calls = VL53L0X::init_calls;
  VL53L0X::result_status = 7 << 3;
  for (unsigned i = 0; i < 300; ++i) {
    delay(10);
    tof.poll();
    tof.recover();
  }
  assert(VL53L0X::init_calls == calls &&
         tof.devices[0].data.health == SensorHealth::RANGE_INVALID);
  VL53L0X::result_status = 11 << 3;
}
static void failed_primary_keeps_group_isolated() {
  TwoWire bus;
  sensor_policy::ToFBus tof(bus, 4, 5, 30, 1000);
  tof.count = 3;
  tof.devices[1].xshut = 41;
  tof.devices[2].xshut = 42;
  tof.begin();
  auto calls = VL53L0X::init_calls;
  VL53L0X::init_ok = false;
  assert(tof.recover());
  assert(VL53L0X::init_calls == calls + 1);
  assert(!tof.devices[1].active && !tof.devices[2].active);
  assert(!tof.recovery_due());
  VL53L0X::init_ok = true;
  delay(500);
  assert(tof.recover());
  assert(tof.devices[0].active && tof.devices[1].active &&
         tof.devices[2].active);
}
static void immediate_publication_and_raw_retention() {
  TwoWire bus;
  sensor_policy::ToFBus tof(bus, 4, 5, 30, 1000);
  tof.count = 2;
  tof.devices[1].xshut = 41;
  tof.begin();
  tof.recover();
  tof.poll();
  auto saved = tof.devices[0].data;
  struct Observer {
    sensor_policy::ToFBus *tof;
    unsigned failure_read_index = 0;
  } observer{&tof};
  tof.set_publisher(
      [](void *ctx) {
        auto &o = *static_cast<Observer *>(ctx);
        if (!o.failure_read_index &&
            o.tof->devices[0].data.health == SensorHealth::READ_ERROR)
          o.failure_read_index = VL53L0X::reads;
      },
      &observer);
  VL53L0X::reads = 0;
  VL53L0X::injected_error = 2;
  tof.poll();
  assert(observer.failure_read_index ==
         1); // Published before any second-device transaction.
  assert(tof.devices[0].data.raw_timestamp_us == saved.raw_timestamp_us);
  VL53L0X::injected_error = 0;
  delay(500);
  tof.recover();
  tof.poll();
  // A partial result and an interrupt-clear failure both retain the entire raw
  // record.
  for (int phase = 0; phase < 2; ++phase) {
    saved = tof.devices[0].data;
    VL53L0X::result_mm = 8191;
    VL53L0X::fail_result = phase == 0;
    VL53L0X::fail_ack = phase == 1;
    tof.poll();
    assert(tof.devices[0].data.raw_range_mm == saved.raw_range_mm);
    assert(tof.devices[0].data.raw_timestamp_us == saved.raw_timestamp_us);
    assert(tof.devices[0].data.timestamp_us == saved.timestamp_us);
    VL53L0X::fail_result = VL53L0X::fail_ack = false;
    VL53L0X::result_mm = 500;
    delay(1100);
    tof.recover();
    tof.poll();
  }
}
static void seed_imu(TwoWire &bus) {
  bus.registers[0x1900] = 0x1e;
  bus.registers[0x6900] = 0x0f;
  bus.registers[0x1912] = 10;
  bus.registers[0x1914] = 20;
  bus.registers[0x1916] = 30;
  bus.registers[0x6902] = 40;
}
static void owner_budget_and_calibration() {
  auto q = xQueueCreate(1, sizeof(MainSensorSnapshot));
  auto commands = xQueueCreate(4, sizeof(SensorCommand));
  TwoWire bus;
  seed_imu(bus);
  sensor_policy::MainSensorOwner owner(bus, 4, 5, 42, 41, 30, 1000, 20000, q,
                                       commands);
  fake_imu_begin_ok = true;
  fake_accel_begins = 0;
  owner.begin();
  SensorCommand start{1, SensorCommandType::START_IMU_CALIBRATION};
  xQueueSend(commands, &start, 0);
  for (unsigned i = 0; i < 100; ++i) {
    owner.tick();
    delay(10);
  }
  assert(owner.snapshot.imu_calib_sample_count >= 50);
  auto count = owner.snapshot.imu_calib_sample_count;
  xQueueSend(commands, &start, 0);
  owner.tick(); // Duplicate must not clear accumulation.
  assert(owner.snapshot.imu_calib_sample_count >= count);
  SensorCommand finish{2, SensorCommandType::FINISH_IMU_CALIBRATION};
  xQueueSend(commands, &finish, 0);
  owner.tick();
  assert(owner.snapshot.last_cmd_result == SensorCommandResult::COMPLETED);
  float flat_pitch = owner.snapshot.imu_calib_flat_pitch;
  uint32_t generation = owner.snapshot.orientation_generation;
  // Alternate successful reinitialization with read failures. Successful calls
  // consume budget.
  for (unsigned attempt = 1; attempt <= 3; ++attempt) {
    assert(fake_accel_begins == attempt);
    bus.fail_status = 2;
    bus.fail_address = 0x19;
    for (unsigned i = 0; i < 4; ++i) {
      owner.tick();
      delay(10);
    }
    bus.fail_status = 0;
    for (unsigned i = 0; i < 250; ++i) {
      owner.tick();
      delay(10);
    }
  }
  assert(fake_accel_begins == 3 && owner.snapshot.imu_disabled_until_reboot);
  assert(owner.snapshot.imu.health == SensorHealth::OFFLINE);
  assert(owner.snapshot.imu_calib_valid &&
         owner.snapshot.imu_calib_flat_pitch == flat_pitch);
  assert(owner.snapshot.orientation_generation > generation);
  assert(owner.snapshot.dist_front.health == SensorHealth::OK);
  start.cmd_id = 3;
  xQueueSend(commands, &start, 0);
  owner.tick();
  assert(owner.snapshot.last_cmd_result == SensorCommandResult::REJECTED);
  SensorCommand abort{4, SensorCommandType::ABORT_IMU_CALIBRATION};
  xQueueSend(commands, &abort, 0);
  owner.tick();
  assert(owner.snapshot.last_cmd_result == SensorCommandResult::ABORTED);
  assert(owner.snapshot.imu_calib_valid &&
         !owner.snapshot.imu_calib_in_progress);
  delete q;
  delete commands;
}
static void failed_initialization_budget() {
  for (bool begin_success : {false, true}) {
    auto q = xQueueCreate(1, sizeof(MainSensorSnapshot));
    auto commands = xQueueCreate(4, sizeof(SensorCommand));
    TwoWire bus;
    seed_imu(bus);
    bus.fail_status = 2;
    bus.fail_address = 0x19;
    sensor_policy::MainSensorOwner owner(bus, 4, 5, 42, 41, 30, 1000, 20000, q,
                                         commands);
    fake_imu_begin_ok = begin_success;
    fake_accel_begins = 0;
    owner.begin();
    for (unsigned i = 0; i < 1200; ++i) {
      owner.tick();
      delay(10);
    }
    assert(fake_accel_begins == 3 && owner.snapshot.imu_disabled_until_reboot);
    assert(owner.snapshot.imu.timestamp_us == 0 &&
           owner.snapshot.dist_front.health == SensorHealth::OK);
    // Bus recovery and new commands cannot clear the lifetime latch.
    auto before = bus.transactions.size();
    VL53L0X::injected_error = 2;
    owner.tick();
    VL53L0X::injected_error = 0;
    for (unsigned i = 0; i < 700; ++i) {
      owner.tick();
      delay(10);
    }
    assert(fake_accel_begins == 3);
    for (unsigned i = before; i < bus.transactions.size(); ++i)
      assert(bus.transactions[i].first != 0x19 &&
             bus.transactions[i].first != 0x69);
    SensorCommand abort{1, SensorCommandType::ABORT_IMU_CALIBRATION};
    xQueueSend(commands, &abort, 0);
    owner.tick();
    assert(owner.snapshot.last_cmd_result == SensorCommandResult::ABORTED);
    delete q;
    delete commands;
  }
  fake_imu_begin_ok = true;
}
int main() {
  calibration_handshake();
  abort_gate();
  motion_and_guidance();
  quality_and_telemetry();
  open_space_racing_fallback();
  recovery_backoff_and_grace();
  failed_primary_keeps_group_isolated();
  immediate_publication_and_raw_retention();
  owner_budget_and_calibration();
  failed_initialization_budget();
  std::cout << "sensor policy, owner recovery, calibration, and telemetry "
               "tests passed\n";
}
