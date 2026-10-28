#include "ble/ble.h"
#include "config.h"
#include "control/ema.h"
#include "control/pid.h"
#include "control/sensor_guidance.h"
#include "control/sensor_motion_gate.h"
#include "control/slew_rate_limiter.h"
#include "drive_output.h"
#include "driver/RZ7886Driver.h"
#include "prefs/prefs.h"
#include "protocol.h"
#include "sensors/tof_bus.h"
#include "state.h"
#include "telemetry.h"
#include "util.h"
#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>
#include <esp_wifi.h>
RZ7886Driver drive_left(pin::left_bi, pin::left_fi);
RZ7886Driver drive_right(pin::right_bi, pin::right_fi);
RobotState state = {};

static SlewRateLimiter throttle_limiter(config::throttle_slew_rate_accel,
                                        config::throttle_slew_rate_decel);

static EMAFilter left_sensor_filter(config::sensor_ema_alpha);
static EMAFilter right_sensor_filter(config::sensor_ema_alpha);

static float prev_error = 0.0f;
static EMAFilter d_error_filter(config::derivative_filter_alpha);

static sensor_policy::ToFBus sensor_bus(Wire, pin::sda, pin::scl,
                                        config::tof_min_mm, config::tof_max_mm,
                                        config::tof_timing_budget_us);
static float ir_left_adc = 0.0f, ir_right_adc = 0.0f;
static DriveOutput drive_output(config::wheel_slew_rate_accel,
                                config::wheel_slew_rate_decel,
                                config::motor_min_throttle);
static bool recovery_stopping = false;
static bool was_inhibited = true;
static bool seed_pid = true;

static uint32_t countdown_start_ms = 0;
static uint32_t obstacle_start_us = 0;
static bool obstacle_detected = false;
static RunningState prev_running_state = RunningState::IDLE;
static float reverse_turn_steering = 0.0f;

static void init_espnow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(espnow_wifi_channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] Initialization failed!");
    return;
  }

  esp_now_peer_info_t peer_info = {};
  memcpy(peer_info.peer_addr, broadcast_address, 6);
  peer_info.encrypt = false;

  if (esp_now_add_peer(&peer_info) != ESP_OK) {
    Serial.println("[ESP-NOW] Failed to add broadcast peer!");
  } else {
    Serial.println("[ESP-NOW] Initialized and broadcast peer added");
  }
}

static void send_telemetry() {
  PacketHeader header = {
      .magic = message_magic,
      .type = PacketType::TELEMETRY,
      .robot_id = RobotId::BANANA,
      .length = sizeof(TelemetryPacket),
  };

  uint8_t flags = 0;
  if (obstacle_detected) {
    flags |= (1 << 0);
  }
  if (ble_is_connected()) {
    flags |= (1 << 1);
  }

  TelemetryPacket pkt = make_telemetry(state, flags, false);

  uint8_t buffer[sizeof(PacketHeader) + sizeof(TelemetryPacket)];
  memcpy(buffer, &header, sizeof(PacketHeader));
  memcpy(buffer + sizeof(PacketHeader), &pkt, sizeof(TelemetryPacket));

  esp_now_send(broadcast_address, buffer, sizeof(buffer));
}

void setup() {
  load_robot_prefs(state);

  Serial.begin(115200);

  drive_left.init();
  drive_right.init();

  pinMode(pin::left_sensor, INPUT);
  pinMode(pin::right_sensor, INPUT);

  for (unsigned i = 0; i < 7; ++i)
    sensor_policy::distances(state, i)->health = SensorHealth::ABSENT;
  state.imu.health = SensorHealth::ABSENT;
  sensor_bus.count = 1;
  sensor_bus.devices[0].address = 0x30;
  drive_left.stop();
  drive_right.stop();
  sensor_bus
      .begin(); // Initialization runs only after zero outputs are written.

  init_ble("banana");
  init_espnow();

  state.running_state = RunningState::COUNTDOWN;
  countdown_start_ms = millis();
  prev_running_state = RunningState::COUNTDOWN;
}

static uint32_t last_loop_us = 0;
constexpr uint32_t loop_interval_us = 1000000 / 100; // 100 Hz
constexpr float delta_time = static_cast<float>(loop_interval_us) / 1000000.0f;

static uint32_t last_telemetry_us = 0;
constexpr uint32_t telemetry_interval_us = 1000000 / 30; // 30 Hz

static inline uint16_t read_clamped_sensor(uint8_t pin) {
  uint16_t raw = analogRead(pin);
  if (raw < config::sensor_min_raw) {
    return config::sensor_min_raw;
  }
  if (raw > config::sensor_max_raw) {
    return config::sensor_max_raw;
  }
  return raw;
}

void loop() {
  state.current_time_ms = millis();
  state.current_time_us = micros();

  if (state.current_time_us - last_loop_us >= loop_interval_us) {
    last_loop_us += loop_interval_us;
    if (state.current_time_us - last_loop_us >= loop_interval_us) {
      last_loop_us = state.current_time_us;
    }

    sensor_bus.poll();
    state.dist_front = sensor_bus.devices[0].data;
    state.current_time_us = micros();
    state.current_time_ms = millis();
    sensor_policy::expire_all(state, state.current_time_us);
    if (sensor_bus.recovery_due())
      recovery_stopping = true;
    bool required_ready =
        sensor_policy::required(state, state.current_time_us, false);
    bool inhibited = !required_ready || recovery_stopping;

    // Read Sharp analog IR distance sensors and clamp to configured range,
    // smoothed with EMA filter
    uint16_t raw_left = read_clamped_sensor(pin::left_sensor);
    uint16_t raw_right = read_clamped_sensor(pin::right_sensor);
    ir_left_adc = left_sensor_filter.update(static_cast<float>(raw_left));
    ir_right_adc = right_sensor_filter.update(static_cast<float>(raw_right));

    bool skip_control = sensor_motion_inhibit(state, inhibited);
    if (skip_control || (!inhibited && was_inhibited)) {
      obstacle_detected = false;
      obstacle_start_us = 0;
      reset_pid(state.steering_pid);
      seed_pid = true;
      d_error_filter.reset();
    }
    was_inhibited = inhibited;

    // Handle state transition edge triggers (e.g., triggered via BLE)
    if (state.running_state != prev_running_state) {
      if (state.running_state == RunningState::COUNTDOWN) {
        countdown_start_ms = state.current_time_ms;
      } else if (state.running_state == RunningState::IDLE) {
        reset_pid(state.steering_pid);
        throttle_limiter.reset(0.0f);
        drive_output.reset();
        left_sensor_filter.reset();
        right_sensor_filter.reset();
        d_error_filter.reset();
        prev_error = 0.0f;
        obstacle_detected = false;
        obstacle_start_us = 0;
      }
      prev_running_state = state.running_state;
    }

    // Main State Machine
    if (!skip_control)
      switch (state.running_state) {
      case RunningState::IDLE:
        state.target_throttle = 0.0f;
        state.current_throttle = 0.0f;
        state.target_steering = 0.0f;
        state.current_steering = 0.0f;
        drive_left.stop();
        drive_right.stop();
        break;

      case RunningState::COUNTDOWN:
        state.target_throttle = 0.0f;
        state.target_steering = 0.0f;
        if (state.current_time_ms - countdown_start_ms >=
                config::start_delay_ms &&
            required_ready && !recovery_stopping) {
          state.running_state = RunningState::RUNNING;
          reset_pid(state.steering_pid);
          d_error_filter.reset();
          prev_error = 0.0f;
          obstacle_detected = false;
          obstacle_start_us = 0;
        }
        break;

      case RunningState::RUNNING: {
        // Front obstacle detection (VL53L0X: lower mm = closer) with debounce
        bool obstacle = (state.dist_reverse > 0 &&
                         state.dist_front.distance_mm < state.dist_reverse);
        if (obstacle) {
          if (!obstacle_detected) {
            obstacle_detected = true;
            obstacle_start_us = state.current_time_us;
          } else if (state.current_time_us - obstacle_start_us >=
                     static_cast<uint32_t>(config::reverse_trigger_delay_ms) *
                         1000) {
            state.running_state = RunningState::REVERSE;
            state.maneuver_start_us = state.current_time_us;
            obstacle_detected = false;
            obstacle_start_us = 0;

            // Latch turn direction for REVERSE and REVERSE_TURN phases:
            // Sharp IR sensors: higher value = closer wall / obstacle.
            // If left wall is closer (dist_left >= dist_right), turn right
            // (+steer). Otherwise turn left (-steer). Using '>=' ensures
            // head-on collisions (dist_left == dist_right) always pick a turn
            // direction rather than oscillating straight back and forth.
            reverse_turn_steering = (ir_left_adc >= ir_right_adc)
                                        ? config::reverse_turn_steer
                                        : -config::reverse_turn_steer;

            // Differential drive: same steering sign while reversing swings
            // the nose away from the wall, so we back off and turn at once.
            state.target_throttle = -state.reverse_throttle;
            state.target_steering =
                reverse_turn_steering * config::reverse_steer_factor;
            break;
          }
        } else {
          obstacle_detected = false;
          obstacle_start_us = 0;
        }

        // 1. Side centering guidance from Sharp IR sensors (higher ADC = closer
        // wall) Steer purely from the +/-45 deg Sharp sensors. This prevents
        // ramps, hills, and floor elevation changes from being mistaken for
        // walls.
        float raw_diff = ir_left_adc - ir_right_adc;
        float steer_side =
            constrain(raw_diff / config::side_turn_range, -1.0f, 1.0f);

        // 2. Steering error and low-pass filtered derivative (pure PD)
        float error =
            constrain(steer_side + state.steering_pid.offset, -1.0f, 1.0f);
        if (seed_pid) {
          prev_error = error;
          d_error_filter.reset();
          seed_pid = false;
        }
        float raw_d_error = (error - prev_error) / delta_time;
        prev_error = error;
        float d_error = d_error_filter.update(raw_d_error);

        // Gentle progressive bite on larger errors to pull the robot away from
        // the outer wall in curves
        float error_mag = fabs(error);
        float progressive_error = error * (1.0f + 0.35f * error_mag);

        state.steering_pid.error = error;
        state.steering_pid.derivative = d_error;
        state.steering_pid.integral = 0.0f;

        // 3. Target steering (tunable via BLE k command: Kp and Kd)
        state.target_steering =
            constrain(state.steering_pid.Kp * progressive_error +
                          state.steering_pid.Kd * d_error,
                      -1.0f, 1.0f);

        // 4. Throttle control:
        // Only engage straight_throttle when running straight with clear path
        // ahead
        bool is_straight = (fabs(state.target_steering) < 0.15f);
        float throttle =
            (state.dist_far > 0 &&
             state.dist_front.distance_mm > state.dist_far && is_straight)
                ? state.straight_throttle
                : state.base_throttle;

        // Active steering slowdown: slows down in corners to tighten turning
        // radius, but maintains full climbing drive when running straight up
        // hills/ramps.
        float steer_slowdown = 1.0f - 0.35f * fabs(state.target_steering);
        // Minimum motor drive is handled by DriveOutput's deadband mapping;
        // clamping here as well would apply it twice.
        state.target_throttle = throttle * steer_slowdown;
        break;
      }

      case RunningState::REVERSE: {
        state.target_throttle = -state.reverse_throttle;
        // Steer while backing up so the nose swings away from the obstacle
        state.target_steering =
            reverse_turn_steering * config::reverse_steer_factor;

        if (state.current_time_us - state.maneuver_start_us >=
            static_cast<uint32_t>(config::reverse_duration_ms) * 1000) {
          state.running_state = RunningState::REVERSE_TURN;
          state.maneuver_start_us = state.current_time_us;
          state.target_throttle = 0.0f;
          state.target_steering = reverse_turn_steering;
        }
        break;
      }

      case RunningState::REVERSE_TURN: {
        state.target_throttle = 0.0f; // Pivot in place to change heading
        state.target_steering = reverse_turn_steering;

        if (state.current_time_us - state.maneuver_start_us >=
            static_cast<uint32_t>(config::reverse_turn_duration_ms) * 1000) {
          reset_pid(state.steering_pid);
          d_error_filter.reset();
          prev_error = 0.0f;
          obstacle_detected = false;
          obstacle_start_us = 0;
          state.running_state = RunningState::RUNNING;
        }
        break;
      }
      case RunningState::WRONG_WAY_RECOVERY:
        state.running_state = RunningState::RUNNING;
        state.target_throttle = 0.0f;
        state.target_steering = 0.0f;
        break;
      }

    // Actuator outputs and slew rate limiting (when active)
    float left_throttle = 0.0f;
    float right_throttle = 0.0f;
    if (state.running_state != RunningState::IDLE) {
      state.current_throttle =
          throttle_limiter.update(state.target_throttle, delta_time);
      // No steering slew: DriveOutput's per-wheel slew already bounds it.
      state.current_steering = state.target_steering;

      auto outputs = drive_output.update(
          state.current_throttle, state.current_steering,
          state.running_state != RunningState::RUNNING,
          inhibited || state.running_state == RunningState::COUNTDOWN,
          delta_time);
      left_throttle = outputs.left;
      right_throttle = outputs.right;

      drive_left.write(left_throttle);
      drive_right.write(right_throttle);
    }

    if (recovery_stopping && left_throttle == 0.0f && right_throttle == 0.0f &&
        sensor_bus.recover())
      recovery_stopping = false;

    static uint32_t last_debug_ms = 0;
    if (state.current_time_ms - last_debug_ms >= 250) {
      last_debug_ms = state.current_time_ms;
      Serial.printf(
          "[RUN] st=%s f=%u l=%u r=%u err=%.2f de=%.2f throt=%.2f "
          "steer=%.2f lthrot=%.2f rthrot=%.2f\n",
          running_state_to_str(state.running_state),
          state.dist_front.distance_mm, static_cast<unsigned>(ir_left_adc),
          static_cast<unsigned>(ir_right_adc), state.steering_pid.error,
          state.steering_pid.derivative, state.current_throttle,
          state.current_steering, left_throttle, right_throttle);
    }
  }

#if 1
  if (state.current_time_us - last_telemetry_us >= telemetry_interval_us) {
    last_telemetry_us += telemetry_interval_us;
    if (state.current_time_us - last_telemetry_us >= telemetry_interval_us) {
      last_telemetry_us = state.current_time_us;
    }
    send_telemetry();
  }
#endif
}

void sensor_task_diagnostics() {
  notify(
      "[SENSORS] control-loop owner; recovery waits for zero motor outputs\n");
}
