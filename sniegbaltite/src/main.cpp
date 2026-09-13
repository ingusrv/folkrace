#include "ble/ble.h"
#include "config.h"
#include "control/pid.h"
#include "control/slew_rate_limiter.h"
#include "driver/esc.h"
#include "driver/servo.h"
#include "prefs/prefs.h"
#include "state.h"
#include <Arduino.h>
#include <Wire.h>

ESC drive_motor = ESC(pin::esc);
Servo steer_servo = Servo(pin::servo);
RobotState state = {};

static SlewRateLimiter throttle_limiter(config::throttle_slew_rate_accel,
                                        config::throttle_slew_rate_decel);
static SlewRateLimiter steering_limiter(config::steering_slew_rate,
                                        config::steering_slew_rate);

static uint32_t countdown_start_ms = 0;
static uint32_t obstacle_start_us = 0;
static bool obstacle_detected = false;
static RunningState prev_running_state = RunningState::IDLE;

void setup() {
  load_robot_prefs(state);

  Wire.begin(pin::sda, pin::scl, 400000);
  Serial.begin(115200);
  drive_motor.init();
  steer_servo.init();

  pinMode(pin::mid_sensor, INPUT);
  pinMode(pin::left_sensor, INPUT);
  pinMode(pin::right_sensor, INPUT);

  init_ble("sniegbaltite");

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

    // Read Sharp analog IR distance sensors and clamp to configured range
    state.dist_front = read_clamped_sensor(pin::mid_sensor);
    state.dist_left = read_clamped_sensor(pin::left_sensor);
    state.dist_right = read_clamped_sensor(pin::right_sensor);

    // Handle state transition edge triggers (e.g., triggered via BLE)
    if (state.running_state != prev_running_state) {
      if (state.running_state == RunningState::COUNTDOWN) {
        countdown_start_ms = state.current_time_ms;
      } else if (state.running_state == RunningState::IDLE) {
        reset_pid(state.steering_pid);
        throttle_limiter.reset(0.0f);
        steering_limiter.reset(0.0f);
        obstacle_detected = false;
      }
      prev_running_state = state.running_state;
    }

    // Main State Machine
    switch (state.running_state) {
    case RunningState::IDLE:
      state.target_throttle = 0.0f;
      state.current_throttle = 0.0f;
      state.target_steering = 0.0f;
      state.current_steering = 0.0f;
      drive_motor.stop();
      steer_servo.write(config::servo_deg_middle);
      break;

    case RunningState::COUNTDOWN:
      state.target_throttle = 0.0f;
      state.target_steering = 0.0f;
      if (state.current_time_ms - countdown_start_ms >=
          config::start_delay_ms) {
        state.running_state = RunningState::RUNNING;
        reset_pid(state.steering_pid);
        obstacle_detected = false;
      }
      break;

    case RunningState::RUNNING: {
      // Collision detection: Sharp IR sensors output higher values the closer
      // an obstacle is. Transition to REVERSE if dist_front >
      // state.dist_reverse for longer than 1 second.
      bool obstacle =
          (state.dist_reverse > 0 && state.dist_front > state.dist_reverse);
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
          break;
        }
      } else {
        obstacle_detected = false;
      }

      // Throttle control: use straight_throttle if dist_front < dist_far,
      // otherwise base_throttle
      // sharp ir sensors: higher value = closer to obstacle.
      if (state.dist_far > 0 && state.dist_front < state.dist_far) {
        state.target_throttle = state.straight_throttle;
      } else {
        state.target_throttle = state.base_throttle;
      }

      // Steering control via normalized PID error
      float raw_diff = static_cast<float>(state.dist_left) -
                       static_cast<float>(state.dist_right);
      state.steering_pid.error =
          (raw_diff / config::sensor_range) + state.steering_pid.offset;
      state.target_steering = update_pid(delta_time, state.steering_pid);
      break;
    }

    case RunningState::REVERSE:
      state.target_throttle = -state.reverse_throttle;
      state.target_steering = 0.0f;
      if (state.current_time_us - state.maneuver_start_us >=
          static_cast<uint32_t>(config::reverse_duration_ms) * 1000) {
        state.running_state = RunningState::REVERSE_TURN;
        state.maneuver_start_us = state.current_time_us;
      }
      break;

    case RunningState::REVERSE_TURN:
      state.target_throttle = -state.reverse_throttle;
      // Steer away from the closer wall (higher value = closer)
      if (state.dist_left > state.dist_right) {
        state.target_steering = 1.0f; // Left wall is closer, steer right
      } else {
        state.target_steering = -1.0f; // Right wall is closer, steer left
      }

      if (state.current_time_us - state.maneuver_start_us >=
          static_cast<uint32_t>(config::reverse_turn_duration_ms) * 1000) {
        reset_pid(state.steering_pid);
        state.running_state = RunningState::RUNNING;
      }
      break;
    }

    // Actuator outputs and slew rate limiting (when active)
    if (state.running_state != RunningState::IDLE) {
      state.current_throttle =
          throttle_limiter.update(state.target_throttle, delta_time);
      state.current_steering =
          steering_limiter.update(state.target_steering, delta_time);

      int servo_deg;
      if (state.current_steering >= 0.0f) {
        servo_deg = config::servo_deg_middle +
                    static_cast<int>(
                        state.current_steering *
                        (config::servo_deg_right - config::servo_deg_middle));
      } else {
        servo_deg = config::servo_deg_middle +
                    static_cast<int>(
                        state.current_steering *
                        (config::servo_deg_middle - config::servo_deg_left));
      }

      steer_servo.write(servo_deg);
      drive_motor.write(state.current_throttle);
    }
  }

  if (state.current_time_us - last_telemetry_us >= telemetry_interval_us) {
    last_telemetry_us += telemetry_interval_us;
    if (state.current_time_us - last_telemetry_us >= telemetry_interval_us) {
      last_telemetry_us = state.current_time_us;
    }
    // Telemetry omitted for now
  }
}