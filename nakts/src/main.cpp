#include <Arduino.h>
#include <VL53L0X.h>
#include <Wire.h>

#include "ble/ble.h"
#include "config.h"
#include "control/pid.h"
#include "control/slew_rate_limiter.h"
#include "driver/RZ7886Driver.h"
#include "driver/servo.h"
#include "prefs/prefs.h"
#include "state.h"

RobotState state = {};

RZ7886Driver drive_motor(pin::bi, pin::fi);
Servo steer_servo(pin::servo);

static SlewRateLimiter throttle_limiter(config::throttle_slew_rate_accel,
                                        config::throttle_slew_rate_decel);
static SlewRateLimiter steering_limiter(config::steering_slew_rate,
                                        config::steering_slew_rate);

VL53L0X sensor_front;
VL53L0X sensor_left;
VL53L0X sensor_right;

static uint32_t countdown_start_ms = 0;
static uint32_t obstacle_start_us = 0;
static bool obstacle_detected = false;
static RunningState prev_running_state = RunningState::IDLE;
static float reverse_turn_steering = 0.0f;

void init_tof() {
  pinMode(pin::xshut_left, OUTPUT);
  pinMode(pin::xshut_right, OUTPUT);

  digitalWrite(pin::xshut_left, LOW);
  digitalWrite(pin::xshut_right, LOW);

  delay(10);

  sensor_front.setAddress(0x30);
  sensor_front.init();
  sensor_front.setTimeout(100);
  sensor_front.setMeasurementTimingBudget(config::tof_timing_budget_us);
  sensor_front.startContinuous();

  delay(5);

  pinMode(pin::xshut_left, INPUT);
  delay(10);
  sensor_left.setAddress(0x32);
  sensor_left.init();
  sensor_left.setTimeout(100);
  sensor_left.setMeasurementTimingBudget(config::tof_timing_budget_us);
  sensor_left.startContinuous();

  delay(5);

  pinMode(pin::xshut_right, INPUT);
  delay(10);
  sensor_right.setAddress(0x31);
  sensor_right.init();
  sensor_right.setTimeout(100);
  sensor_right.setMeasurementTimingBudget(config::tof_timing_budget_us);
  sensor_right.startContinuous();

  delay(10);
}

void setup() {
  load_robot_prefs(state);

  Wire.begin(pin::sda, pin::scl, 400000);
  Serial.begin(115200);

  drive_motor.init();
  steer_servo.init();

  init_tof();

  init_ble("nakts");

  state.running_state = RunningState::COUNTDOWN;
  countdown_start_ms = millis();
  prev_running_state = state.running_state;

  state.track_direction = TrackDirection::CLOCKWISE;
}

static uint32_t last_loop_us = 0;
constexpr uint32_t loop_interval_us = 1000000 / 100; // 100 Hz
constexpr float delta_time = static_cast<float>(loop_interval_us) / 1000000.0f;

bool is_tof_data_ready(VL53L0X &sensor) {
  return (sensor.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) != 0;
}

uint16_t tof_read_distance(VL53L0X &sensor) {
  uint16_t range = sensor.readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
  sensor.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
  return range;
}

static inline uint16_t read_clamped_sensor(VL53L0X &sensor) {
  uint16_t raw = tof_read_distance(sensor);
  if (sensor.timeoutOccurred() || raw > config::sensor_max_raw) {
    return config::sensor_max_raw;
  }
  if (raw < config::sensor_min_raw) {
    return config::sensor_min_raw;
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

    if (is_tof_data_ready(sensor_front)) {
      state.dist_front = read_clamped_sensor(sensor_front);
    }
    if (is_tof_data_ready(sensor_left)) {
      state.dist_left = read_clamped_sensor(sensor_left);
    }
    if (is_tof_data_ready(sensor_right)) {
      state.dist_right = read_clamped_sensor(sensor_right);
    }

    // Handle state transition edge triggers (e.g., triggered via BLE)
    if (state.running_state != prev_running_state) {
      if (state.running_state == RunningState::COUNTDOWN) {
        countdown_start_ms = state.current_time_ms;
      } else if (state.running_state == RunningState::IDLE) {
        reset_pid(state.steering_pid);
        throttle_limiter.reset(0.0f);
        steering_limiter.reset(0.0f);
        obstacle_detected = false;
      } else if (state.running_state == RunningState::REVERSE_TURN) {
        // Reverse-turn to swing nose towards the open side (no IMU dependency)
        reverse_turn_steering =
            (state.dist_right >= state.dist_left) ? -1.0f : 1.0f;
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
      // Front obstacle detection
      uint16_t rev_dist = (state.dist_reverse > 0)
                              ? state.dist_reverse
                              : config::default_dist_reverse;
      bool obstacle = (state.dist_front < rev_dist);
      if (obstacle) {
        if (!obstacle_detected) {
          obstacle_detected = true;
          obstacle_start_us = state.current_time_us;
        } else if (state.current_time_us - obstacle_start_us >=
                   static_cast<uint32_t>(config::reverse_trigger_delay_ms) *
                       1000) {
          state.running_state = RunningState::REVERSE;
          state.maneuver_start_us = state.current_time_us;
          reverse_turn_steering =
              (state.dist_right >= state.dist_left) ? -1.0f : 1.0f;
          obstacle_detected = false;
          break;
        }
      } else {
        obstacle_detected = false;
      }

      // 1. Lateral centering (positive = steer right away from left wall)
      float raw_diff = static_cast<float>(state.dist_right) -
                       static_cast<float>(state.dist_left);
      float steer_side =
          constrain(raw_diff / config::side_turn_range, -1.0f, 1.0f);

      // 2. Emergency side wall repulsion
      float steer_repel = 0.0f;
      if (state.dist_left < config::side_danger_dist) {
        steer_repel += static_cast<float>(config::side_danger_dist - state.dist_left) /
                       static_cast<float>(config::side_danger_dist);
      }
      if (state.dist_right < config::side_danger_dist) {
        steer_repel -= static_cast<float>(config::side_danger_dist - state.dist_right) /
                       static_cast<float>(config::side_danger_dist);
      }

      // 3. Front obstacle / corner anticipation
      float front_urgency = 0.0f;
      float turn_dir = (state.dist_right >= state.dist_left) ? 1.0f : -1.0f;

      if (state.dist_front < config::front_turn_dist) {
        front_urgency =
            static_cast<float>(config::front_turn_dist - state.dist_front) /
            static_cast<float>(config::front_turn_dist - config::front_critical_dist);
        front_urgency = constrain(front_urgency, 0.0f, 1.0f);
      }

      // Blend side guidance and front cornering
      float raw_steer_target =
          (1.0f - front_urgency) * constrain(steer_side + steer_repel, -1.0f, 1.0f) +
          (front_urgency * turn_dir);
      raw_steer_target = constrain(raw_steer_target, -1.0f, 1.0f);

      // Steering PID update (includes trim offset)
      state.steering_pid.error = raw_steer_target + state.steering_pid.offset;
      state.target_steering = update_pid(delta_time, state.steering_pid);

      // 4. Throttle control with corner slowing
      float throttle = state.base_throttle;
      if (state.dist_far > 0 && state.dist_front > state.dist_far) {
        throttle = state.straight_throttle;
      }
      // Slow down when entering a corner so front wheels grip and don't understeer
      float corner_speed_factor = 1.0f - 0.45f * front_urgency;
      state.target_throttle = throttle * corner_speed_factor;
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
      state.target_steering = reverse_turn_steering;

      if (state.current_time_us - state.maneuver_start_us >=
          static_cast<uint32_t>(config::reverse_turn_duration_ms) * 1000) {
        reset_pid(state.steering_pid);
        obstacle_detected = false;
        state.running_state = RunningState::RUNNING;
      }
      break;
    case RunningState::WRONG_WAY_RECOVERY: {
      break;
    }
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

    static uint32_t last_debug_ms = 0;
    if (state.current_time_ms - last_debug_ms >= 200) {
      last_debug_ms = state.current_time_ms;
      Serial.printf("[RUN] st=%s f=%u l=%u r=%u err=%.2f str=%.2f thr=%.2f\n",
                    running_state_to_str(state.running_state),
                    state.dist_front, state.dist_left, state.dist_right,
                    state.steering_pid.error, state.current_steering,
                    state.current_throttle);
    }
  }
}