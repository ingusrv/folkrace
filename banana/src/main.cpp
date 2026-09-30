#include "ble/ble.h"
#include "config.h"
#include "control/ema.h"
#include "control/pid.h"
#include "control/slew_rate_limiter.h"
#include "driver/RZ7886Driver.h"
#include "prefs/prefs.h"
#include "protocol.h"
#include "state.h"
#include "util.h"
#include <Arduino.h>
#include <VL53L0X.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>
#include <esp_wifi.h>

RZ7886Driver drive_left(pin::left_bi, pin::left_fi);
RZ7886Driver drive_right(pin::right_bi, pin::right_fi);
VL53L0X tof_front;
RobotState state = {};

static SlewRateLimiter throttle_limiter(config::throttle_slew_rate_accel,
                                        config::throttle_slew_rate_decel);
static SlewRateLimiter steering_limiter(config::steering_slew_rate,
                                        config::steering_slew_rate);

static EMAFilter left_sensor_filter(config::sensor_ema_alpha);
static EMAFilter right_sensor_filter(config::sensor_ema_alpha);

static float prev_error = 0.0f;
static EMAFilter d_error_filter(config::derivative_filter_alpha);

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

  TelemetryPacket pkt = {
      .timestamp_ms = state.current_time_ms,
      .running_state = state.running_state,
      .flags = flags,
      .dist_front = state.dist_front,
      .dist_right = state.dist_right,
      .dist_left = state.dist_left,
      .dist_side_front_left = 0,
      .dist_side_rear_left = 0,
      .dist_side_front_right = 0,
      .dist_side_rear_right = 0,
      .target_steering = state.target_steering,
      .current_steering = state.current_steering,
      .target_throttle = state.target_throttle,
      .current_throttle = state.current_throttle,
      .pid_error = state.steering_pid.error,
      .pid_integral = state.steering_pid.integral,
      .pid_derivative = state.steering_pid.derivative,
      .accel_x = state.imu.accel_x,
      .accel_y = state.imu.accel_y,
      .accel_z = state.imu.accel_z,
      .gyro_x = state.imu.gyro_x,
      .gyro_y = state.imu.gyro_y,
      .gyro_z = state.imu.gyro_z,
      .roll = state.imu.roll,
      .pitch = state.imu.pitch,
      .yaw = state.imu.yaw,
  };

  uint8_t buffer[sizeof(PacketHeader) + sizeof(TelemetryPacket)];
  memcpy(buffer, &header, sizeof(PacketHeader));
  memcpy(buffer + sizeof(PacketHeader), &pkt, sizeof(TelemetryPacket));

  esp_now_send(broadcast_address, buffer, sizeof(buffer));
}

void setup() {
  load_robot_prefs(state);

  Wire.begin(pin::sda, pin::scl, 400000);
  Serial.begin(115200);

  drive_left.init();
  drive_right.init();

  pinMode(pin::left_sensor, INPUT);
  pinMode(pin::right_sensor, INPUT);

  tof_front.setBus(&Wire);
  if (!tof_front.init()) {
    Serial.println("[ToF] Failed to initialize VL53L0X!");
  }
  tof_front.setTimeout(config::tof_timeout_ms);
  tof_front.setMeasurementTimingBudget(config::tof_timing_budget_us);
  tof_front.startContinuous();

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

bool is_tof_data_ready(VL53L0X &sensor) {
  return (sensor.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) != 0;
}

uint16_t tof_read_distance(VL53L0X &sensor) {
  uint16_t range = sensor.readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
  sensor.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
  return range;
}

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

static inline float apply_motor_deadband(float throttle) {
  if (fabs(throttle) < 0.02f)
    return 0.0f;
  float sign = (throttle > 0.0f) ? 1.0f : -1.0f;
  return sign * (config::motor_min_throttle +
                 (1.0f - config::motor_min_throttle) * fabs(throttle));
}

void loop() {
  state.current_time_ms = millis();
  state.current_time_us = micros();

  if (state.current_time_us - last_loop_us >= loop_interval_us) {
    last_loop_us += loop_interval_us;
    if (state.current_time_us - last_loop_us >= loop_interval_us) {
      last_loop_us = state.current_time_us;
    }

    // Read front ToF sensor non-blocking with I2C transmission error check
    if (is_tof_data_ready(tof_front)) {
      uint16_t tof_dist = tof_read_distance(tof_front);
      state.dist_front =
          constrain(tof_dist, config::tof_min_mm, config::tof_max_mm);
    } else if (tof_front.last_status != 0) {
      // Re-initialize only on actual I2C transmission error
      tof_front.init();
      tof_front.setTimeout(config::tof_timeout_ms);
      tof_front.setMeasurementTimingBudget(config::tof_timing_budget_us);
      tof_front.startContinuous();
      state.dist_front = config::tof_max_mm;
    }

    // Read Sharp analog IR distance sensors and clamp to configured range,
    // smoothed with EMA filter
    uint16_t raw_left = read_clamped_sensor(pin::left_sensor);
    uint16_t raw_right = read_clamped_sensor(pin::right_sensor);
    float filtered_left =
        left_sensor_filter.update(static_cast<float>(raw_left));
    float filtered_right =
        right_sensor_filter.update(static_cast<float>(raw_right));
    state.dist_left = static_cast<uint16_t>(filtered_left);
    state.dist_right = static_cast<uint16_t>(filtered_right);

    // Handle state transition edge triggers (e.g., triggered via BLE)
    if (state.running_state != prev_running_state) {
      if (state.running_state == RunningState::COUNTDOWN) {
        countdown_start_ms = state.current_time_ms;
      } else if (state.running_state == RunningState::IDLE) {
        reset_pid(state.steering_pid);
        throttle_limiter.reset(0.0f);
        steering_limiter.reset(0.0f);
        left_sensor_filter.reset();
        right_sensor_filter.reset();
        d_error_filter.reset();
        prev_error = 0.0f;
        obstacle_detected = false;
        obstacle_start_us = 0;
      } else if (state.running_state == RunningState::REVERSE) {
        throttle_limiter.reset(-state.reverse_throttle);
        steering_limiter.reset(0.0f);
      } else if (state.running_state == RunningState::REVERSE_TURN) {
        throttle_limiter.reset(0.0f);
        steering_limiter.reset(reverse_turn_steering);
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
      drive_left.stop();
      drive_right.stop();
      break;

    case RunningState::COUNTDOWN:
      state.target_throttle = 0.0f;
      state.target_steering = 0.0f;
      if (state.current_time_ms - countdown_start_ms >=
          config::start_delay_ms) {
        state.running_state = RunningState::RUNNING;
        reset_pid(state.steering_pid);
        throttle_limiter.reset(0.0f);
        steering_limiter.reset(0.0f);
        d_error_filter.reset();
        prev_error = 0.0f;
        obstacle_detected = false;
        obstacle_start_us = 0;
      }
      break;

    case RunningState::RUNNING: {
      // Front obstacle detection (VL53L0X: lower mm = closer) with debounce
      bool obstacle =
          (state.dist_reverse > 0 && state.dist_front < state.dist_reverse);
      if (obstacle) {
        if (!obstacle_detected) {
          obstacle_detected = true;
          obstacle_start_us = state.current_time_us;
        } else if (state.current_time_us - obstacle_start_us >=
                   static_cast<uint32_t>(config::reverse_trigger_delay_ms) *
                       1000) {
          state.running_state = RunningState::REVERSE;
          state.maneuver_start_us = state.current_time_us;
          state.target_throttle = -state.reverse_throttle;
          state.target_steering = 0.0f;
          throttle_limiter.reset(-state.reverse_throttle);
          steering_limiter.reset(0.0f);
          obstacle_detected = false;
          obstacle_start_us = 0;

          // Latch pivot turn direction for REVERSE_TURN phase:
          // Sharp IR sensors: higher value = closer wall / obstacle.
          // If left wall is closer (dist_left >= dist_right), turn right
          // (+steer). Otherwise turn left (-steer). Using '>=' ensures head-on
          // collisions (dist_left == dist_right) always pick a turn direction
          // rather than oscillating straight back and forth.
          reverse_turn_steering = (filtered_left >= filtered_right)
                                      ? config::reverse_turn_steer
                                      : -config::reverse_turn_steer;
          break;
        }
      } else {
        obstacle_detected = false;
        obstacle_start_us = 0;
      }

      // 1. Side centering guidance from Sharp IR sensors (higher ADC = closer wall)
      // Steer purely from the +/-45 deg Sharp sensors.
      // This prevents ramps, hills, and floor elevation changes from being mistaken for walls.
      float raw_diff = filtered_left - filtered_right;
      float steer_side =
          constrain(raw_diff / config::side_turn_range, -1.0f, 1.0f);

      // 2. Steering error and low-pass filtered derivative (pure PD)
      float error =
          constrain(steer_side + state.steering_pid.offset, -1.0f, 1.0f);
      float raw_d_error = (error - prev_error) / delta_time;
      prev_error = error;
      float d_error = d_error_filter.update(raw_d_error);

      // Gentle progressive bite on larger errors to pull the robot away from the outer wall in curves
      float error_mag = fabs(error);
      float progressive_error = error * (1.0f + 0.35f * error_mag);

      state.steering_pid.error = error;
      state.steering_pid.derivative = d_error;
      state.steering_pid.integral = 0.0f;

      // 3. Target steering (tunable via BLE k command: Kp and Kd)
      state.target_steering = constrain(
          state.steering_pid.Kp * progressive_error + state.steering_pid.Kd * d_error,
          -1.0f, 1.0f);

      // 4. Throttle control:
      // Only engage straight_throttle when running straight with clear path ahead
      bool is_straight = (fabs(state.target_steering) < 0.15f);
      float throttle = (state.dist_far > 0 && state.dist_front > state.dist_far && is_straight)
                           ? state.straight_throttle
                           : state.base_throttle;

      // Active steering slowdown: slows down in corners to tighten turning radius,
      // but maintains full climbing drive when running straight up hills/ramps.
      float steer_slowdown = 1.0f - 0.35f * fabs(state.target_steering);
      state.target_throttle = throttle * steer_slowdown;
      state.target_throttle = std::max(state.target_throttle, config::motor_min_throttle);
      break;
    }

    case RunningState::REVERSE: {
      state.target_throttle = -state.reverse_throttle;
      state.target_steering = 0.0f; // Pure straight reverse to clear obstacle

      if (state.current_time_us - state.maneuver_start_us >=
          static_cast<uint32_t>(config::reverse_duration_ms) * 1000) {
        state.running_state = RunningState::REVERSE_TURN;
        state.maneuver_start_us = state.current_time_us;
        state.target_throttle = 0.0f;
        throttle_limiter.reset(0.0f);
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
        steering_limiter.reset(0.0f);
        d_error_filter.reset();
        prev_error = 0.0f;
        obstacle_detected = false;
        obstacle_start_us = 0;
        state.running_state = RunningState::RUNNING;
        state.dist_front = config::tof_max_mm; // Clear stale ToF reading
      }
      break;
    }
    }

    // Actuator outputs and slew rate limiting (when active)
    float left_throttle = 0.0f;
    float right_throttle = 0.0f;
    if (state.running_state != RunningState::IDLE) {
      state.current_throttle =
          throttle_limiter.update(state.target_throttle, delta_time);
      state.current_steering =
          steering_limiter.update(state.target_steering, delta_time);

      // Differential drive motor mixing:
      // During forward driving (RUNNING), clamp the inside wheel at 0.0f minimum.
      // Reversing the inside wheel at speed causes violent snap oversteer and yaw oscillation!
      float left_raw = state.current_throttle + state.current_steering;
      float right_raw = state.current_throttle - state.current_steering;

      if (state.running_state == RunningState::RUNNING) {
        left_throttle = constrain(left_raw, 0.0f, 1.0f);
        right_throttle = constrain(right_raw, 0.0f, 1.0f);
      } else {
        // Allow reversing during REVERSE and REVERSE_TURN recovery maneuvers
        left_throttle = constrain(left_raw, -1.0f, 1.0f);
        right_throttle = constrain(right_raw, -1.0f, 1.0f);
      }

      left_throttle = apply_motor_deadband(left_throttle);
      right_throttle = apply_motor_deadband(right_throttle);

      drive_left.write(left_throttle);
      drive_right.write(right_throttle);
    }

    static uint32_t last_debug_ms = 0;
    if (state.current_time_ms - last_debug_ms >= 250) {
      last_debug_ms = state.current_time_ms;
      Serial.printf("[RUN] st=%s f=%u l=%u r=%u err=%.2f de=%.2f throt=%.2f "
                    "steer=%.2f lthrot=%.2f rthrot=%.2f\n",
                    running_state_to_str(state.running_state), state.dist_front,
                    state.dist_left, state.dist_right, state.steering_pid.error,
                    state.steering_pid.derivative, state.current_throttle,
                    state.current_steering, left_throttle, right_throttle);
    }
  }

#if 0
  if (state.current_time_us - last_telemetry_us >= telemetry_interval_us) {
    last_telemetry_us += telemetry_interval_us;
    if (state.current_time_us - last_telemetry_us >= telemetry_interval_us) {
      last_telemetry_us = state.current_time_us;
    }
    send_telemetry();
  }
#endif
}