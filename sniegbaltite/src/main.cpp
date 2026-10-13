#include "ble/ble.h"
#include "config.h"
#include "control/pid.h"
#include "control/sensor_guidance.h"
#include "control/sensor_led.h"
#include "control/sensor_motion_gate.h"
#include "control/sensor_startup.h"
#include "control/slew_rate_limiter.h"
#include "driver/esc.h"
#include "driver/servo.h"
#include "obstacle_navigation.h"
#include "prefs/prefs.h"
#include "protocol.h"
#include "sensors.h"
#include "sensors/health.h"
#include "state.h"
#include "telemetry.h"
#include "util.h"
#include <Arduino.h>
#include <WiFi.h>
#include <atomic>
#include <esp_now.h>
#include <esp_wifi.h>

ESC drive_motor = ESC(pin::esc);
Servo steer_servo = Servo(pin::servo);
static MainSensorSnapshot main_snapshot;
static SideSensorSnapshot side_snapshot;
static SensorStartup startup;
static SensorLED sensor_led;
static bool sensors_started = false;
static bool was_inhibited = true;
static bool seed_pid = true;
static uint32_t orientation_generation = 0;
static std::atomic<uint32_t> control_max_period_us{0};
RobotState state = {};

static SlewRateLimiter throttle_limiter(config::throttle_slew_rate_accel,
                                        config::throttle_slew_rate_decel);
static SlewRateLimiter steering_limiter(config::steering_slew_rate,
                                        config::steering_slew_rate);

static uint32_t countdown_start_ms = 0;
static uint32_t obstacle_start_us = 0;
static bool obstacle_detected = false;
static RunningState prev_running_state = RunningState::IDLE;
static ObstacleAvoidance obstacle_avoidance;
static ReverseRecovery reverse_recovery;
static bool resume_centering = false;

static float max_forward_yaw = 0.0f;
static float min_forward_yaw = 0.0f;
static uint32_t wrong_way_start_us = 0;
static uint8_t recovery_phase = 1;
static uint32_t recovery_phase_start_us = 0;

static float flat_pitch = 0.0f;
static bool is_on_slope = false;
static uint32_t last_slope_us = 0;

static void init_espnow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(espnow_wifi_channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    notify("[ESP-NOW] initialization failed\n");
    return;
  }

  esp_now_peer_info_t peer_info = {};
  memcpy(peer_info.peer_addr, broadcast_address, 6);
  peer_info.encrypt = false;

  if (esp_now_add_peer(&peer_info) != ESP_OK) {
    notify("[ESP-NOW] adding broadcast peer failed\n");
  }
}

void setup() {
  load_robot_prefs(state);

  drive_motor.init();
  steer_servo.init();
  bool led_started = sensor_led.begin(
      pin::rgb_led_pin, config::rgb_led_brightness, config::imu_offline_led_r,
      config::imu_offline_led_g, config::imu_offline_led_b,
      config::imu_offline_blink_interval_ms);
  sensors_started = sensors::init() && led_started;

  pinMode(pin::direction_switch, INPUT_PULLUP);

  // INPUT_PULLUP: on/LOW is clockwise; off/HIGH is counterclockwise.
  state.track_direction = (digitalRead(pin::direction_switch) == LOW)
                              ? TrackDirection::CLOCKWISE
                              : TrackDirection::COUNTER_CLOCKWISE;

  init_ble("sniegbaltite");
  init_espnow();

  state.running_state = RunningState::COUNTDOWN;
  countdown_start_ms = millis();
  startup.enter(countdown_start_ms);
  prev_running_state = RunningState::COUNTDOWN;
}

static uint32_t last_loop_us = 0;
constexpr uint32_t loop_interval_us = 1000000 / 100; // 100 Hz
constexpr float delta_time = static_cast<float>(loop_interval_us) / 1000000.0f;

static uint32_t last_telemetry_us = 0;
constexpr uint32_t telemetry_interval_us = 1000000 / 30; // 30 Hz

static void send_telemetry() {
  PacketHeader header = {
      .magic = message_magic,
      .type = PacketType::TELEMETRY,
      .robot_id = RobotId::SNIEGBALTITE,
      .length = sizeof(TelemetryPacket),
  };

  uint8_t flags = 0;
  if (obstacle_detected) {
    flags |= (1 << 0);
  }
  if (ble_is_connected()) {
    flags |= (1 << 1);
  }
  if (state.track_direction == TrackDirection::CLOCKWISE) {
    flags |= (1 << 2);
  }
  if (is_on_slope) {
    flags |= (1 << 4);
  }

  TelemetryPacket pkt = make_telemetry(state, flags);

  uint8_t buffer[sizeof(PacketHeader) + sizeof(TelemetryPacket)];
  memcpy(buffer, &header, sizeof(PacketHeader));
  memcpy(buffer + sizeof(PacketHeader), &pkt, sizeof(TelemetryPacket));

  esp_now_send(broadcast_address, buffer, sizeof(buffer));
}

void loop() {
  state.current_time_ms = millis();
  state.current_time_us = micros();

  if (state.current_time_us - last_loop_us >= loop_interval_us) {
    last_loop_us += loop_interval_us;
    if (state.current_time_us - last_loop_us >= loop_interval_us) {
      last_loop_us = state.current_time_us;
    }

    static uint32_t previous_control_us = 0;
    if (previous_control_us) {
      uint32_t period_us = state.current_time_us - previous_control_us;
      control_max_period_us.store(
          std::max(control_max_period_us.load(), period_us));
    }
    previous_control_us = state.current_time_us;
    sensors::get_main_snapshot(main_snapshot);
    sensors::get_side_snapshot(side_snapshot);
    state.dist_front = main_snapshot.dist_front;
    state.dist_left = main_snapshot.dist_left;
    state.dist_right = main_snapshot.dist_right;
    state.dist_side_front_left = side_snapshot.dist_side_front_left;
    state.dist_side_rear_left = side_snapshot.dist_side_rear_left;
    state.dist_side_front_right = side_snapshot.dist_side_front_right;
    state.dist_side_rear_right = side_snapshot.dist_side_rear_right;
    state.imu = main_snapshot.imu;
    // Must follow both queue copies: a concurrent sample cannot be in the
    // future.
    state.current_time_us = micros();
    state.current_time_ms = millis();
    sensor_policy::expire_all(state, state.current_time_us);
    bool required_ready = sensors_started &&
                          sensor_policy::required(state, state.current_time_us);
    bool imu_ready = startup.imu_enabled(main_snapshot) &&
                     is_imu_available(state.imu, state.current_time_us,
                                      config::sensor_freshness_imu_us);
    if (imu_ready &&
        orientation_generation != main_snapshot.orientation_generation) {
      orientation_generation = main_snapshot.orientation_generation;
      max_forward_yaw = min_forward_yaw = state.imu.measurement.yaw;
      wrong_way_start_us = 0;
      last_slope_us = 0;
      is_on_slope = false;
      reset_pid(state.steering_pid);
      seed_pid = true;
      if (state.running_state == RunningState::WRONG_WAY_RECOVERY)
        state.running_state = RunningState::RUNNING;
    }
    if (!imu_ready) {
      wrong_way_start_us = 0;
      last_slope_us = 0;
      is_on_slope = false;
      if (state.running_state == RunningState::WRONG_WAY_RECOVERY)
        state.running_state = RunningState::RUNNING;
    }
    if (main_snapshot.imu_calib_valid)
      flat_pitch = main_snapshot.imu_calib_flat_pitch;

    // Slope / hill detection based on pitch deviation from flat ground
    float pitch_dev = fabs(state.imu.measurement.pitch - flat_pitch);
    bool pitch_on_slope =
        imu_ready && (pitch_dev >= config::slope_pitch_threshold_deg);
    if (pitch_on_slope) {
      last_slope_us = state.current_time_us;
    }
    is_on_slope = pitch_on_slope ||
                  (last_slope_us > 0 &&
                   (state.current_time_us - last_slope_us <
                    static_cast<uint32_t>(config::slope_hold_ms) * 1000));

    // Read toggle switch position (live reading with transition detection)
    TrackDirection current_direction =
        (digitalRead(pin::direction_switch) == LOW)
            ? TrackDirection::CLOCKWISE
            : TrackDirection::COUNTER_CLOCKWISE;
    if (current_direction != state.track_direction) {
      state.track_direction = current_direction;
      max_forward_yaw = state.imu.measurement.yaw;
      min_forward_yaw = state.imu.measurement.yaw;
      wrong_way_start_us = 0;
      reset_pid(state.steering_pid);
      seed_pid = true;
      if (state.running_state == RunningState::WRONG_WAY_RECOVERY) {
        state.running_state = RunningState::RUNNING;
      }
    }

    // Empirical Sniegbaltite yaw sign: right/CW positive, left/CCW negative.
    float yaw_deviation =
        imu_ready ? heading_deviation(state.track_direction,
                                      state.imu.measurement.yaw,
                                      max_forward_yaw, min_forward_yaw)
                  : 0.0f;

    // Handle state transition edge triggers (e.g., triggered via BLE)
    if (state.running_state != prev_running_state) {
      if (state.running_state == RunningState::COUNTDOWN) {
        countdown_start_ms = state.current_time_ms;
        startup.enter(countdown_start_ms);
        obstacle_avoidance.reset();
        reverse_recovery.reset();
        resume_centering = false;
      } else if (state.running_state == RunningState::IDLE) {
        reset_pid(state.steering_pid);
        seed_pid = true;
        throttle_limiter.reset(0.0f);
        steering_limiter.reset(0.0f);
        obstacle_detected = false;
        wrong_way_start_us = 0;
        last_slope_us = 0;
        is_on_slope = false;
        obstacle_avoidance.reset();
        reverse_recovery.reset();
        resume_centering = false;
      } else if (state.running_state == RunningState::WRONG_WAY_RECOVERY) {
        recovery_phase = 1;
        recovery_phase_start_us = state.current_time_us;
      }
      prev_running_state = state.running_state;
    }

    bool countdown_ready = startup.tick(
        main_snapshot, state.running_state == RunningState::COUNTDOWN,
        state.current_time_ms, config::start_delay_ms,
        config::imu_calib_extension_ms, config::imu_min_calib_samples,
        sensors::send_command);
    bool inhibited = !required_ready || startup.pending();
    bool skip_control = sensor_motion_inhibit(state, inhibited);
    if (skip_control) {
      recovery_phase_start_us = 0;
      recovery_phase = 0;
      obstacle_detected = false;
      obstacle_start_us = 0;
      wrong_way_start_us = 0;
      seed_pid = true;
      obstacle_avoidance.reset();
      reverse_recovery.reset();
      resume_centering = state.current_throttle < 0.0f;
    }
    if (!inhibited && was_inhibited) {
      reset_pid(state.steering_pid);
      seed_pid = true;
      obstacle_detected = false;
      wrong_way_start_us = 0;
    }
    was_inhibited = inhibited;
    sensor_led.update(state.imu.health,
                      state.running_state == RunningState::COUNTDOWN,
                      required_ready);
    if (state.running_state == RunningState::COUNTDOWN &&
        (!countdown_ready || !required_ready)) {
      static uint32_t last_reason_ms = 0;
      if (state.current_time_ms - last_reason_ms >= 1000) {
        last_reason_ms = state.current_time_ms;
        notify("[RUN] waiting: sensors=%u calibration_pending=%u\n",
               required_ready, startup.pending());
      }
    }

    // Main State Machine
    if (!skip_control)
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
        if (countdown_ready && required_ready) {
          startup.permitted_motion();
          last_slope_us = 0;
          is_on_slope = false;
          state.running_state = RunningState::RUNNING;
          reset_pid(state.steering_pid);
          seed_pid = true;
          obstacle_detected = false;
          max_forward_yaw = state.imu.measurement.yaw;
          min_forward_yaw = state.imu.measurement.yaw;
          wrong_way_start_us = 0;
        }
        break;

      case RunningState::RUNNING: {
        if (resume_centering && state.current_throttle >= 0.0f) {
          resume_centering = false;
          obstacle_avoidance.resume(state.current_time_us);
        }
        if (!resume_centering)
          obstacle_avoidance.update(state, state.current_time_us, imu_ready,
                                    orientation_generation);
        // Wrong-way check
        if (imu_ready && yaw_deviation > config::wrong_way_yaw_threshold_deg) {
          if (wrong_way_start_us == 0) {
            wrong_way_start_us = state.current_time_us;
          } else if (state.current_time_us - wrong_way_start_us >=
                     static_cast<uint32_t>(config::wrong_way_debounce_ms) *
                         1000) {
            state.running_state = RunningState::WRONG_WAY_RECOVERY;
            state.maneuver_start_us = state.current_time_us;
            recovery_phase = 1;
            recovery_phase_start_us = state.current_time_us;
            break;
          }
        } else {
          wrong_way_start_us = 0;
        }

        // Front obstacle detection
        uint16_t rev_dist = (state.dist_reverse > 0)
                                ? state.dist_reverse
                                : config::default_dist_reverse;
        bool obstacle = (state.dist_front.distance_mm < rev_dist);
        if (obstacle) {
          if (!obstacle_detected) {
            obstacle_detected = true;
            obstacle_start_us = state.current_time_us;
          } else if (state.current_time_us - obstacle_start_us >=
                     static_cast<uint32_t>(config::reverse_trigger_delay_ms) *
                         1000) {
            state.running_state = RunningState::REVERSE;
            state.maneuver_start_us = state.current_time_us;
            reverse_recovery.begin(state, obstacle_avoidance.direction(),
                                   imu_ready, orientation_generation);
            state.target_throttle = -state.reverse_throttle;
            state.target_steering = 0.0f;
            resume_centering = false;
            obstacle_detected = false;
            break;
          }
        } else {
          obstacle_detected = false;
        }

        SensorGuidance guidance =
            obstacle_avoidance.guidance(state, state.current_time_us);
        float raw_steer_target = obstacle_avoidance.steer(guidance.steering);
        float front_urgency =
            std::max(guidance.urgency, obstacle_avoidance.urgency());

        // Steering PID update (includes trim offset)
        state.steering_pid.error = raw_steer_target + state.steering_pid.offset;
        if (seed_pid) {
          state.steering_pid.last_error = state.steering_pid.error;
          seed_pid = false;
        }
        state.target_steering = update_pid(delta_time, state.steering_pid);
        state.target_steering = obstacle_avoidance.steer(state.target_steering);
        if (resume_centering) {
          // Forward steering while the ESC still commands reverse would rotate
          // the nose the wrong way. Center until the throttle crosses zero.
          state.target_steering = 0.0f;
          reset_pid(state.steering_pid);
          seed_pid = true;
        }

        // 5. Throttle control with corner slowing
        float throttle = state.base_throttle;
        if (!is_on_slope && state.dist_far > 0 &&
            state.dist_front.distance_mm > state.dist_far) {
          throttle = state.straight_throttle;
        }
        float corner_speed_factor = 1.0f - 0.45f * front_urgency;
        state.target_throttle = throttle * corner_speed_factor;
        if (obstacle)
          state.target_throttle = 0.0f;

        break;
      }

      case RunningState::REVERSE:
      case RunningState::REVERSE_TURN: {
        uint16_t rev_dist = state.dist_reverse > 0
                                ? state.dist_reverse
                                : config::default_dist_reverse;
        RunningState next = reverse_recovery.update(state, rev_dist, imu_ready,
                                                    orientation_generation);
        if (next != state.running_state)
          state.maneuver_start_us = state.current_time_us;
        state.running_state = next;
        state.target_throttle =
            next == RunningState::RUNNING ? 0.0f : -state.reverse_throttle;
        state.target_steering = reverse_recovery.steering();
        if (next == RunningState::RUNNING) {
          reset_pid(state.steering_pid);
          seed_pid = true;
          obstacle_detected = false;
          obstacle_start_us = 0;
          wrong_way_start_us = 0;
          resume_centering = true;
        }
        break;
      }

      case RunningState::WRONG_WAY_RECOVERY: {
        float recovery_forward_steer =
            (state.track_direction == TrackDirection::CLOCKWISE) ? 1.0f : -1.0f;
        float recovery_reverse_steer = -recovery_forward_steer;

        uint32_t phase_time_us =
            state.current_time_us - recovery_phase_start_us;
        uint32_t total_recovery_time_us =
            state.current_time_us - state.maneuver_start_us;

        // Timeout safeguard: if recovery takes too long, reset frontier and
        // resume
        if (total_recovery_time_us >=
            static_cast<uint32_t>(config::wrong_way_turnaround_timeout_ms) *
                1000) {
          max_forward_yaw = min_forward_yaw = state.imu.measurement.yaw;
          wrong_way_start_us = 0;
          reset_pid(state.steering_pid);
          seed_pid = true;
          obstacle_detected = false;
          state.running_state = RunningState::RUNNING;
          break;
        }

        if (recovery_phase == 1) {
          // Phase 1: Forward drive with full lock towards turn / wall to pivot
          // nose (500ms max, min 250ms)
          state.target_throttle = state.base_throttle;
          state.target_steering = recovery_forward_steer;

          // Transition to Phase 2 after 500ms (or if hitting wall after at
          // least 250ms)
          uint16_t rev_dist = (state.dist_reverse > 0)
                                  ? state.dist_reverse
                                  : config::default_dist_reverse;
          bool obstacle = (state.dist_front.distance_mm < rev_dist);
          if (phase_time_us >= 500000 ||
              (obstacle && phase_time_us >= 250000)) {
            recovery_phase = 2;
            recovery_phase_start_us = state.current_time_us;
          }
        } else if (recovery_phase == 2) {
          // Phase 2: Reverse drive with opposite steering lock to swing tail
          // and rotate nose (900ms)
          state.target_throttle = -state.reverse_throttle;
          state.target_steering = recovery_reverse_steer;

          // Run reverse for 900ms to give plenty of distance to rotate
          if (phase_time_us >= 900000) {
            recovery_phase = 3;
            recovery_phase_start_us = state.current_time_us;
          }
        } else if (recovery_phase == 3) {
          // Phase 3: Counter-turn forward with recovery steering to finish the
          // 180° turnaround! (600ms max, min 250ms)
          state.target_throttle = state.base_throttle;
          state.target_steering = recovery_forward_steer;

          // Finish recovery when heading is aligned, front obstacle hit, or
          // after 600ms
          uint16_t rev_dist = (state.dist_reverse > 0)
                                  ? state.dist_reverse
                                  : config::default_dist_reverse;
          bool obstacle = (state.dist_front.distance_mm < rev_dist);
          if (yaw_deviation <= config::wrong_way_resume_threshold_deg ||
              (obstacle && phase_time_us >= 250000) ||
              phase_time_us >= 600000) {
            max_forward_yaw = min_forward_yaw = state.imu.measurement.yaw;
            wrong_way_start_us = 0;
            reset_pid(state.steering_pid);
            seed_pid = true;
            obstacle_detected = false;
            state.running_state = RunningState::RUNNING;
          }
        }
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

    // telemetry logging
    if (state.current_time_us - last_telemetry_us >= telemetry_interval_us) {
      last_telemetry_us += telemetry_interval_us;
      if (state.current_time_us - last_telemetry_us >= telemetry_interval_us) {
        last_telemetry_us = state.current_time_us;
      }
      send_telemetry();
    }
  }
}

void sensor_task_diagnostics() {
  MainSensorSnapshot main;
  SideSensorSnapshot side;
  sensors::get_main_snapshot(main);
  sensors::get_side_snapshot(side);
  notify("[SENSORS] main stack_min=%lu bytes max_tick=%lu us imu_init=%lu us\n",
         static_cast<unsigned long>(main.task_stack_min_bytes),
         static_cast<unsigned long>(main.max_tick_us),
         static_cast<unsigned long>(main.last_imu_init_us));
  notify("[SENSORS] side stack_min=%lu bytes max_tick=%lu us generation=%lu\n",
         static_cast<unsigned long>(side.task_stack_min_bytes),
         static_cast<unsigned long>(side.max_tick_us),
         static_cast<unsigned long>(main.orientation_generation));
  notify("[SENSORS] max periods: main=%lu side=%lu control=%lu us\n",
         static_cast<unsigned long>(main.max_period_us),
         static_cast<unsigned long>(side.max_period_us),
         static_cast<unsigned long>(control_max_period_us.load()));
}
