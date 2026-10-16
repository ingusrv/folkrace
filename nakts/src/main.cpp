#include "ble/ble.h"
#include "config.h"
#include "control/pid.h"
#include "control/sensor_guidance.h"
#include "control/sensor_motion_gate.h"
#include "control/slew_rate_limiter.h"
#include "driver/RZ7886Driver.h"
#include "driver/servo.h"
#include "escape_recovery.h"
#include "obstacle_navigation.h"
#include "prefs/prefs.h"
#include "protocol.h"
#include "sensors/health.h"
#include "sensors/tof_bus.h"
#include "stall_detector.h"
#include "state.h"
#include "telemetry.h"
#include "util.h"
#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>
#include <esp_wifi.h>

RobotState state = {};

RZ7886Driver drive_motor(pin::bi, pin::fi);
Servo steer_servo(pin::servo);

static SlewRateLimiter throttle_limiter(config::throttle_slew_rate_accel,
                                        config::throttle_slew_rate_decel);
static SlewRateLimiter steering_limiter(config::steering_slew_rate,
                                        config::steering_slew_rate);

static sensor_policy::ToFBus sensor_bus(Wire, pin::sda, pin::scl,
                                        config::sensor_min_raw,
                                        config::sensor_max_raw,
                                        config::tof_timing_budget_us);
static bool recovery_stopping = false;
static bool was_inhibited = true;
static bool seed_pid = true;

static uint32_t countdown_start_ms = 0;
static uint32_t obstacle_start_us = 0;
static bool obstacle_detected = false;
static RunningState prev_running_state = RunningState::IDLE;

static ObstacleAvoidance obstacle_avoidance;
static ReverseRecovery reverse_recovery;
static EscapeRecovery escape_recovery;
static StallDetector stall_detector;
static bool resume_centering = false;

static uint32_t last_telemetry_us = 0;
constexpr uint32_t telemetry_interval_us = 1000000 / 30; // 30 Hz

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

static void send_telemetry() {
  PacketHeader header = {
      .magic = message_magic,
      .type = PacketType::TELEMETRY,
      .robot_id = RobotId::NAKTS,
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

  TelemetryPacket pkt = make_telemetry(state, flags, true, 0.0f);

  uint8_t buffer[sizeof(PacketHeader) + sizeof(TelemetryPacket)];
  memcpy(buffer, &header, sizeof(PacketHeader));
  memcpy(buffer + sizeof(PacketHeader), &pkt, sizeof(TelemetryPacket));

  esp_now_send(broadcast_address, buffer, sizeof(buffer));
}

void setup() {
  load_robot_prefs(state);

  Serial.begin(115200);

  drive_motor.init();
  steer_servo.init();

  for (unsigned i = 0; i < 7; ++i)
    sensor_policy::distances(state, i)->health = SensorHealth::ABSENT;
  state.imu.health = SensorHealth::ABSENT;
  sensor_bus.count = 3;
  sensor_bus.devices[0].address = 0x30;
  sensor_bus.devices[0].xshut = pin::xshut_front;
  sensor_bus.devices[1].address = 0x31;
  sensor_bus.devices[1].xshut = pin::xshut_right;
  sensor_bus.devices[2].address = 0x32;
  sensor_bus.devices[2].xshut = pin::xshut_left;
  drive_motor.stop();
  sensor_bus.begin(); // Initialization runs only after zero outputs are written.

  init_ble("nakts");
  init_espnow();

  state.running_state = RunningState::COUNTDOWN;
  countdown_start_ms = millis();
  prev_running_state = state.running_state;

  state.track_direction = TrackDirection::CLOCKWISE;
}

static uint32_t last_loop_us = 0;
constexpr uint32_t loop_interval_us = 1000000 / 100; // 100 Hz
constexpr float delta_time = static_cast<float>(loop_interval_us) / 1000000.0f;

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
    state.dist_right = sensor_bus.devices[1].data;
    state.dist_left = sensor_bus.devices[2].data;
    state.current_time_us = micros();
    state.current_time_ms = millis();
    sensor_policy::expire_all(state, state.current_time_us);
    if (sensor_bus.recovery_due())
      recovery_stopping = true;
    bool required_ready =
        sensor_policy::required(state, state.current_time_us, true);
    bool inhibited = !required_ready || recovery_stopping;

    // Handle state transition edge triggers (e.g., triggered via BLE)
    if (state.running_state != prev_running_state) {
      if (state.running_state == RunningState::COUNTDOWN) {
        countdown_start_ms = state.current_time_ms;
        escape_recovery.reset();
        obstacle_avoidance.reset();
        reverse_recovery.reset();
        resume_centering = false;
      } else if (state.running_state == RunningState::IDLE) {
        escape_recovery.reset();
        reset_pid(state.steering_pid);
        seed_pid = true;
        throttle_limiter.reset(0.0f);
        steering_limiter.reset(0.0f);
        obstacle_detected = false;
        obstacle_avoidance.reset();
        reverse_recovery.reset();
        resume_centering = false;
      }
      prev_running_state = state.running_state;
    }

    bool skip_control = sensor_motion_inhibit(state, inhibited);
    if (skip_control) {
      escape_recovery.reset();
      obstacle_detected = false;
      obstacle_start_us = 0;
      seed_pid = true;
      obstacle_avoidance.reset();
      reverse_recovery.reset();
      resume_centering = state.current_throttle < 0.0f;
    }
    if (!inhibited && was_inhibited) {
      reset_pid(state.steering_pid);
      seed_pid = true;
      obstacle_detected = false;
    }
    was_inhibited = inhibited;

    // Main State Machine
    if (skip_control || escape_recovery.active() ||
        state.running_state != RunningState::RUNNING || resume_centering)
      stall_detector.reset();

    if (!skip_control && escape_recovery.active()) {
      // Own the complete rock, including its short forward phase. Ordinary
      // obstacle detection must not immediately cancel forward rocking.
      escape_recovery.update(state);
      if (!escape_recovery.active()) {
        notify("[RUN] escape complete, resuming\n");
        reset_pid(state.steering_pid);
        seed_pid = true;
        reverse_recovery.reset();
        obstacle_avoidance.resume(state.current_time_us);
        obstacle_detected = false;
        obstacle_start_us = 0;
        resume_centering = state.current_throttle < 0.0f;
      }
    } else if (!skip_control)
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
                config::start_delay_ms &&
            required_ready && !recovery_stopping) {
          state.running_state = RunningState::RUNNING;
          reset_pid(state.steering_pid);
          seed_pid = true;
          obstacle_detected = false;
        }
        break;

      case RunningState::RUNNING: {
        uint16_t rev_dist = (state.dist_reverse > 0)
                                ? state.dist_reverse
                                : config::default_dist_reverse;
        escape_recovery.observe(state, rev_dist);
        if (resume_centering && state.current_throttle >= 0.0f) {
          resume_centering = false;
          obstacle_avoidance.resume(state.current_time_us);
        }
        if (!resume_centering)
          obstacle_avoidance.update(state, state.current_time_us, false, 0);

        // Front obstacle detection
        bool obstacle = (state.dist_front.distance_mm < rev_dist);
        bool obstacle_confirmed = false;
        if (obstacle) {
          if (!obstacle_detected) {
            obstacle_detected = true;
            obstacle_start_us = state.current_time_us;
          } else if (state.current_time_us - obstacle_start_us >=
                     static_cast<uint32_t>(config::reverse_trigger_delay_ms) *
                         1000) {
            obstacle_confirmed = true;
          }
        } else {
          obstacle_detected = false;
        }

        bool stalled = !resume_centering &&
                       stall_detector.update(state, false, 0);
        if (obstacle_confirmed || stalled) {
          if (stalled)
            notify("[RUN] stalled for %lu ms (distance only), reversing\n",
                   static_cast<unsigned long>(config::stall_timeout_ms));
          state.running_state = RunningState::REVERSE;
          state.maneuver_start_us = state.current_time_us;
          if (escape_recovery.begin_attempt(state,
                                             obstacle_avoidance.direction())) {
            notify("[RUN] repeated reverses, trying timed escape\n");
            reverse_recovery.reset();
          } else {
            reverse_recovery.begin(state, obstacle_avoidance.direction(),
                                   false, 0);
          }
          state.target_throttle = -state.reverse_throttle;
          state.target_steering = 0.0f;
          resume_centering = false;
          obstacle_detected = false;
          stall_detector.reset();
          break;
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
          // Forward steering while motor still commands reverse would rotate
          // the nose the wrong way. Center until the throttle crosses zero.
          state.target_steering = 0.0f;
          reset_pid(state.steering_pid);
          seed_pid = true;
        }

        // Throttle control with corner slowing
        float throttle = state.base_throttle;
        if (state.dist_far > 0 &&
            state.dist_front.distance_mm > state.dist_far) {
          throttle = state.straight_throttle;
        }
        // Slow down when entering a corner so front wheels grip and don't understeer
        float corner_speed_factor = 1.0f - 0.45f * front_urgency;
        state.target_throttle = throttle * corner_speed_factor;
        if (obstacle)
          state.target_throttle = 0.0f;
        break;
      }

      case RunningState::REVERSE:
      case RunningState::REVERSE_TURN: {
        uint16_t rev_dist = (state.dist_reverse > 0)
                                ? state.dist_reverse
                                : config::default_dist_reverse;
        RunningState next = reverse_recovery.update(state, rev_dist, false, 0);
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
          resume_centering = true;
        }
        break;
      }

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

    if (recovery_stopping && state.current_throttle == 0.0f &&
        sensor_bus.recover())
      recovery_stopping = false;

    // ESP-NOW Telemetry
    if (state.current_time_us - last_telemetry_us >= telemetry_interval_us) {
      last_telemetry_us += telemetry_interval_us;
      if (state.current_time_us - last_telemetry_us >= telemetry_interval_us) {
        last_telemetry_us = state.current_time_us;
      }
      send_telemetry();
    }

    static uint32_t last_debug_ms = 0;
    if (state.current_time_ms - last_debug_ms >= 200) {
      last_debug_ms = state.current_time_ms;
      Serial.printf("[RUN] st=%s f=%u l=%u r=%u err=%.2f str=%.2f thr=%.2f\n",
                    running_state_to_str(state.running_state),
                    state.dist_front.distance_mm, state.dist_left.distance_mm,
                    state.dist_right.distance_mm, state.steering_pid.error,
                    state.current_steering, state.current_throttle);
    }
  }
}

void sensor_task_diagnostics() {
  notify(
      "[SENSORS] control-loop owner; recovery waits for zero motor outputs\n");
}
