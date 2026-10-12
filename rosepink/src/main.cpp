#include "ble/ble.h"
#include "config.h"
#include "control/pid.h"
#include "control/slew_rate_limiter.h"
#include "driver/bmi088.h"
#include "driver/esc.h"
#include "driver/servo.h"
#include "prefs/prefs.h"
#include "protocol.h"
#include "state.h"
#include "util.h"
#include <Arduino.h>
#include <SparkFun_VL53L5CX_Library.h>
#include <VL53L0X.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>
#include <esp_wifi.h>

ESC drive_motor = ESC(pin::esc);
Servo steer_servo = Servo(pin::servo);
BMI088Driver imu = BMI088Driver(Wire);
RobotState state = {};

static SlewRateLimiter throttle_limiter(config::throttle_slew_rate_accel,
                                        config::throttle_slew_rate_decel);
static SlewRateLimiter steering_limiter(config::steering_slew_rate,
                                        config::steering_slew_rate);

VL53L0X sensor_front;
VL53L0X sensor_left;
VL53L0X sensor_right;

static SparkFun_VL53L5CX sensor_side_left;
static SparkFun_VL53L5CX sensor_side_right;

struct SideWallEstimate {
  float distance_mm = 0.0f;
  float min_distance_mm = 0.0f;
  float front_dist_mm = 0.0f;
  float rear_dist_mm = 0.0f;
  float heading_error_rad = 0.0f;
  float heading_error_deg = 0.0f;
  bool valid = false;
};

static SideWallEstimate side_wall_left = {};
static SideWallEstimate side_wall_right = {};
static portMUX_TYPE side_wall_mux = portMUX_INITIALIZER_UNLOCKED;

static TaskHandle_t side_tof_task_handle = nullptr;
static volatile bool side_tof_ready = false;
static volatile bool side_tof_error = false;

static uint16_t dist_front_left = config::sensor_max_raw;
static uint16_t dist_front_right = config::sensor_max_raw;
static uint16_t dist_rear_left = config::sensor_max_raw;
static uint16_t dist_rear_right = config::sensor_max_raw;

static uint32_t countdown_start_ms = 0;
static uint32_t obstacle_start_us = 0;
static bool obstacle_detected = false;
static RunningState prev_running_state = RunningState::IDLE;
static float reverse_turn_steering = 0.0f;

static float max_forward_yaw = 0.0f;
static float min_forward_yaw = 0.0f;
static uint32_t wrong_way_start_us = 0;
static uint8_t recovery_phase = 1;
static uint32_t recovery_phase_start_us = 0;

static float flat_pitch = 0.0f;
static bool is_on_slope = false;
static uint32_t last_slope_us = 0;

static TaskHandle_t tof_led_task_handle = nullptr;
static volatile bool tof_recovering = true;

static void tof_led_task(void *pvParameters) {
  bool led_on = false;
  while (true) {
    if (tof_recovering) {
      led_on = !led_on;
      if (led_on) {
        rgbLedWrite(pin::rgb_led_pin, config::rgb_led_brightness, 0, 0);
      } else {
        rgbLedWrite(pin::rgb_led_pin, 0, 0, 0);
      }
      vTaskDelay(pdMS_TO_TICKS(150));
    } else {
      if (led_on) {
        led_on = false;
        rgbLedWrite(pin::rgb_led_pin, 0, 0, 0);
      }
      vTaskDelay(pdMS_TO_TICKS(50));
    }
  }
}

void init_tof() {
  pinMode(pin::xshut_left, OUTPUT);
  pinMode(pin::xshut_right, OUTPUT);

  digitalWrite(pin::xshut_left, LOW);
  digitalWrite(pin::xshut_right, LOW);

  delay(10);

  // Front sensor: if power was lost, it reset to default 0x29; if power was
  // retained, it's at 0x30
  if (sensor_front.getAddress() != 0x30) {
    sensor_front.setAddress(0x30);
  } else {
    sensor_front.setAddress(0x30);
    if (sensor_front.last_status != 0) {
      sensor_front = VL53L0X();
      sensor_front.setAddress(0x30);
    }
  }
  sensor_front.init();
  sensor_front.setTimeout(100);
  sensor_front.setMeasurementTimingBudget(config::tof_timing_budget_us);
  sensor_front.startContinuous();

  delay(5);

  pinMode(pin::xshut_left, INPUT);
  delay(10);
  sensor_left = VL53L0X();
  sensor_left.setAddress(0x32);
  sensor_left.init();
  sensor_left.setTimeout(100);
  sensor_left.setMeasurementTimingBudget(config::tof_timing_budget_us);
  sensor_left.startContinuous();

  delay(5);

  pinMode(pin::xshut_right, INPUT);
  delay(10);
  sensor_right = VL53L0X();
  sensor_right.setAddress(0x31);
  sensor_right.init();
  sensor_right.setTimeout(100);
  sensor_right.setMeasurementTimingBudget(config::tof_timing_budget_us);
  sensor_right.startContinuous();

  delay(10);
}

static constexpr float col_sin[4] = {
    -0.290285f,
    -0.098017f,
     0.098017f,
     0.290285f
};
static constexpr float col_cos[4] = {
     0.956940f,
     0.995185f,
     0.995185f,
     0.956940f
};

static SideWallEstimate process_vl53l5cx(const VL53L5CX_ResultsData &data,
                                        bool flip_x, bool flip_y, bool is_left) {
  SideWallEstimate est = {};
  float x_pts[4];
  float y_pts[4];
  int valid_count = 0;
  float min_observed_dist = 10000.0f;

  for (int c = 0; c < 4; c++) {
    int eff_c = flip_x ? (3 - c) : c;
    float sum_d = 0.0f;
    int row_valid = 0;

    for (int r = 0; r < 4; r++) {
      int eff_r = flip_y ? (3 - r) : r;
      int raw_idx = eff_r * 4 + eff_c;
      uint8_t status = data.target_status[raw_idx];
      uint16_t d = data.distance_mm[raw_idx];

      // Target status 5 (100% valid), 6 (wrap around), 9 (large pulse)
      if ((status == 5 || status == 6 || status == 9) &&
          d >= config::sensor_min_raw && d <= config::sensor_max_raw) {
        sum_d += static_cast<float>(d);
        row_valid++;
        if (static_cast<float>(d) < min_observed_dist) {
          min_observed_dist = static_cast<float>(d);
        }
      }
    }

    if (row_valid > 0) {
      float r_avg = sum_d / static_cast<float>(row_valid);
      x_pts[valid_count] = r_avg * col_sin[c];
      y_pts[valid_count] = r_avg * col_cos[c];
      valid_count++;
    }
  }

  if (valid_count >= 2) {
    float sum_x = 0.0f, sum_y = 0.0f;
    for (int i = 0; i < valid_count; i++) {
      sum_x += x_pts[i];
      sum_y += y_pts[i];
    }
    float mean_x = sum_x / static_cast<float>(valid_count);
    float mean_y = sum_y / static_cast<float>(valid_count);

    float num = 0.0f, den = 0.0f;
    for (int i = 0; i < valid_count; i++) {
      float dx = x_pts[i] - mean_x;
      num += dx * (y_pts[i] - mean_y);
      den += dx * dx;
    }

    float m = (den > 1e-4f) ? (num / den) : 0.0f;
    float y0 = mean_y - m * mean_x;

    // Heading error: positive = vehicle turned into wall (demands opposite steer)
    // Left side: turned into left wall -> m < 0 -> theta = -atan(m) > 0 (steer right)
    // Right side: turned into right wall -> m < 0 -> theta = +atan(m) < 0 (steer left)
    float theta_rad = is_left ? -std::atan(m) : std::atan(m);
    float dist = std::max(0.0f, y0 * std::cos(theta_rad));

    est.heading_error_rad = theta_rad;
    est.heading_error_deg = theta_rad * (180.0f / 3.14159265f);
    est.distance_mm = dist;
    est.min_distance_mm = (min_observed_dist < 10000.0f) ? min_observed_dist : dist;
    est.valid = (dist >= config::sensor_min_raw && dist <= config::sensor_max_raw);

    float half_baseline = config::side_sensor_baseline_mm * 0.5f;
    float cos_m = std::cos(std::atan(m));
    est.front_dist_mm = std::max(0.0f, (y0 + m * half_baseline) * cos_m);
    est.rear_dist_mm = std::max(0.0f, (y0 - m * half_baseline) * cos_m);
  } else if (valid_count == 1) {
    float dist = y_pts[0];
    est.heading_error_rad = 0.0f;
    est.heading_error_deg = 0.0f;
    est.distance_mm = dist;
    est.min_distance_mm = (min_observed_dist < 10000.0f) ? min_observed_dist : dist;
    est.front_dist_mm = dist;
    est.rear_dist_mm = dist;
    est.valid = (dist >= config::sensor_min_raw && dist <= config::sensor_max_raw);
  } else {
    est.distance_mm = static_cast<float>(config::sensor_max_raw);
    est.min_distance_mm = static_cast<float>(config::sensor_max_raw);
    est.front_dist_mm = static_cast<float>(config::sensor_max_raw);
    est.rear_dist_mm = static_cast<float>(config::sensor_max_raw);
    est.heading_error_rad = 0.0f;
    est.heading_error_deg = 0.0f;
    est.valid = false;
  }

  return est;
}

static bool init_side_sensors_async() {
  Serial.println("[TOF-SIDE] Initializing VL53L5CX sensors on Wire1...");

  // Hold Right sensor in reset
  pinMode(pin::xshut_side_right, OUTPUT);
  digitalWrite(pin::xshut_side_right, LOW);
  vTaskDelay(pdMS_TO_TICKS(50));

  // Probe Left sensor address before calling begin()
  uint8_t left_probe_addr = 0;
  Wire1.beginTransmission(0x29);
  if (Wire1.endTransmission() == 0) {
    left_probe_addr = 0x29;
  } else {
    Wire1.beginTransmission(config::vl53l5cx_addr_left);
    if (Wire1.endTransmission() == 0) {
      left_probe_addr = config::vl53l5cx_addr_left;
    }
  }

  if (left_probe_addr == 0) {
    Serial.println("[TOF-SIDE] Left VL53L5CX not responding on 0x29 or 0x30!");
    return false;
  }

  bool left_ok = sensor_side_left.begin(left_probe_addr, Wire1);
  if (!left_ok) {
    Serial.println("[TOF-SIDE] Failed to initialize Left VL53L5CX!");
    return false;
  }
  if (left_probe_addr != config::vl53l5cx_addr_left) {
    sensor_side_left.setAddress(config::vl53l5cx_addr_left);
  }
  sensor_side_left.setResolution(4 * 4);
  sensor_side_left.setRangingFrequency(config::vl53l5cx_frequency_hz);
  sensor_side_left.startRanging();
  Serial.printf("[TOF-SIDE] Left VL53L5CX initialized at 0x%02X\n", config::vl53l5cx_addr_left);

  vTaskDelay(pdMS_TO_TICKS(50));

  // Release Right sensor from reset
  digitalWrite(pin::xshut_side_right, HIGH);
  vTaskDelay(pdMS_TO_TICKS(50));

  // Probe Right sensor address before calling begin()
  uint8_t right_probe_addr = 0;
  Wire1.beginTransmission(0x29);
  if (Wire1.endTransmission() == 0) {
    right_probe_addr = 0x29;
  } else {
    Wire1.beginTransmission(config::vl53l5cx_addr_right);
    if (Wire1.endTransmission() == 0) {
      right_probe_addr = config::vl53l5cx_addr_right;
    }
  }

  if (right_probe_addr == 0) {
    Serial.println("[TOF-SIDE] Right VL53L5CX not responding on 0x29 or 0x31!");
    return false;
  }

  bool right_ok = sensor_side_right.begin(right_probe_addr, Wire1);
  if (!right_ok) {
    Serial.println("[TOF-SIDE] Failed to initialize Right VL53L5CX!");
    return false;
  }
  if (right_probe_addr != config::vl53l5cx_addr_right) {
    sensor_side_right.setAddress(config::vl53l5cx_addr_right);
  }
  sensor_side_right.setResolution(4 * 4);
  sensor_side_right.setRangingFrequency(config::vl53l5cx_frequency_hz);
  sensor_side_right.startRanging();
  Serial.printf("[TOF-SIDE] Right VL53L5CX initialized at 0x%02X\n", config::vl53l5cx_addr_right);

  return true;
}

static void side_tof_task(void *pvParameters) {
  // Move large result buffers to static storage to conserve task stack
  static VL53L5CX_ResultsData left_results = {};
  static VL53L5CX_ResultsData right_results = {};

  while (true) {
    side_tof_ready = false;
    side_tof_error = true;

    // Retry initialization until successful
    while (!init_side_sensors_async()) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }

    side_tof_ready = true;
    side_tof_error = false;
    Serial.println("[TOF-SIDE] Both VL53L5CX sensors ready and ranging!");

    uint32_t last_left_data_ms = millis();
    uint32_t last_right_data_ms = millis();

    while (true) {
      if (sensor_side_left.isDataReady()) {
        if (sensor_side_left.getRangingData(&left_results)) {
          SideWallEstimate est_l = process_vl53l5cx(
              left_results, config::side_left_flip_x, config::side_left_flip_y, true);
          portENTER_CRITICAL(&side_wall_mux);
          side_wall_left = est_l;
          portEXIT_CRITICAL(&side_wall_mux);
          last_left_data_ms = millis();
        }
      }

      if (sensor_side_right.isDataReady()) {
        if (sensor_side_right.getRangingData(&right_results)) {
          SideWallEstimate est_r = process_vl53l5cx(
              right_results, config::side_right_flip_x, config::side_right_flip_y, false);
          portENTER_CRITICAL(&side_wall_mux);
          side_wall_right = est_r;
          portEXIT_CRITICAL(&side_wall_mux);
          last_right_data_ms = millis();
        }
      }

      // Check for communication stall (>1000ms without data from an active sensor)
      uint32_t now = millis();
      if ((now - last_left_data_ms > 1000) || (now - last_right_data_ms > 1000)) {
        Serial.printf("[TOF-SIDE] Stalled! l_age=%u r_age=%u. Triggering recovery...\n",
                      now - last_left_data_ms, now - last_right_data_ms);
        break; // Exit to outer retry loop
      }

      vTaskDelay(pdMS_TO_TICKS(5)); // ~200 Hz polling
    }
  }
}

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

void setup() {
  load_robot_prefs(state);

  xTaskCreatePinnedToCore(tof_led_task, "tof_led_task", 2048, NULL, 1,
                          &tof_led_task_handle, 0);

  Wire.begin(pin::sda, pin::scl, 400000);
  Wire1.setBufferSize(1024);
  Wire1.begin(pin::sda_side, pin::scl_side, 400000);
  Serial.begin(115200);

  if (!imu.begin()) {
    Serial.println("[IMU] BMI088 initialization failed!");
  } else {
    Serial.println("[IMU] BMI088 initialized successfully");
    imu.update(state.imu);
    flat_pitch = state.imu.pitch;
  }

  drive_motor.init();
  steer_servo.init();

  init_tof();

  // Launch asynchronous side ToF background task on Core 0 with 16KB stack
  xTaskCreatePinnedToCore(side_tof_task, "side_tof_task", 16384, NULL, 1,
                          &side_tof_task_handle, 0);

  tof_recovering =
      (sensor_front.last_status != 0 || sensor_left.last_status != 0 ||
       sensor_right.last_status != 0 || !side_tof_ready);

  pinMode(pin::direction_switch, INPUT_PULLUP);

  state.track_direction = (digitalRead(pin::direction_switch) == LOW)
                              ? TrackDirection::COUNTER_CLOCKWISE
                              : TrackDirection::CLOCKWISE;
  Serial.printf("[DIRECTION] Track direction: %s (switch pin %d = %s)\n",
                track_direction_to_str(state.track_direction),
                pin::direction_switch,
                digitalRead(pin::direction_switch) == LOW ? "LOW" : "HIGH");

  init_ble("roze");
  init_espnow();

  state.dist_side_front_left = config::sensor_max_raw;
  state.dist_side_front_right = config::sensor_max_raw;
  state.dist_side_rear_left = config::sensor_max_raw;
  state.dist_side_rear_right = config::sensor_max_raw;

  state.running_state = RunningState::COUNTDOWN;
  countdown_start_ms = millis();
  imu.start_calibration();
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

static inline uint16_t read_clamped_sensor(VL53L0X &sensor) {
  uint16_t raw = tof_read_distance(sensor);
  if (sensor.last_status != 0 || raw > config::sensor_max_raw) {
    return config::sensor_max_raw;
  }
  if (raw < config::sensor_min_raw) {
    return config::sensor_min_raw;
  }
  return raw;
}

static void send_telemetry() {
  PacketHeader header = {
      .magic = message_magic,
      .type = PacketType::TELEMETRY,
      .robot_id = RobotId::ROSEPINK,
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

  TelemetryPacket pkt = {
      .timestamp_ms = state.current_time_ms,
      .running_state = state.running_state,
      .flags = flags,
      .dist_front = state.dist_front,
      .dist_right = state.dist_right,
      .dist_left = state.dist_left,
      .dist_side_front_left = state.dist_side_front_left,
      .dist_side_rear_left = state.dist_side_rear_left,
      .dist_side_front_right = state.dist_side_front_right,
      .dist_side_rear_right = state.dist_side_rear_right,
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

void loop() {
  state.current_time_ms = millis();
  state.current_time_us = micros();

  if (state.current_time_us - last_loop_us >= loop_interval_us) {
    last_loop_us += loop_interval_us;
    if (state.current_time_us - last_loop_us >= loop_interval_us) {
      last_loop_us = state.current_time_us;
    }

    // Update IMU sensor data & fusion
    imu.update(state.imu);

    // Slope / hill detection based on pitch deviation from flat ground
    float pitch_dev = fabs(state.imu.pitch - flat_pitch);
    bool pitch_on_slope = (pitch_dev >= config::slope_pitch_threshold_deg);
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
            ? TrackDirection::COUNTER_CLOCKWISE
            : TrackDirection::CLOCKWISE;
    if (current_direction != state.track_direction) {
      state.track_direction = current_direction;
      max_forward_yaw = state.imu.yaw;
      min_forward_yaw = state.imu.yaw;
      wrong_way_start_us = 0;
      reset_pid(state.steering_pid);
      if (state.running_state == RunningState::WRONG_WAY_RECOVERY) {
        state.running_state = RunningState::RUNNING;
      }
      Serial.printf("[DIRECTION] Switched track direction to: %s\n",
                    track_direction_to_str(state.track_direction));
    }

    // Read VL53L0X ToF distance sensors when ready
    if (is_tof_data_ready(sensor_front)) {
      state.dist_front = read_clamped_sensor(sensor_front);
    }
    if (is_tof_data_ready(sensor_left)) {
      state.dist_left = read_clamped_sensor(sensor_left);
    }
    if (is_tof_data_ready(sensor_right)) {
      state.dist_right = read_clamped_sensor(sensor_right);
    }

    // Read side ToF distance sensors (Wire1 VL53L5CX background task)
    SideWallEstimate cur_wall_left, cur_wall_right;
    portENTER_CRITICAL(&side_wall_mux);
    cur_wall_left = side_wall_left;
    cur_wall_right = side_wall_right;
    portEXIT_CRITICAL(&side_wall_mux);

    dist_front_left = static_cast<uint16_t>(constrain(
        cur_wall_left.front_dist_mm, 0.0f, static_cast<float>(config::sensor_max_raw)));
    dist_rear_left = static_cast<uint16_t>(constrain(
        cur_wall_left.rear_dist_mm, 0.0f, static_cast<float>(config::sensor_max_raw)));
    dist_front_right = static_cast<uint16_t>(constrain(
        cur_wall_right.front_dist_mm, 0.0f, static_cast<float>(config::sensor_max_raw)));
    dist_rear_right = static_cast<uint16_t>(constrain(
        cur_wall_right.rear_dist_mm, 0.0f, static_cast<float>(config::sensor_max_raw)));

    state.dist_side_front_left = dist_front_left;
    state.dist_side_rear_left = dist_rear_left;
    state.dist_side_front_right = dist_front_right;
    state.dist_side_rear_right = dist_rear_right;

    bool main_tof_error =
        (sensor_front.last_status != 0 || sensor_left.last_status != 0 ||
         sensor_right.last_status != 0);

    tof_recovering = (main_tof_error || !side_tof_ready || side_tof_error);

    // Check if any ToF sensor lost connection (I2C transmission error)
    if (main_tof_error) {
      static uint32_t last_tof_reset_ms = 0;
      if (state.current_time_ms - last_tof_reset_ms >= 250) {
        last_tof_reset_ms = state.current_time_ms;
        Serial.printf("[TOF] I2C error (f=%d l=%d r=%d). Re-initializing ToF "
                      "sensors...\n",
                      sensor_front.last_status, sensor_left.last_status,
                      sensor_right.last_status);
        init_tof();
      }
    }

    // Calculate progress frontier & yaw deviation based on track direction
    float yaw_deviation = 0.0f;
    if (state.track_direction == TrackDirection::COUNTER_CLOCKWISE) {
      if (state.imu.yaw > max_forward_yaw) {
        max_forward_yaw = state.imu.yaw;
      }
      yaw_deviation = max_forward_yaw - state.imu.yaw;
    } else {
      if (state.imu.yaw < min_forward_yaw) {
        min_forward_yaw = state.imu.yaw;
      }
      yaw_deviation = state.imu.yaw - min_forward_yaw;
    }

    // Handle state transition edge triggers (e.g., triggered via BLE)
    if (state.running_state != prev_running_state) {
      if (state.running_state == RunningState::COUNTDOWN) {
        countdown_start_ms = state.current_time_ms;
        imu.start_calibration();
      } else if (state.running_state == RunningState::IDLE) {
        reset_pid(state.steering_pid);
        throttle_limiter.reset(0.0f);
        steering_limiter.reset(0.0f);
        obstacle_detected = false;
        wrong_way_start_us = 0;
        last_slope_us = 0;
        is_on_slope = false;
      } else if (state.running_state == RunningState::REVERSE_TURN) {
        reverse_turn_steering =
            (state.track_direction == TrackDirection::CLOCKWISE) ? 1.0f : -1.0f;
      } else if (state.running_state == RunningState::WRONG_WAY_RECOVERY) {
        recovery_phase = 1;
        recovery_phase_start_us = state.current_time_us;
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
      imu.update_calibration();
      if (state.current_time_ms - countdown_start_ms >=
          config::start_delay_ms) {
        imu.finish_calibration();
        flat_pitch = state.imu.pitch;
        last_slope_us = 0;
        is_on_slope = false;
        state.running_state = RunningState::RUNNING;
        reset_pid(state.steering_pid);
        obstacle_detected = false;
        max_forward_yaw = state.imu.yaw;
        min_forward_yaw = state.imu.yaw;
        wrong_way_start_us = 0;
      }
      break;

    case RunningState::RUNNING: {
      // Wrong-way check
      if (yaw_deviation > config::wrong_way_yaw_threshold_deg) {
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
              (state.track_direction == TrackDirection::CLOCKWISE) ? -1.0f
                                                                   : 1.0f;
          obstacle_detected = false;
          break;
        }
      } else {
        obstacle_detected = false;
      }

      // 1. Diagonal front sensors lateral centering (positive = steer right
      // away from left wall)
      float diag_diff = static_cast<float>(state.dist_right) -
                        static_cast<float>(state.dist_left);
      float steer_diag =
          constrain(diag_diff / config::side_turn_range, -1.0f, 1.0f);

      // 2. Side sensors (FL, RL, FR, RR) centering & orientation compensation
      uint16_t side_left = std::min(dist_front_left, dist_rear_left);
      uint16_t side_right = std::min(dist_front_right, dist_rear_right);

      bool side_left_valid = (side_left < config::sensor_max_raw);
      bool side_right_valid = (side_right < config::sensor_max_raw);

      float steer_side = 0.0f;
      if (side_left_valid && side_right_valid) {
        // Centering between two side walls
        float side_diff =
            static_cast<float>(side_right) - static_cast<float>(side_left);
        steer_side =
            constrain(side_diff / config::side_turn_range, -1.0f, 1.0f);
      } else if (side_left_valid) {
        // Only left side wall visible: push away if closer than side_turn_range
        if (side_left < static_cast<uint16_t>(config::side_turn_range)) {
          steer_side =
              (config::side_turn_range - static_cast<float>(side_left)) /
              config::side_turn_range;
        }
      } else if (side_right_valid) {
        // Only right side wall visible: push away if closer than
        // side_turn_range
        if (side_right < static_cast<uint16_t>(config::side_turn_range)) {
          steer_side =
              -(config::side_turn_range - static_cast<float>(side_right)) /
              config::side_turn_range;
        }
      }

      // Side orientation alignment: steer away if car is angled towards a side
      // wall
      float steer_side_angle = 0.0f;
      if (dist_front_left < config::sensor_max_raw &&
          dist_rear_left < config::sensor_max_raw) {
        // Nose pointing into left wall: rear is farther than front
        float diff = static_cast<float>(dist_rear_left) -
                     static_cast<float>(dist_front_left);
        steer_side_angle += constrain(diff / 150.0f, -0.5f, 0.5f);
      }
      if (dist_front_right < config::sensor_max_raw &&
          dist_rear_right < config::sensor_max_raw) {
        // Nose pointing into right wall: rear is farther than front
        float diff = static_cast<float>(dist_front_right) -
                     static_cast<float>(dist_rear_right);
        steer_side_angle += constrain(diff / 150.0f, -0.5f, 0.5f);
      }

      // Combine diagonal guidance and side guidance
      float steer_guidance;
      if (side_left_valid || side_right_valid) {
        steer_guidance =
            0.5f * steer_diag + 0.5f * (steer_side + steer_side_angle);
      } else {
        steer_guidance = steer_diag;
      }

      // 3. Emergency wall danger repulsion (checks both diagonal and side
      // sensors)
      float steer_repel = 0.0f;
      uint16_t min_left = std::min(side_left, state.dist_left);
      uint16_t min_right = std::min(side_right, state.dist_right);

      if (min_left < config::side_danger_dist) {
        steer_repel += static_cast<float>(config::side_danger_dist - min_left) /
                       static_cast<float>(config::side_danger_dist);
      }
      if (min_right < config::side_danger_dist) {
        steer_repel -=
            static_cast<float>(config::side_danger_dist - min_right) /
            static_cast<float>(config::side_danger_dist);
      }

      // 4. Front obstacle / corner anticipation
      float front_urgency = 0.0f;
      float turn_dir = (state.dist_right >= state.dist_left) ? 1.0f : -1.0f;

      if (state.dist_front < config::front_turn_dist) {
        front_urgency =
            static_cast<float>(config::front_turn_dist - state.dist_front) /
            static_cast<float>(config::front_turn_dist -
                               config::front_critical_dist);
        front_urgency = constrain(front_urgency, 0.0f, 1.0f);
      }

      // Blend side guidance/repulsion and front cornering
      float raw_steer_target =
          (1.0f - front_urgency) *
              constrain(steer_guidance + steer_repel, -1.0f, 1.0f) +
          (front_urgency * turn_dir);
      raw_steer_target = constrain(raw_steer_target, -1.0f, 1.0f);

      // Steering PID update (includes trim offset)
      state.steering_pid.error = raw_steer_target + state.steering_pid.offset;
      state.target_steering = update_pid(delta_time, state.steering_pid);

      // 5. Throttle control with corner slowing
      float throttle = state.base_throttle;
      if (!is_on_slope && state.dist_far > 0 &&
          state.dist_front > state.dist_far) {
        throttle = state.straight_throttle;
      }
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
        reverse_turn_steering =
            (state.track_direction == TrackDirection::CLOCKWISE) ? -1.0f
                                                                 : 1.0f;
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
      float recovery_forward_steer =
          (state.track_direction == TrackDirection::CLOCKWISE) ? -1.0f : 1.0f;
      float recovery_reverse_steer = -recovery_forward_steer;

      uint32_t phase_time_us = state.current_time_us - recovery_phase_start_us;
      uint32_t total_recovery_time_us =
          state.current_time_us - state.maneuver_start_us;

      // Timeout safeguard: if recovery takes too long, reset frontier and
      // resume
      if (total_recovery_time_us >=
          static_cast<uint32_t>(config::wrong_way_turnaround_timeout_ms) *
              1000) {
        if (state.track_direction == TrackDirection::COUNTER_CLOCKWISE) {
          max_forward_yaw = state.imu.yaw;
        } else {
          min_forward_yaw = state.imu.yaw;
        }
        wrong_way_start_us = 0;
        reset_pid(state.steering_pid);
        obstacle_detected = false;
        state.running_state = RunningState::RUNNING;
        break;
      }

      if (recovery_phase == 1) {
        // Phase 1: Forward drive with full lock towards turn / wall to pivot
        // nose (500ms max, min 250ms)
        state.target_throttle = state.base_throttle;
        state.target_steering = recovery_forward_steer;

        // Transition to Phase 2 after 500ms (or if hitting wall after at least
        // 250ms)
        uint16_t rev_dist = (state.dist_reverse > 0)
                                ? state.dist_reverse
                                : config::default_dist_reverse;
        bool obstacle = (state.dist_front < rev_dist);
        if (phase_time_us >= 500000 || (obstacle && phase_time_us >= 250000)) {
          recovery_phase = 2;
          recovery_phase_start_us = state.current_time_us;
        }
      } else if (recovery_phase == 2) {
        // Phase 2: Reverse drive with opposite steering lock to swing tail and
        // rotate nose (900ms)
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

        // Finish recovery when heading is aligned, front obstacle hit, or after
        // 600ms
        uint16_t rev_dist = (state.dist_reverse > 0)
                                ? state.dist_reverse
                                : config::default_dist_reverse;
        bool obstacle = (state.dist_front < rev_dist);
        if (yaw_deviation <= config::wrong_way_resume_threshold_deg ||
            (obstacle && phase_time_us >= 250000) || phase_time_us >= 600000) {
          if (state.track_direction == TrackDirection::COUNTER_CLOCKWISE) {
            max_forward_yaw = state.imu.yaw;
          } else {
            min_forward_yaw = state.imu.yaw;
          }
          wrong_way_start_us = 0;
          reset_pid(state.steering_pid);
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

    static uint32_t last_debug_ms = 0;
    if (state.current_time_ms - last_debug_ms >= 200) {
      last_debug_ms = state.current_time_ms;
      Serial.printf("[RUN] st=%s f=%u l=%u r=%u | fl=%u fr=%u rl=%u rr=%u | "
                    "head=%.1f° cte=%.0fmm str=%.2f thr=%.2f pit=%.1f slp=%d\n",
                    running_state_to_str(state.running_state), state.dist_front,
                    state.dist_left, state.dist_right, dist_front_left,
                    dist_front_right, dist_rear_left, dist_rear_right,
                    state.steering_pid.error, state.steering_pid.integral,
                    state.current_steering, state.current_throttle,
                    state.imu.pitch, is_on_slope ? 1 : 0);
    }
  }

  if (state.current_time_us - last_telemetry_us >= telemetry_interval_us) {
    last_telemetry_us += telemetry_interval_us;
    if (state.current_time_us - last_telemetry_us >= telemetry_interval_us) {
      last_telemetry_us = state.current_time_us;
    }
    send_telemetry();
  }
}