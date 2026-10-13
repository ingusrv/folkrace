#include "commands.h"
#include "../prefs/prefs.h"
#include "../state.h"
#include "ble.h"
#include "sensor_snapshots.h"
#include "sensors/health.h"
#include <Arduino.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

extern RobotState state;

static void format_distance(const SensorData &sensor, char *out, size_t size) {
  if (sensor_policy::available(sensor, state.current_time_us))
    snprintf(out, size, "%u", sensor.distance_mm);
  else
    snprintf(out, size, "%s", sensor_health_to_str(sensor.health));
}

// One bounded reply per slot; optionally request a single slot (0..7).
void cmd_sensors(char *args) {
  static const char *names[] = {"front",   "right",   "left",    "side_fl",
                                "side_rl", "side_fr", "side_rr", "imu"};
  int first = 0, last = 7;
  if (args && *args) {
    char *end = nullptr;
    long slot = strtol(args, &end, 10);
    if (end == args || slot < 0 || slot > 7) {
      notify("sensors [0..7]\n");
      return;
    }
    first = last = static_cast<int>(slot);
  }
  for (int i = first; i <= last; ++i) {
    const SensorData *distance =
        i < 7 ? sensor_policy::distances(state, i) : nullptr;
    const SensorStatus &status =
        distance ? static_cast<const SensorStatus &>(*distance)
                 : static_cast<const SensorStatus &>(state.imu);
    if (distance) {
      notify("%d %s %s age=%u errors=%lu streak=%u error=%u "
             "last_mm=%u raw_mm=%u status=0x%02x internal=%u raw_age=%u\n",
             i, names[i], sensor_health_to_str(status.health),
             sensor_policy::age_ms(status.timestamp_us, state.current_time_us),
             static_cast<unsigned long>(status.error_count),
             status.consecutive_errors,
             static_cast<unsigned>(status.last_error), distance->distance_mm,
             distance->raw_range_mm, distance->raw_status,
             (distance->raw_status & 0x78) >> 3,
             sensor_policy::age_ms(distance->raw_timestamp_us,
                                   state.current_time_us));
    } else {
      notify("7 imu %s age=%u errors=%lu streak=%u error=%u\n",
             sensor_health_to_str(status.health),
             sensor_policy::age_ms(status.timestamp_us, state.current_time_us),
             static_cast<unsigned long>(status.error_count),
             status.consecutive_errors,
             static_cast<unsigned>(status.last_error));
    }
  }
  notify("ages in ms; 65535 means no sample. taskstats: task timing/stacks\n");
}

// Implemented by the project using thread-safe snapshot peeks, without I2C.
extern void sensor_task_diagnostics();
void cmd_taskstats(char *) { sensor_task_diagnostics(); }

void cmd_start(char *args) {
  state.running_state = RunningState::COUNTDOWN;
  notify("starting countdown\n");
}

void cmd_stop(char *args) {
  state.running_state = RunningState::IDLE;
  state.target_throttle = 0.0f;
  state.current_throttle = 0.0f;
  state.target_steering = 0.0f;
  notify("stopped\n");
}

void cmd_k(char *args) {
  if (args == nullptr || *args == '\0') {
    notify("kp %.4f\nki %.4f\nkd %.4f\n", state.steering_pid.Kp,
           state.steering_pid.Ki, state.steering_pid.Kd);
    return;
  }

  char *saveptr = nullptr;
  char *token = strtok_r(args, " \r\n\t", &saveptr);
  if (token == nullptr) {
    notify("kp %.4f\nki %.4f\nkd %.4f\n", state.steering_pid.Kp,
           state.steering_pid.Ki, state.steering_pid.Kd);
    return;
  }

  if (strcasecmp(token, "p") == 0) {
    char *val = strtok_r(nullptr, " \r\n\t", &saveptr);
    if (val) {
      state.steering_pid.Kp = atoff(val);
    }
    notify("kp %.4f\n", state.steering_pid.Kp);
  } else if (strcasecmp(token, "i") == 0) {
    char *val = strtok_r(nullptr, " \r\n\t", &saveptr);
    if (val) {
      state.steering_pid.Ki = atoff(val);
    }
    notify("ki %.4f\n", state.steering_pid.Ki);
  } else if (strcasecmp(token, "d") == 0) {
    char *val = strtok_r(nullptr, " \r\n\t", &saveptr);
    if (val) {
      state.steering_pid.Kd = atoff(val);
    }
    notify("kd %.4f\n", state.steering_pid.Kd);
  } else {
    // Try setting all three: k <p> <i> <d>
    state.steering_pid.Kp = atoff(token);
    char *val_i = strtok_r(nullptr, " \r\n\t", &saveptr);
    if (val_i) {
      state.steering_pid.Ki = atoff(val_i);
    }
    char *val_d = strtok_r(nullptr, " \r\n\t", &saveptr);
    if (val_d) {
      state.steering_pid.Kd = atoff(val_d);
    }
    notify("kp %.4f\nki %.4f\nkd %.4f\n", state.steering_pid.Kp,
           state.steering_pid.Ki, state.steering_pid.Kd);
  }
}

static inline float parse_throttle_arg(const char *str) {
  float v = atoff(str);
  if (v > 1.0f || v < -1.0f) {
    v /= 100.0f;
  }
  return v;
}

void cmd_speed(char *args) {
  if (args == nullptr || *args == '\0') {
    notify("throttle base %.2f\nstraight %.2f\nreverse %.2f\n",
           state.base_throttle, state.straight_throttle,
           state.reverse_throttle);
    return;
  }

  char *saveptr = nullptr;
  char *token = strtok_r(args, " \r\n\t", &saveptr);
  if (token == nullptr) {
    notify("throttle base %.2f\nstraight %.2f\nreverse %.2f\n",
           state.base_throttle, state.straight_throttle,
           state.reverse_throttle);
    return;
  }

  if (strcasecmp(token, "base") == 0) {
    char *val = strtok_r(nullptr, " \r\n\t", &saveptr);
    if (val) {
      state.base_throttle = parse_throttle_arg(val);
    }
    notify("base throttle %.2f\n", state.base_throttle);
  } else if (strcasecmp(token, "straight") == 0) {
    char *val = strtok_r(nullptr, " \r\n\t", &saveptr);
    if (val) {
      state.straight_throttle = parse_throttle_arg(val);
    }
    notify("straight throttle %.2f\n", state.straight_throttle);
  } else if (strcasecmp(token, "reverse") == 0 ||
             strcasecmp(token, "rev") == 0) {
    char *val = strtok_r(nullptr, " \r\n\t", &saveptr);
    if (val) {
      state.reverse_throttle = parse_throttle_arg(val);
    }
    notify("reverse throttle %.2f\n", state.reverse_throttle);
  } else {
    // Try setting all three: speed <base> <straight> <reverse>
    state.base_throttle = parse_throttle_arg(token);
    char *val_s = strtok_r(nullptr, " \r\n\t", &saveptr);
    if (val_s) {
      state.straight_throttle = parse_throttle_arg(val_s);
    }
    char *val_r = strtok_r(nullptr, " \r\n\t", &saveptr);
    if (val_r) {
      state.reverse_throttle = parse_throttle_arg(val_r);
    }
    notify("throttle base %.2f\nstraight %.2f\nreverse %.2f\n",
           state.base_throttle, state.straight_throttle,
           state.reverse_throttle);
  }
}

void cmd_offset(char *args) {
  if (args != nullptr && *args != '\0') {
    char *saveptr = nullptr;
    char *val = strtok_r(args, " \r\n\t", &saveptr);
    if (val != nullptr) {
      state.steering_pid.offset = atoff(val);
    }
  }
  notify("steering offset %.2f\n", state.steering_pid.offset);
}

void cmd_dist(char *args) {
  if (args != nullptr && *args != '\0') {
    char *saveptr = nullptr;
    char *token = strtok_r(args, " \r\n\t", &saveptr);
    if (token != nullptr) {
      if (strcasecmp(token, "far") == 0) {
        char *val = strtok_r(nullptr, " \r\n\t", &saveptr);
        if (val) {
          state.dist_far = (uint16_t)atol(val);
        }
      } else if (strcasecmp(token, "rev") == 0 ||
                 strcasecmp(token, "reverse") == 0) {
        char *val = strtok_r(nullptr, " \r\n\t", &saveptr);
        if (val) {
          state.dist_reverse = (uint16_t)atol(val);
        }
      } else {
        state.dist_far = (uint16_t)atol(token);
        char *val = strtok_r(nullptr, " \r\n\t", &saveptr);
        if (val) {
          state.dist_reverse = (uint16_t)atol(val);
        }
      }
    }
  }
  char front[16], left[16], right[16];
  format_distance(state.dist_front, front, sizeof(front));
  format_distance(state.dist_left, left, sizeof(left));
  format_distance(state.dist_right, right, sizeof(right));
  notify("dist front %s, left %s, right %s, far %u, rev %u\n", front, left,
         right, state.dist_far, state.dist_reverse);
}

void cmd_save(char *args) {
  save_robot_prefs(state);
  notify("settings saved to nvs\n");
}

void cmd_load(char *args) {
  load_robot_prefs(state);
  notify("settings loaded from nvs\n");
}

void cmd_state(char *args) {
  notify("state %s, steer %.2f throttle %.2f\n",
         running_state_to_str(state.running_state), state.current_steering,
         state.current_throttle);
  cmd_dist(nullptr);
  cmd_k(nullptr);
  cmd_speed(nullptr);
}

void cmd_help(char *args) {
  notify(
      " sensors [0..7] - health/raw diagnostics\n taskstats - timing/stacks\n");
  notify("Commands:\n"
         " start - countdown\n"
         " stop - idle / stop\n"
         " state - view state\n"
         " k [p|i|d <v>] | [<p> <i> <d>]\n"
         " speed [base|straight|rev <v>]\n"
         " offset [<v>] - steer trim\n"
         " dist [far <v> | rev <v>] - sensors\n"
         " save - save to nvs\n"
         " load - load from nvs\n");
}

CommandEntry command_list[] = {
    {"sensors", cmd_sensors}, {"taskstats", cmd_taskstats},
    {"start", cmd_start},     {"stop", cmd_stop},
    {"state", cmd_state},     {"k", cmd_k},
    {"pid", cmd_k},           {"speed", cmd_speed},
    {"throt", cmd_speed},     {"offset", cmd_offset},
    {"o", cmd_offset},        {"dist", cmd_dist},
    {"save", cmd_save},       {"load", cmd_load},
    {"help", cmd_help},       {"?", cmd_help},
};

void handle_command(char *command, char *args) {
  for (auto &entry : command_list) {
    if (strcasecmp(command, entry.name) == 0) {
      entry.handler(args);
      return;
    }
  }
  notify("unknown command: %s\n", command);
}
