#include "commands.h"
#include "../prefs/prefs.h"
#include "../state.h"
#include "ble.h"
#include <Arduino.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

extern RobotState state;

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

void cmd_speed(char *args) {
  if (args == nullptr || *args == '\0') {
    notify("throttle base %.1f\nstraight %.1f\nreverse %.1f\n",
           state.base_throttle, state.straight_throttle,
           state.reverse_throttle);
    return;
  }

  char *saveptr = nullptr;
  char *token = strtok_r(args, " \r\n\t", &saveptr);
  if (token == nullptr) {
    notify("throttle base %.1f\nstraight %.1f\nreverse %.1f\n",
           state.base_throttle, state.straight_throttle,
           state.reverse_throttle);
    return;
  }

  if (strcasecmp(token, "base") == 0) {
    char *val = strtok_r(nullptr, " \r\n\t", &saveptr);
    if (val) {
      state.base_throttle = atoff(val);
    }
    notify("base throttle %.1f\n", state.base_throttle);
  } else if (strcasecmp(token, "straight") == 0) {
    char *val = strtok_r(nullptr, " \r\n\t", &saveptr);
    if (val) {
      state.straight_throttle = atoff(val);
    }
    notify("straight throttle %.1f\n", state.straight_throttle);
  } else if (strcasecmp(token, "reverse") == 0 ||
             strcasecmp(token, "rev") == 0) {
    char *val = strtok_r(nullptr, " \r\n\t", &saveptr);
    if (val) {
      state.reverse_throttle = atoff(val);
    }
    notify("reverse throttle %.1f\n", state.reverse_throttle);
  } else {
    // Try setting all three: speed <base> <straight> <reverse>
    state.base_throttle = atoff(token);
    char *val_s = strtok_r(nullptr, " \r\n\t", &saveptr);
    if (val_s) {
      state.straight_throttle = atoff(val_s);
    }
    char *val_r = strtok_r(nullptr, " \r\n\t", &saveptr);
    if (val_r) {
      state.reverse_throttle = atoff(val_r);
    }
    notify("throttle base %.1f\nstraight %.1f\nreverse %.1f\n",
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
  notify("dist front %u, left %u, right %u, far %u, rev %u\n", state.dist_front,
         state.dist_left, state.dist_right, state.dist_far, state.dist_reverse);
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
  notify("state: %s\n"
         "pid: kp=%.4f ki=%.4f kd=%.4f offset=%.2f\n"
         "throt: base=%.1f str=%.1f rev=%.1f\n"
         "dist: f=%u l=%u r=%u far=%u rev=%u\n"
         "curr: steer=%.2f throt=%.2f\n",
         running_state_to_str(state.running_state), state.steering_pid.Kp,
         state.steering_pid.Ki, state.steering_pid.Kd,
         state.steering_pid.offset, state.base_throttle,
         state.straight_throttle, state.reverse_throttle, state.dist_front,
         state.dist_left, state.dist_right, state.dist_far, state.dist_reverse,
         state.current_steering, state.current_throttle);
}

void cmd_help(char *args) {
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
    {"start", cmd_start}, {"stop", cmd_stop},     {"state", cmd_state},
    {"k", cmd_k},         {"pid", cmd_k},         {"speed", cmd_speed},
    {"throt", cmd_speed}, {"offset", cmd_offset}, {"o", cmd_offset},
    {"dist", cmd_dist},   {"save", cmd_save},     {"load", cmd_load},
    {"help", cmd_help},   {"?", cmd_help},
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
