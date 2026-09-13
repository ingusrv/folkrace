#include "prefs.h"
#include <Preferences.h>

void load_state_from_prefs(Preferences &prefs, RobotState &state) {
  state.steering_pid.Kp = prefs.getFloat("kp", 1.0f);
  state.steering_pid.Ki = prefs.getFloat("ki", 0.0f);
  state.steering_pid.Kd = prefs.getFloat("kd", 0.0f);

  state.base_throttle = prefs.getFloat("base_throt", 20.0f);
  state.reverse_throttle = prefs.getFloat("reverse_throt", 20.0f);
  state.straight_throttle = prefs.getFloat("straight_throt", 20.0f);

  state.steering_pid.offset = prefs.getFloat("offset", 0.0f);

  state.dist_far = prefs.getUShort("dist_far", 0);
  state.dist_reverse = prefs.getUShort("dist_rev", 0);
}

void save_state_to_prefs(Preferences &prefs, const RobotState &state) {
  prefs.putFloat("kp", state.steering_pid.Kp);
  prefs.putFloat("ki", state.steering_pid.Ki);
  prefs.putFloat("kd", state.steering_pid.Kd);

  prefs.putFloat("base_throt", state.base_throttle);
  prefs.putFloat("reverse_throt", state.reverse_throttle);
  prefs.putFloat("straight_throt", state.straight_throttle);

  prefs.putFloat("offset", state.steering_pid.offset);

  prefs.putUShort("dist_far", state.dist_far);
  prefs.putUShort("dist_rev", state.dist_reverse);
}

void load_robot_prefs(RobotState &state) {
  Preferences prefs;
  prefs.begin("folkrace", false);
  load_state_from_prefs(prefs, state);
  prefs.end();
}

void save_robot_prefs(const RobotState &state) {
  Preferences prefs;
  prefs.begin("folkrace", false);
  save_state_to_prefs(prefs, state);
  prefs.end();
}
