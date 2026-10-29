#pragma once
#include "sensor_snapshots.h"
#include "sensors/health.h"
#include "side_fit.h"

struct ToFInitDiagnostics {
  char stage[20] = "not_started";
  uint32_t duration_ms = 0;
  uint32_t driver_value = 0;
  unsigned driver_error = 0;
  uint8_t transport_error = 0;
  uint8_t found_address = 0;
  bool started = false, complete = false, success = false, in_progress = false;
};

struct ToFZoneDiagnostics {
  int16_t range_mm[16] = {};
  uint8_t status[16] = {};
  uint8_t targets[16] = {};
  uint16_t valid_zones = 0, floor_zones = 0, clearance_zones = 0;
  uint8_t nearest_zone = 255;
  float pitch_deg = 0.0f;
};

// All three ranging sensors cross cores in one coherent queue message.
struct RosepinkToFSnapshot {
  uint32_t publish_timestamp_us = 0;
  uint32_t task_stack_min_bytes = 0;
  uint32_t max_tick_us = 0;
  uint32_t max_period_us = 0;
  uint32_t recovery_id = 0;
  bool recovering = true;
  SensorData dist_front, dist_left, dist_right;
  SideWallEstimate left_wall, right_wall;
  uint32_t frame_count[3] = {};
  uint32_t max_read_us[3] = {};
  uint16_t frame_bytes[3] = {};
  ToFInitDiagnostics initialization[3];
  ToFZoneDiagnostics zones[3];
};
namespace sensors {
/** Require every physical ranging sensor and no outstanding firmware reload. */
inline bool ranging_ready(const RosepinkToFSnapshot &s, uint32_t now_us) {
  return !s.recovering && sensor_policy::available(s.dist_front, now_us) &&
         sensor_policy::available(s.dist_left, now_us) &&
         sensor_policy::available(s.dist_right, now_us);
}
/** Create initial snapshots and both bus owners. False inhibits motion. */
bool init();
/** Copy the latest complete IMU snapshot without waiting. */
bool get_main_snapshot(MainSensorSnapshot &out);
/** Copy the latest complete three-sensor ranging snapshot without waiting. */
bool get_tof_snapshot(RosepinkToFSnapshot &out);
/** Acknowledge a reload request only after zero throttle reaches the ESC.
 * @param recovery_id Request copied from the ranging snapshot. */
void acknowledge_stopped(uint32_t recovery_id);
/** Enqueue a bounded diagnostic line; delivery is paced by the control loop. */
void queue_diagnostic(const char *format, ...);
/** Forward one queued sensor log to BLE; call from the control loop. */
void poll_diagnostics();
/** Report retained initialization details, including before BLE connected. */
void notify_init_diagnostics(const RosepinkToFSnapshot &snapshot);
/** Enqueue without waiting; on failure retry the same command ID. */
bool send_command(const SensorCommand &cmd);
} // namespace sensors
