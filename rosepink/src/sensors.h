#pragma once
#include "sensor_snapshots.h"
#include "side_fit.h"

// Wall model and clearances cross cores in one coherent queue message.
struct RosepinkSideSnapshot : SideSensorSnapshot {
  SideWallEstimate left_wall, right_wall;
};
namespace sensors {
/** Create safe initial snapshots and both owners. False permanently inhibits
 * motion. */
bool init();
/** Copy the latest complete main-bus snapshot without waiting. */
bool get_main_snapshot(MainSensorSnapshot &out);
/** Copy the latest complete side-bus snapshot without waiting. */
bool get_side_snapshot(RosepinkSideSnapshot &out);
/** Enqueue without waiting; on failure retry the same command ID. */
bool send_command(const SensorCommand &cmd);
} // namespace sensors
