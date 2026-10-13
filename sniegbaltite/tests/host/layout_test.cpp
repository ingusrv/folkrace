#include "driver/bmi088.h"
#include "protocol.h"
#include "state.h"
#include <type_traits>
static_assert(std::is_same<decltype(RobotState::dist_front), SensorData>::value);
static_assert(std::is_same<decltype(RobotState::imu), IMUSensorData>::value);
static_assert(sizeof(TelemetryPacket) == 184);
static_assert(sizeof(TelemetryPacketV1) == 184);
