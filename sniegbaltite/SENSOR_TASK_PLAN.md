# Sensor task and I²C ownership plan

Updated: 2026-10-13. This document records the architecture and concrete design for dual sensor tasks, snapshot queues, blocking IMU recovery, sensor data models, telemetry protocol v1 (first versioned protocol), queued binary serial transport, degraded control policy, and cross-robot migration.

## Open-space policy amendment (2026-10-13)

The user requested continued racing on open straights after observing purple
motion-inhibit blinking. Live telemetry verified fresh 8191/status-4 and
8190/status-6 results with zero I2C errors on Sniegbaltite. This amendment
supersedes the strict range-quality rejection below for that robot only.

`tof_allow_no_return = true` explicitly permits status 4/6 with exact 8190/8191
sentinels as an assumed maximum control distance. Report appended health value
8, `NO_RETURN`, rather than OK. Refresh the usable timestamp only after complete
checked acquisition and interrupt acknowledgement; normal 100 ms freshness,
transport/hardware failure, and no-result timeout rules still apply. Other
robots keep the strict default. Unknown/minimum-range statuses and non-sentinel
quality failures remain unavailable. No-return does not establish an absent
obstacle: validate nearby dark/angled targets on hardware. Raw diagnostics and
quality remain visible in BLE/telemetry/Python; binary packet layout is unchanged.

## Objective

Replace the existing ToF initialization task with two dedicated sensor tasks, one per I²C bus. Each task exclusively owns its bus and all devices on it. Move initialization, acquisition, fusion, calibration, disconnect detection, and recovery into those tasks. Keep the 100 Hz control loop responsive, ensure failure cannot be mistaken for open track, and expose granular sensor health through wireless telemetry.

## Current implementation and problems

- `src/main.cpp` initializes `Wire` and `Wire1` in `setup()`, reads sensors in `loop()`, and initializes/reinitializes ToF sensors in `tof_init_task`.
- IMU reads can run on `Wire` while the ToF initialization task uses the same bus. Initialization flags only gate ToF reads.
- ToF library timeouts are configured after `init()`. They must be configured before initialization, because library initialization contains polling loops (with a zero timeout these loops never exit).
- ToF `last_status` describes the latest write-side transaction. Later transactions can overwrite an earlier error, and register reads do not validate received byte counts. A failed data-ready poll currently looks identical to "not ready".
- The BMI088 library read path does not expose transaction failures or short reads. The shared wrapper performs fusion and calibration without checking read success.
- IMU restarts have not been observed yet. The installed BMI088 initialization includes over 400 ms of reset, self-test, and settling delays. Blocking initialization in the main sensor task is an accepted simplification: front/diagonal acquisition pauses, cached distances expire at the unchanged 100 ms freshness limit, and motion is temporarily inhibited. Continuous distance acquisition during IMU restart is not a requirement for this rollout.
- Recovery currently retries ToF groups at a fixed 500 ms interval. Failed/recovering distances are represented as the maximum range, which makes unavailable data look like clear space.
- Control code uses `sensor_max_raw` as an "invalid" marker (e.g. `side_left_valid = side_left < sensor_max_raw`), conflating "no wall in range" with "sensor unavailable".
- Measurements have no timestamps, so stopped measurements can remain in use indefinitely.
- The receiver currently formats and writes JSON inside the ESP-NOW receive callback. Expanded diagnostics exceed its 460800-baud serial capacity with two robots at 30 Hz. A queue protects the callback from output delays; binary serial transport provides the sustained bandwidth needed for four robots.

## Ownership and communication

### Per-bus tasks

A blocking VL53L0X `init()` (reference calibrations and polling loops, typically 20–40 ms) must not stall devices on the other bus. Therefore, use one task per bus:

| Task | Bus | Core | Devices |
| --- | --- | --- | --- |
| Main sensor task (`main_sensor_task`) | `Wire` | Core 0 | BMI088 (accel + gyro), front, left, right ToF |
| Side sensor task (`side_tof_task`) | `Wire1` | Core 0 | Rear-left, front-left, front-right, rear-right ToF |

Each task exclusively owns:

- Its I²C bus, bus configuration, and bus restart/recovery.
- Its driver objects and every I²C transaction on that bus, including presence checks.
- Its XSHUT pins and ToF address assignment.
- Device health, error accounting, and recovery state for its devices.
- The main task additionally owns IMU fusion, orientation generation, and calibration state.

The control loop (running on Core 1) owns `RobotState`, control decisions, motors, steering, and applying calibration results to control references. Sensor tasks must never write directly into `RobotState`.

### Public API (`src/sensors.h`)

Create `src/sensors.h` and `src/sensors.cpp` with the following clean interface:

```cpp
#pragma once
#include "state.h"
#include <stdint.h>

// Calibration commands apply strictly to the IMU (gyro zero-rate bias & flat pitch).
// ToF sensors are factory-calibrated and do not undergo runtime calibration.
// If ToF calibration is ever required in the future, dedicated command types will be added.
enum class SensorCommandType : uint8_t {
  START_IMU_CALIBRATION,
  FINISH_IMU_CALIBRATION,
  ABORT_IMU_CALIBRATION,
};

enum class SensorCommandResult : uint8_t {
  NONE,
  ACCEPTED,
  COMPLETED,
  INSUFFICIENT_SAMPLES,
  ABORTED,
  REJECTED,
};

struct SensorCommand {
  uint32_t cmd_id = 0;
  SensorCommandType type = SensorCommandType::START_IMU_CALIBRATION;
};

struct MainSensorSnapshot {
  uint32_t publish_timestamp_us = 0;
  uint32_t orientation_generation = 0;
  IMUSensorData imu = {};
  SensorData dist_front = {};
  SensorData dist_left = {};
  SensorData dist_right = {};

  // IMU calibration acknowledgement & status
  uint32_t last_cmd_id_ack = 0;
  SensorCommandResult last_cmd_result = SensorCommandResult::NONE;
  uint32_t imu_calib_start_cmd_id = 0;
  bool imu_calib_in_progress = false;
  bool imu_calib_completed = false; // Completion of the current attempt
  bool imu_calib_valid = false; // Committed initial calibration exists
  uint16_t imu_calib_sample_count = 0;
  float imu_calib_flat_pitch = 0.0f;
};

struct SideSensorSnapshot {
  uint32_t publish_timestamp_us = 0;
  SensorData dist_side_front_left = {};
  SensorData dist_side_rear_left = {};
  SensorData dist_side_front_right = {};
  SensorData dist_side_rear_right = {};
};

namespace sensors {
bool init();
bool get_main_snapshot(MainSensorSnapshot &out);
bool get_side_snapshot(SideSensorSnapshot &out);
bool send_command(const SensorCommand &cmd);
}
```

### Snapshot queues

Each task publishes a complete, trivially copyable snapshot (`MainSensorSnapshot`, `SideSensorSnapshot`) to its own length-one FreeRTOS queue:

- The producer publishes with `xQueueOverwrite()` after acquisition and before background recovery work. It also publishes immediately when a read fails, before attempting another potentially blocking transaction. Before device/group/bus recovery, mark all affected devices `RECOVERING` and publish again; never hide a known failure behind initialization timeouts.
- The control loop copies both with `xQueuePeek(..., 0)` once at the start of each 100 Hz control tick.
- Queue overwrite prevents queue buildup; peeking preserves the most recent snapshot when no new publication has occurred.
- All control calculations and telemetry for that iteration use the copied sensor state. Capture the freshness-check `now_us` after both queue copies so a concurrently published sample cannot appear to come from the future. Producer and consumer loops normally run asynchronously at 100 Hz; expect 0–10 ms additional sample latency when acquisition meets its deadline.
- A published motion-critical failure must cause `state.target_throttle = 0.0f` by the next control tick (nominally within 10 ms, with measured scheduling jitter reported), ramping `state.current_throttle` smoothly to zero via `throttle_limiter` (using `config::throttle_slew_rate_decel`) rather than stepping directly to neutral, which would trigger aggressive ESC braking and stress MOSFETs/gears. If no failure is published because acquisition stalls, the 100 ms ToF freshness deadline independently inhibits driving on the next tick. Physical disconnect-to-inhibit latency includes detection time; measure it separately from publication-to-inhibit latency.
- Pre-seed both snapshot queues inside `sensors::init()` using `xQueueOverwrite()` with default-constructed snapshots (`SensorHealth::INITIALIZING`, `timestamp_us = 0`) so `xQueuePeek(..., 0)` always succeeds with a safe unavailable state from tick 0. Also initialize the control loop's local snapshots as unavailable (`SensorHealth::INITIALIZING`). If queue or task creation fails during boot, keep autonomous motion strictly inhibited.

Commands to the main sensor task use an ordered FreeRTOS queue (`xQueueSend` with 0 timeout):
- Each command carries a monotonic `cmd_id`.
- The main sensor task acknowledges processed commands via `last_cmd_id_ack` and `last_cmd_result` in `MainSensorSnapshot`; acknowledgement alone does not imply calibration succeeded. Publish command results promptly.
- Command queue depth is 4. The control loop keeps at most one unacknowledged command outstanding. If `send_command()` fails because the queue is full, retain and retry the same command on later ticks without blocking or advancing the calibration state. Deduplicate repeated IDs and retain the last result until the next command is processed.
- Match completion to `imu_calib_start_cmd_id` and the acknowledged finish command. A prior attempt's completion cannot satisfy a new attempt. On cancellation, stop issuing finish commands and deliver `ABORT_IMU_CALIBRATION` after any outstanding command is acknowledged; keep motion inhibited until abort is acknowledged.

### Task configuration & watchdog protection

- Both tasks are pinned to Core 0 with priority 2 (`config::sensor_task_priority`), co-existing with WiFi/BLE system tasks.
- Remove `Serial.begin()` and all `Serial.print` / `Serial.println` / `Serial.printf` calls from `sniegbaltite` (`src/main.cpp` and `src/sensors.cpp`). Live debugging and health monitoring are handled via ESP-NOW telemetry and BLE notifications, avoiding USB CDC blocking and stack overhead on Core 0 and Core 1.
- Sensor libraries use polling loops. To prevent starving the FreeRTOS IDLE0 task or triggering the Task Watchdog (TWDT):
  - Always set sensor timeouts (`setTimeout(config::tof_timeout_ms)`) before calling library `init()`.
  - Enforce hardware bus timeouts (`Wire.setTimeOut(config::sensor_bus_timeout_ms)`).
  - Use `xTaskDelayUntil()` at the normal 10 ms cadence and check whether it actually blocked. On an overrun, use `vTaskDelay(1)` and reset the wake-time baseline; do not run repeated catch-up cycles. `vTaskDelayUntil()` alone does not guarantee a yield when its deadline is already past.
  - Break recovery into bounded operations with deadline-based backoff. Blocking IMU initialization and its settling delays are allowed in the sensor-owner task; they must yield CPU time so the side task, control loop, and watchdog idle task can run. Bound library calls as well as bus transactions; audit polling loops so no individual operation can consume the watchdog budget. Do not chain initialization of an entire group without yielding between devices.
  - Watchdog yields do not preserve acquisition in the blocked task. Accept the main-bus data gap during IMU initialization and enforce freshness-based motion inhibit; do not claim uninterrupted 100 Hz sensor acquisition during recovery.
  - Verify the 4096-byte task stack margin under IMU fusion math (`atan2f`, `sqrtf`) and recovery call chains using `uxTaskGetStackHighWaterMark()`.

## Sensor data model (`shared/src/state.h`)

Replace measured numeric distances in `RobotState` with `SensorData`, and give `RobotState::imu` equivalent status metadata. Retain `dist_far` and `dist_reverse` as numeric configuration thresholds. Banana's analog IR readings are ADC values, not distances: move them and their filter state into local variables in `banana/src/main.cpp`; they do not populate `SensorData`.

```cpp
#pragma once
#include <stdint.h>

enum class SensorHealth : uint8_t {
  ABSENT,       // Robot does not populate this distance/IMU slot
  INITIALIZING, // Initializing or waiting for first sample
  OK,           // Actively acquiring valid samples
  STALE,        // Sample exceeded freshness limit
  READ_ERROR,   // Transport or device hardware failure
  OFFLINE,      // Unavailable: recovery backoff or IMU disabled until reboot
  RECOVERING,   // Bus reset or XSHUT sequencing in progress
  RANGE_INVALID, // Device returned a result, but distance is unreliable
};

inline const char *sensor_health_to_str(SensorHealth h) {
  switch (h) {
  case SensorHealth::ABSENT:       return "ABSENT";
  case SensorHealth::INITIALIZING: return "INIT";
  case SensorHealth::OK:           return "OK";
  case SensorHealth::STALE:        return "STALE";
  case SensorHealth::READ_ERROR:   return "ERROR";
  case SensorHealth::OFFLINE:      return "OFFLINE";
  case SensorHealth::RECOVERING:   return "RECOVER";
  case SensorHealth::RANGE_INVALID: return "RANGE_INVALID";
  default:                         return "UNKNOWN";
  }
}

enum class SensorError : uint8_t {
  NONE,
  NACK,
  TIMEOUT,
  SHORT_READ,
  BUS_ERROR,
  INIT_FAILED,
  CONFIG_FAILED,
  MEASUREMENT_TIMEOUT,
  HARDWARE_ERROR,
};

enum class SensorSlot : uint8_t {
  FRONT = 0,
  RIGHT = 1,
  LEFT = 2,
  SIDE_FRONT_LEFT = 3,
  SIDE_REAR_LEFT = 4,
  SIDE_FRONT_RIGHT = 5,
  SIDE_REAR_RIGHT = 6,
  IMU = 7,
  COUNT = 8,
};

struct SensorStatus {
  uint32_t timestamp_us = 0; // Last successful measurement timestamp
  SensorHealth health = SensorHealth::INITIALIZING;
  uint32_t error_count = 0;
  uint16_t consecutive_errors = 0;
  SensorError last_error = SensorError::NONE;
};

struct SensorData : SensorStatus {
  uint16_t distance_mm = 0;
  uint16_t raw_range_mm = 0;   // Unclamped VL53L0X result, including sentinels
  uint8_t raw_status = 0;      // Full RESULT_RANGE_STATUS register
  uint32_t raw_timestamp_us = 0; // Zero until a complete checked result
};

struct IMUSensorData : SensorStatus {
  IMUState measurement = {};
};
```

### Freshness and availability helpers

```cpp
inline bool is_sensor_fresh(uint32_t timestamp_us, uint32_t now_us, uint32_t max_age_us) {
  return (timestamp_us != 0) && (static_cast<uint32_t>(now_us - timestamp_us) <= max_age_us);
}

inline bool is_sensor_available(const SensorData &s, uint32_t now_us, uint32_t max_age_us) {
  return (s.health == SensorHealth::OK) && is_sensor_fresh(s.timestamp_us, now_us, max_age_us);
}

inline bool is_imu_available(const IMUSensorData &s, uint32_t now_us, uint32_t max_age_us) {
  return (s.health == SensorHealth::OK) && is_sensor_fresh(s.timestamp_us, now_us, max_age_us);
}
```

### Field semantics

- **Successful acquisition**: updates `.distance_mm` (or `.measurement`), updates `timestamp_us`, sets `health = OK`, resets `consecutive_errors = 0`.
- **ToF not ready poll**: if transport succeeds and sensor signals "data not ready", state and counters are unchanged. If the transport poll fails (NACK/timeout), it counts as a failed acquisition.
- **ToF valid distance beyond the configured control range**: if transport and measurement status are valid, clamp a distance above `config::sensor_max_raw` to that maximum, with `health = OK`. A raw value of 8190/8191, or a signal/phase/sigma failure, does not by itself prove open track and must not be promoted to a valid maximum-range reading.
- **ToF unreliable result**: after a complete checked result read and interrupt acknowledgement, a measurement-quality failure sets `health = RANGE_INVALID`. Preserve the last valid distance and its timestamp; it is unavailable to control immediately because health is not `OK`. Do not increment transport/hardware error counters or reset the device solely because results have poor quality. `RANGE_INVALID` distinguishes measurement quality from the `READ_ERROR`/recovery path.
- **Raw ToF diagnostics**: every complete checked VL53L0X result read and interrupt acknowledgement updates `raw_range_mm`, the full `raw_status` byte, and `raw_timestamp_us`, including quality-invalid and hardware-fault results. Failed/partial transport preserves the previous raw record. Copy these fields through snapshots into `RobotState`, BLE diagnostics, and telemetry; do not leave them private to the acquisition task. They never authorize motion or refresh the last usable distance. Non-VL53L0X/absent slots leave `raw_timestamp_us = 0`.
- **Failed acquisition**: a transport or hardware failure immediately sets `health = READ_ERROR`, records `last_error`, increments `consecutive_errors` (saturating at 65535) and `error_count` (saturating at `UINT32_MAX`). The last valid distance/measurement and timestamp are preserved for post-mortem diagnostics. Hardware range statuses use `SensorError::HARDWARE_ERROR`.
- **Freshness expiry**: an otherwise `OK` measurement older than `max_age_us` becomes `STALE` on consumer inspection. Do not overwrite a more specific fault/quality state with `STALE`. Presence probes alone do not restore `OK`.
- **Initializing / Recovering**: remains unavailable until initialization/configuration succeeds and a valid sample arrives.
- **Counter persistence**: `error_count` persists across device and bus resets (cleared only on reboot). `consecutive_errors` tracks transport/hardware failures: a fully checked ToF result with no hardware fault resets this streak even if range quality is invalid. A valid IMU measurement resets its streak. Neither presence probes nor successful initialization alone reset it.
- **ToF result cadence vs usable-distance age**: the owner separately tracks the last fully received/acknowledged result time, including unreliable results, and the time continuous ranging started before any result. These are task-local recovery timers, not replacements for `SensorStatus::timestamp_us`. Repeated quality failures must not trigger a no-result timeout just because the last usable distance is old.
- **IMU timestamp**: advances only when both accelerometer and gyroscope reads succeed. Partial or short reads abort fusion and do not advance the timestamp.

### Updated `RobotState` struct (`shared/src/state.h`)

```cpp
struct RobotState {
  RunningState running_state;
  uint32_t current_time_ms;
  uint32_t current_time_us;
  uint32_t maneuver_start_us;

  SensorData dist_front;
  SensorData dist_right;
  SensorData dist_left;
  SensorData dist_side_front_left;
  SensorData dist_side_rear_left;
  SensorData dist_side_front_right;
  SensorData dist_side_rear_right;

  uint16_t dist_far;      // threshold mm
  uint16_t dist_reverse;  // threshold mm

  float target_steering;
  float current_steering;
  float target_throttle;
  float current_throttle;

  float base_throttle;
  float straight_throttle;
  float reverse_throttle;

  PIDState steering_pid;
  IMUSensorData imu;

  TrackDirection track_direction;
};
```

## Acquisition scheduling

- IMU acquisition & complementary filter fusion normally run at 100 Hz in `main_sensor_task`; recovery may interrupt this cadence.
- ToF continuous ranging is normally polled at 100 Hz in both tasks (retaining the 20 ms timing budget). Main-bus polling pauses during blocking IMU initialization; side-bus acquisition continues independently.
- Within `main_sensor_task`, acquire/fuse the IMU only when ready, then poll front/left/right ToFs and publish acquisition results. Process commands before beginning a due recovery operation. Publish failures immediately as described above. Recovering devices do not receive normal acquisition calls or repeated probes during their backoff. Commands arriving during blocking initialization wait until it returns; process them before another recovery attempt.
- Use the normal delay-until cadence with mandatory blocking delay and schedule reset on overrun, as specified under watchdog protection. Recovery on `Wire` can interrupt its own IMU cadence; treat resulting orientation gaps explicitly.
- Sample physical timestamps using `micros()` immediately upon successful sensor read; publishing the snapshot does not touch sample timestamps.

### ToF timeout handling after an owner pause

- Poll readiness and consume any complete checked result before evaluating the no-result deadline, including on the first cycle after blocking IMU initialization. A ready result must not be discarded because the owner was unable to poll it for 400 ms.
- If a known blocking operation prevents polling, grant previously active ToFs one bounded `tof_resume_grace_us = 100000` window from the operation's return when no result is ready. This only defers the no-result recovery decision; never change valid/raw sample timestamps, freshness limits, or clear an existing fault. A deadline already expired before the operation began gets no grace.
- End grace on a complete result or declare `MEASUREMENT_TIMEOUT` at its deadline. Do not begin another discretionary blocking recovery until affected ToFs have produced a result or reached that deadline. A genuine bus/transport failure can escalate immediately and cancels grace for affected devices.
- Ordinary scheduling overruns do not restart result timers or grant another grace window. Resetting the task's scheduling baseline is independent of these sensor deadlines. This prevents repeated overruns or IMU retries from hiding a silent ToF indefinitely. Apply the same rule to other bus owners that pause polling for known blocking work.

### Blocking IMU initialization and recovery

- Retain synchronous IMU initialization at boot and during recovery. Run `BMI088Driver::begin()` only in the sensor-owner task, never in the 100 Hz control loop. No resumable initialization engine or per-register recovery state machine is required.
- Before calling it, publish the IMU as `RECOVERING` (or `INITIALIZING` at boot) and preserve the last successful ToF sample timestamps. An IMU-only failure does not itself require resetting the bus or ToFs.
- While initialization blocks for approximately 400 ms or longer, front/left/right acquisition and main-task command processing pause. The control loop keeps consuming its last snapshot, applying the normal 100 ms ToF freshness limit and slew-limited zero-throttle override when required distances become stale. Do not extend freshness limits or restamp cached distances. Temporary motion inhibit from this acquisition gap is explicitly accepted.
- Keep the side sensor task independent and the control loop responsive. Initialization delays must yield; finite bus timeouts and bounded polling remain required. Measure full initialization duration under faults rather than assuming the 25 ms bus timeout bounds the whole call.
- On return, process pending commands and resume ToF acquisition/publication before another retry. If initialization failed, use the normal backoff while distances continue updating. Fresh required distance data allows driving to resume even if the IMU remains unavailable.
- Limit initialization to `config::imu_max_init_attempts_per_boot = 3` calls per MCU boot, including initial startup and initialization caused by shared-bus recovery. Increment the count before every call; success never replenishes it. Apply backoff between attempts, with distance acquisition running during backoff. An attempt succeeds only after initialization and a checked accelerometer/gyroscope sample both succeed; successful initialization followed by no valid sample within `sensor_freshness_imu_us` counts as a failed attempt.
- A successful third attempt may keep supplying IMU guidance normally. If that attempt fails, or a later fault requires another initialization after the budget is spent, latch the IMU disabled until MCU reboot, publish `health = OFFLINE`, and retain its last error, counters, measurement, and timestamp for diagnostics. Stop IMU reads, probes, and automatic initialization attempts; a BLE stop/start or later shared-bus recovery must not clear the count/latch or retry the IMU. Continue normal ToF acquisition and permit racing whenever required distances are healthy and fresh. Disable wrong-way detection and slope hold; leave `WRONG_WAY_RECOVERY` if active.
- Giving up on the IMU must not strand calibration commands: keep acknowledging commands, reject new calibration starts while latched disabled, and acknowledge abort by clearing pending calibration state. During initial countdown, abort any pending attempt and wait for its acknowledgement before selecting degraded operation and allowing motion. Preserve any previously committed biases and flat pitch, but do not use IMU guidance while disabled.
- After successful initialization, restore committed biases before accepting the first checked IMU sample, reseed fusion timing/orientation, and publish the new orientation generation. Initialization/self-test samples never enter normal fusion or stationary calibration. Calibration acknowledgements remain mandatory even when delayed by initialization.

## Error detection, recovery, and bus isolation

### Checked acquisition

- Explicitly verify register-address write return codes (`endTransmission() == 0`).
- Validate received byte counts (`requestFrom()` count matches expected bytes).
- For ToF, classify raw device status separately from transport failures and measurement quality using the mapping below. Never treat every non-valid range status as a device fault, and never treat every large raw range as clear space.
- For BMI088, check error registers and transaction return codes before passing samples to fusion.

### VL53L0X result classification and racing fallback

Decode the internal status as `(RESULT_RANGE_STATUS & 0x78) >> 3`. These values are not the public ST API `RangeStatus` values. Use the mapping in the [ST implementation distributed by Adafruit](https://github.com/adafruit/Adafruit_VL53L0X/blob/master/src/core/src/vl53l0x_api_core.cpp), pinning the reference revision when implementing the fork:

| Internal device status | Interpretation | Handling |
| --- | --- | --- |
| 1, 2, 3 | Hardware failure | `READ_ERROR`, `HARDWARE_ERROR`, normal recovery policy |
| 4 | Signal failure | `RANGE_INVALID`; no reset solely for quality |
| 6, 9 | Phase failure | `RANGE_INVALID`; no reset solely for quality |
| 8, 10 | Minimum-range failure | `RANGE_INVALID`; never classify as clear space |
| 11 | Valid device range | Accept only after complete transport/status checks; clamp usable distance to the configured range |
| 0, 5, 7, 12–15 | No usable result in the ST mapping | `RANGE_INVALID`; not evidence of clear space |

- Sigma failure is determined by additional ST API calculations/limit checks; internal status 7 is not a sigma-failure code. Do not claim to implement the full ST quality checks merely by decoding these bits. If the adapter implements sigma checking, a failed check also makes the result `RANGE_INVALID`.
- Treat 8190/8191 sentinels or contradictory/unknown status-distance combinations as unreliable, even if a superficial distance comparison would exceed the configured maximum. Transport failure takes precedence over all status/distance interpretation; never classify bytes from a short read.
- The [ST API manual](https://www.pololu.com/file/0J1508/VL53L0X-UM2309.pdf) describes signal/sigma/phase failures as uncertainty or measurement failure, not proof of an absent obstacle. Open space can produce poor returns, but nearby difficult targets can also be unreliable. Do not restart a functioning device just because it keeps returning weak measurements.
- Default racing behavior treats an unreliable distance as unavailable and applies the existing required-sensor/one-diagonal fallback policy. Thus an unreliable front result can temporarily inhibit motion even on an open straight. This is an explicit limitation, not a device-disconnect diagnosis. Continue ranging and resume normal control on the next valid result without reinitialization.
- Before track rollout, validate behavior against open straights, nearby dark/angled walls, strong ambient light, and too-close targets. Any later rule permitting racing on a weak/no-return result must be an explicit, hardware-validated control policy, with diagnostics retaining the quality distinction; do not silently relabel all such results `OK`.

### Repository-managed driver changes

These are required implementation work, not capabilities of the installed libraries. Do not edit generated `.pio/libdeps` files.

- Refactor `shared/src/driver/bmi088.h` and `.cpp` to provide checked acquisition while retaining synchronous `begin()`. The existing vendor initialization sequence may be reused; a new initialization state machine is out of scope. Replace unchecked runtime `readSensor()` calls with checked register reads; checking a separate presence/error register before an unchecked sample read is insufficient. Preserve configured ranges, scaling, and axis mapping from the pinned upstream version, documenting any reused code's provenance.
- The installed BolderFlight driver maps both accelerometer and gyroscope axes as `X = chip X`, `Y = -chip Y`, `Z = -chip Z` (`tX={1,0,0}`, `tY={0,-1,0}`, `tZ={0,0,-1}`). Preserve this transform exactly once before calibration/fusion; do not introduce an X/Y swap. Verify signs against the current driver with known robot tilts and rotations.
- Retain `<BMI088.h>`, the `__has_include(<BMI088.h>)` guards, and the forced `esp32_bmi088_fix.h` include while synchronous initialization still uses BolderFlight. Remove them only if that dependency is fully eliminated, then verify the shared library still builds for robots without an IMU.
- Keep `begin()` success/failure reporting, clear initialized state on failure, and require a checked sample before publishing the IMU as `OK`. Perform initialization in the bus-owner task and verify bus pins, clock, and timeout configuration remain correct afterward. Expose a checked sample read returning success/error and physical samples (acceleration in m/s² and gyro rate in rad/s). Failed/partial reads must not mutate committed measurements, advance sample timestamps, run fusion, or accumulate calibration samples. Any initial orientation seeding must also use checked samples rather than the vendor's unchecked read path.
- Make fusion accept those already-checked samples and their timestamp, with an explicit timing/orientation reset operation. Calibration consumes the same samples without additional I²C reads. Keep `set_gyro_biases(float bx, float by, float bz)` in rad/s; published angular rates retain the current degrees/s convention. Update Rosepink callers to the same checked API.
- Maintain a pinned, repository-managed Pololu VL53L0X fork under a shared vendor-library directory, with its license and upstream revision recorded. Add `set_host_address(uint8_t address)` to change only the driver's address without any I²C transaction. Keep hardware address changes separate and checked, committing the new host address only after a successful write. Update affected PlatformIO dependencies to select this fork without also resolving a duplicate registry copy.
- The ToF fork must expose checked register operations, including byte-count validation and the first transaction failure across initialization/configuration. Use these for ready/status/range reads and interrupt acknowledgement, and propagate failures to `SensorError`; a later successful write must not erase an earlier failure. Bound initialization polling loops with the configured timeout before starting initialization.

### Escalation sequence

1. **Transient transport/hardware failure**: Mark sensor `READ_ERROR`, preserve last distance, publish immediately, retry next tick. For ToF, poll/consume ready results first, then trigger `MEASUREMENT_TIMEOUT` and recovery if no complete result arrives within the 100 ms result deadline, measured from continuous-ranging start or the last complete result, subject only to the bounded owner-pause grace above. Repeated unreliable results keep the device active and do not enter this recovery path; usable-distance freshness/quality gating still applies independently.
2. **Device recovery (consecutive errors >= 3 or timeout)**:
   - Publish affected devices as `RECOVERING` before blocking work.
   - For sensors with XSHUT, pull XSHUT low, wait 10 ms, re-raise XSHUT, reprogram I²C address, and reinitialize.
   - For the BMI088, run bounded blocking initialization in the owning sensor task, accepting the main-bus acquisition gap and freshness-based motion inhibit. Restore the committed initial calibration via `BMI088Driver::set_gyro_biases(bx, by, bz)` before resuming fusion. Never collect calibration samples as part of recovery; resume distance reads and command processing before retry/backoff handling.
3. **Group recovery**:
   - If address state is uncertain, hold all XSHUT sensors low on that bus, re-address the non-XSHUT sensor (`sensor_front` on `Wire`, `sensor_rear_left` on `Wire1`), then release and address each XSHUT sensor sequentially.
   - Because the non-XSHUT sensor retains power during runtime recovery, it remains at `0x30` rather than resetting to `0x29` (where a fresh `VL53L0X` object's `setAddress(0x30)` would NACK). With all XSHUT pins held `LOW`:
     1. Probe `0x30` first (`bus.beginTransmission(0x30); bus.endTransmission() == 0`). If ACKed, call the fork's `set_host_address(0x30)` without issuing an I²C address-write to `0x29`.
     2. Otherwise, probe `0x29`. If ACKed, call `set_host_address(0x29)` to match the responding device, then use checked `setAddress(0x30)` to assign `0x30` and verify the result.
     3. If neither address ACKs, mark the sensor `INIT_FAILED`.
4. **Bus recovery**:
   - If transactions fail across multiple devices or lines appear held low:
     1. Call `Wire.end()` / `Wire1.end()`.
     2. Set XSHUT pins low.
     3. Configure SCL with Arduino-ESP32's `OUTPUT_OPEN_DRAIN` mode (not `OPEN_DRAIN`, which the BMI088 compatibility header undefines). If SDA is low, toggle SCL up to 9 times until SDA releases, then issue an I²C STOP condition, driving SDA low or releasing it as appropriate.
     4. Call `Wire.begin()` / `Wire1.begin()` with configured pins and 400 kHz clock.
     5. Re-run group address assignment and reinitialization.
5. **Backoff**:
   - Retry failed device/bus recovery with capped exponential backoff: 500 ms, 1 s, 2 s, capped at 5 s.
   - Preserve the current retry delay across isolated successes. A recovery that fails, or relapses before `sensor_recovery_healthy_ms = 2000` of sustained acquisition, advances the delay to the next step. Only that continuous healthy window resets it to 500 ms. For ToFs, checked results without hardware faults count even when range quality is invalid; transport/hardware failure or a result gap over 100 ms restarts the healthy window. Presence probes and initialization success do not count. Returning valid data may restore control readiness immediately; the longer window governs only recovery history.
   - Keep device and bus recovery history outside reconstructed drivers. Reset bus backoff only after all devices participating in that bus recovery meet the same healthy window; exclude absent or latched-disabled devices. A known owner pause may restart the healthy window but never resets backoff by itself.
   - IMU initialization has a separate three-call budget per boot as specified above; sustained health does not replenish it. If recovery is required with no attempts left, keep the IMU disabled until MCU reboot and continue distance-only racing; indefinite device/bus retries must not re-enable it.

Because `Wire` and `Wire1` live in separate tasks, side bus recovery never restarts `Wire` or makes its owner wait for that recovery. Both tasks share Core 0, so scheduling jitter must still be measured under fault load.

## Calibration and control behavior

### IMU calibration & orientation generation

Initial stationary calibration applies exclusively to the BMI088 IMU (gyro zero-rate bias and flat pitch baseline). ToF sensors are factory-calibrated by ST/Pololu and do not undergo runtime calibration; if ToF offset or crosstalk calibration is ever needed in the future, it will be introduced with separate, dedicated command and data types.

- Calibrate only during the initial stationary startup window, before the robot has driven. The control loop tracks whether motion has ever been permitted this boot. A later BLE start/countdown, sensor restart, or bus recovery reuses the initial calibration and must not start another calibration while the car may still be moving.
- Store committed gyro biases and flat pitch in task-owned storage independent of replaceable driver objects. Preserve them and `imu_calib_valid` across all device/bus restarts; restore the biases before resuming fusion via `void set_gyro_biases(float bias_x, float bias_y, float bias_z)` added to `BMI088Driver` in `shared/src/driver/bmi088.h` (which sets `m_gyro_bias_x/y/z` and `m_calibrated = true`). This is lifetime-of-boot storage, not new NVS persistence. A full MCU reboot requires a stationary startup again.
- IMU calibration runs inside `main_sensor_task` upon receiving an eligible `START_IMU_CALIBRATION`. Reject start if a committed calibration already exists. The control loop must never issue start after motion has first been permitted.
- On an accepted start, record `imu_calib_start_cmd_id`, clear accumulation buffers and the current attempt's completion flag, and set `imu_calib_in_progress = true`. Attempt state is separate from committed calibration validity.
- While active (`imu_calib_in_progress = true`), normal 100 Hz IMU reads during `COUNTDOWN` accumulate:
  - Gyroscope angular rate sums ($\Sigma \omega_x, \Sigma \omega_y, \Sigma \omega_z$) and sample count `imu_calib_sample_count`. Reuses regular checked sensor reads without extra bus transactions.
  - Accelerometer gravity vector samples for baseline flat pitch determination.
- Upon receiving `FINISH_IMU_CALIBRATION`, the task:
  - Validates that an attempt is active and at least `config::imu_min_calib_samples` were accumulated. If insufficient samples were recorded, acknowledge `INSUFFICIENT_SAMPLES`, leave `imu_calib_completed = false`, and continue accumulating while the initial startup window remains open. Reject finish if no attempt is active.
  - Computes gyro zero-rate bias offsets: $\text{bias} = \frac{\Sigma \omega}{N}$.
  - Computes and records baseline `imu_calib_flat_pitch`.
  - Resets fusion integrated yaw to 0.0f.
  - Commits the biases and flat pitch together, sets `imu_calib_valid = true`, `imu_calib_completed = true`, and `imu_calib_in_progress = false`, and acknowledges `COMPLETED`.
  - Increments `orientation_generation++`.
- Upon receiving `ABORT_IMU_CALIBRATION` (e.g. countdown aborted by BLE `stop`):
  - Clears only the pending accumulation buffers and current attempt's completion flag, sets `imu_calib_in_progress = false`, and acknowledges `ABORTED`. Never erase or overwrite committed calibration values.
- The snapshot publishes orientation generation, attempt ID/status, committed calibration validity and flat pitch, and the acknowledged command ID/result. A restart preserves the committed values even while IMU health is unavailable.
- Increment `orientation_generation` on initial calibration completion and whenever orientation continuity is lost: IMU reinitialization, fusion reset, or a successful-read gap exceeding `sensor_freshness_imu_us`. Count a gap once, reset fusion timing, and do not integrate the missing interval using a single new gyro sample. Publish the new generation with the first usable orientation in the new frame.
- On a new usable generation, reset yaw frontiers to that snapshot's yaw, clear wrong-way debounce/recovery state, and reset PID history before evaluating heading. Clear transient slope-hold state but retain the original calibrated flat pitch. Re-seeding instantaneous roll/pitch from gravity is permitted; treating the current incline as the new flat baseline is not.
- Recovery never averages gyro samples for bias or learns a new flat pitch. If initial calibration was unavailable when the race began, keep IMU-dependent control disabled for the rest of that boot; recovery must not silently calibrate while driving.

### Degraded operation policy

Stopping mid-race because a secondary sensor glitched causes unnecessary race losses. The control loop enforces this explicit policy:

| Sensor | Motion-critical? | Degraded behavior when unavailable |
| --- | --- | --- |
| **Front ToF** | **Yes** | **Inhibit driving immediately** (`target_throttle = 0.0f`, ramped down via `throttle_limiter` to avoid harsh ESC braking). |
| **Left / Right Diagonals** | **Partially** (need >= 1) | If one diagonal is unavailable, use single-wall repulsion on the remaining diagonal. If both are unavailable, **inhibit motion**. |
| **Side ToFs (FL, RL, FR, RR)** | **No** | Exclude unavailable sensors from side wall centering and angle alignment. If an entire side is lost, rely on the diagonal sensor for repulsion. |
| **IMU** | **No, directly** | Disable wrong-way detection and slope hold. Fall back to standard `base_throttle` / `straight_throttle` while required distances remain available. If in `WRONG_WAY_RECOVERY`, immediately return to `RUNNING`. Blocking IMU reinitialization can temporarily inhibit motion indirectly by making main-bus distances stale. |

- **IMU retry exhaustion**: When initialization is required but the three-call boot budget is exhausted (including failure of the final attempt), keep the IMU `OFFLINE` and disabled until reboot. A successful final attempt remains usable until another recovery is needed. Resume/continue racing on healthy, fresh required distances without further IMU retry pauses. This applies to Sniegbaltite and Rosepink, both during startup and mid-race.
- **Single-diagonal steering fallback**: When one diagonal sensor is unavailable, do not compute `dist_right - dist_left` (which would peg steering hard into the opposite wall). Instead, apply one-sided wall repulsion symmetrically to single-side guidance:
  - Only `dist_left` available: if `dist_left < config::side_turn_range`, `steer_diag = (config::side_turn_range - dist_left) / config::side_turn_range`, else `0.0f`.
  - Only `dist_right` available: if `dist_right < config::side_turn_range`, `steer_diag = -(config::side_turn_range - dist_right) / config::side_turn_range`, else `0.0f`.
  - For front corner anticipation (`turn_dir`), steer away from the available diagonal when within `config::side_turn_range`, or fall back to the track direction.
- **Slew-limited motion inhibit**: The motion inhibit overrides `state.target_throttle = 0.0f` in every running state, including reverse maneuvers. Do **not** bypass `throttle_limiter` when inhibiting while moving—stepping directly to 1500 µs neutral triggers the ESC's built-in brake, causing regenerative current/voltage spikes across the MOSFETs, mechanical shock to the gears, and potential spin-outs. Instead, let `throttle_limiter` ramp `state.current_throttle` smoothly to `0.0f` using `config::throttle_slew_rate_decel`. IMU-dependent control requires both a fresh healthy IMU and committed initial calibration.

### Motion-inhibit entry and exit

- Evaluate required-sensor readiness before maneuver transitions and PID updates. On entering sensor-loss inhibit, cancel any active `REVERSE`, `REVERSE_TURN`, or `WRONG_WAY_RECOVERY` maneuver and return to `RUNNING` under the zero-throttle override. Clear maneuver phase/start times, obstacle debounce, and wrong-way debounce. Do not let maneuver timers expire or start new maneuvers while inhibited; decisions resume from fresh measurements.
- Reset steering PID integral/derivative history and skip PID accumulation while inhibited, including Banana's PD/derivative filter. Hold the last applied steering command during smooth deceleration; normal steering limiting resumes with usable inputs. On exit, seed derivative history from the current valid error and restart obstacle/heading debounce, so stale integral or an error discontinuity cannot produce a resume kick.
- Preserve the throttle limiter's actual output throughout inhibit and resume; do not reset it to zero while the vehicle is still being commanded to move. Retain the configured smooth deceleration. `abs(current_throttle) / throttle_slew_rate_decel` estimates command ramp duration, not stopping distance; measure physical travel under sensor loss on hardware.
- `IDLE`, BLE stop, and countdown/calibration retain their own entry rules and deadlines. Sensor readiness returning must not start an idle robot or bypass the calibration acknowledgement gate. Apply this inhibit policy on all robots, including both final Banana motor outputs.

### Countdown state & readiness gating

- During initial stationary countdown, send start once and wait for its acknowledgement. Once the normal countdown expires and enough samples are reported, send finish once and wait for its matching acknowledgement. On `INSUFFICIENT_SAMPLES`, continue stationary acquisition and retry finish with a new command ID once enough samples are available.
- Bound the initial calibration window by `start_delay_ms + imu_calib_extension_ms` from countdown entry. Do not reset this deadline on sensor recovery. If calibration cannot complete in that window, send abort, wait for acknowledgement, and explicitly select degraded operation with IMU-dependent control disabled. Missing acknowledgements keep motion inhibited; elapsed time alone never implies successful cancellation.
- Transition to `RUNNING` only after the countdown expires, required sensors are healthy and fresh, and either the initial attempt has matching successful completion, a previously committed calibration is being reused, or IMU use is explicitly disabled/absent. No calibration may remain in progress when driving starts.
- If required sensors are not ready or a calibration command is pending, remain in `COUNTDOWN`, blink LED amber/yellow (R=128, G=64, B=0) unless the higher-priority IMU `OFFLINE` indication applies, and rate-limit reason logs via BLE (`notify`). Transition when readiness is attained.
- Subsequent countdowns reuse committed calibration without sending start/finish. Once the robot has driven without valid calibration, subsequent countdowns also retain degraded operation. An aborted initial attempt may be retried only before any motion has been permitted, with the car still stationary.

### IMU offline LED indication

- On Sniegbaltite and Rosepink, blink the robot's RGB LED pink whenever the published IMU health is `SensorHealth::OFFLINE`: R=128, G=32, B=64, with 250 ms on / 250 ms off. This includes an `OFFLINE` backoff state if published before retry exhaustion; after the three-attempt limit latches the IMU disabled, pink blinking persists until MCU reboot.
- Pink `OFFLINE` blinking takes priority over countdown amber, ToF recovery blinking, and normal running/idle LED indications. Keep it visible in `IDLE`, `COUNTDOWN`, and every racing state, including after BLE stop/start. Other sensor faults remain visible through telemetry/BLE and retain their normal motion-inhibit behavior.
- Select the LED mode from the control loop's copied sensor status and publish it to the LED task; one LED task owns all `rgbLedWrite()` calls. Keep blinking asynchronous on core 0, independent of the sensor-owner task, so blocking IMU initialization cannot stall it. Do not restart the blink phase on each control tick or let another LED writer overwrite the pink indication. Restore the applicable normal indication when IMU health leaves `OFFLINE` before retry exhaustion.
- `ABSENT`, `INITIALIZING`, `RECOVERING`, and transient `READ_ERROR` alone do not select pink. The LED indication does not change throttle or restart the IMU; it makes degraded operation visible without a computer.

### Distinguishing availability vs range

Existing code checks `side_left < config::sensor_max_raw` to mean "valid reading". In the new model:
1. `is_sensor_available(state.dist_front, now_us, max_age_us)` checks health and freshness.
2. Once verified available, `state.dist_front.distance_mm < config::sensor_max_raw` determines whether a physical wall is within detection range.

## Telemetry protocol v1 (`shared/src/protocol.h`)

No extra ESP-NOW packet types are introduced. We extend `TelemetryPacket` directly. All robots and the receiver dongle are updated and flashed together, and the Python forwarder is updated with them. We set `protocol_version = 1` as this is the first time explicit packet versioning is introduced. The receiver forwards these binary packets inside a separately versioned serial envelope; JSON is generated on the computer.

### Struct definitions

```cpp
#pragma once
#include "state.h"
#include <stdint.h>

constexpr uint8_t TELEMETRY_PROTOCOL_VERSION = 1;

struct __attribute__((packed)) SensorDiagnostic {
  uint8_t health;              // SensorHealth (1 byte)
  uint16_t age_ms;             // 0xFFFF = no sample, saturated at 65534 (2 bytes)
  uint16_t error_count;        // Saturated (2 bytes)
  uint16_t consecutive_errors; // Saturated (2 bytes)
  uint8_t last_error;          // SensorError (1 byte)
};                             // Total: 8 bytes per slot

struct __attribute__((packed)) RawTofDiagnostic {
  uint16_t range_mm;           // Unclamped VL53L0X result
  uint8_t status;              // Full RESULT_RANGE_STATUS byte
  uint16_t age_ms;             // Raw result age; 0xFFFF = no raw sample
};                             // Total: 5 bytes per distance slot

struct __attribute__((packed)) TelemetryPacket {
  uint8_t protocol_version;    // TELEMETRY_PROTOCOL_VERSION = 1
  uint32_t timestamp_ms;
  RunningState running_state;
  uint8_t flags;               // Bit 0: obstacle, Bit 1: BLE, Bit 2: CW, Bit 3: required sensors OK, Bit 4: slope

  uint16_t dist_front;
  uint16_t dist_right;
  uint16_t dist_left;

  uint16_t dist_side_front_left;
  uint16_t dist_side_rear_left;
  uint16_t dist_side_front_right;
  uint16_t dist_side_rear_right;

  float target_steering;
  float current_steering;
  float target_throttle;
  float current_throttle;

  float pid_error;
  float pid_integral;
  float pid_derivative;

  float accel_x;
  float accel_y;
  float accel_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float roll;
  float pitch;
  float yaw;

  SensorDiagnostic sensors[8]; // Order: FRONT, RIGHT, LEFT, SIDE_FL, SIDE_RL, SIDE_FR, SIDE_RR, IMU
  RawTofDiagnostic raw_tof[7];  // Same distance-slot order, excludes IMU
};

static_assert(sizeof(SensorDiagnostic) == 8, "SensorDiagnostic size mismatch");
static_assert(sizeof(RawTofDiagnostic) == 5, "RawTofDiagnostic size mismatch");
static_assert(sizeof(TelemetryPacket) == 184, "TelemetryPacket size mismatch");
```

- Packet header is 8 bytes; total frame size over ESP-NOW is `8 + 184 = 192 bytes`, well within the 250-byte ESP-NOW limit.
- Flags bit 3 indicates `all_required_sensors_ok`, using each robot's required-sensor policy. It reports sensor readiness, not overall permission to drive: countdown, calibration, or `IDLE` may still inhibit motion.
- Encode raw age from `raw_timestamp_us`, independently of usable-distance age; saturate at 65534 ms and use `0xFFFF` when no raw record exists. Non-VL53L0X slots (including Rosepink's derived VL53L5CX side distances) use the no-raw-record sentinel and zero raw range/status. A raw status of zero is otherwise a real register value, not a missing-data marker. Decode internal status as `(status & 0x78) >> 3`; show it alongside current health/last error so retained raw results are not mistaken for current successful acquisition.
- Raw diagnostics report the latest complete result at telemetry cadence, not every ranging frame. Update every sender, the receiver, BLE `cmd_sensors`, and Python layouts together. BLE exposes the same raw range/status/age on robots without active ESP-NOW telemetry, with bounded paged replies if needed.

### Telemetry receiver & Python forwarder

Keep the existing receiver and wired serial connection at **460800 baud, 8N1**. Forward binary packets and generate JSON in Python. A queue absorbs bursts and isolates radio reception from serial output; it cannot compensate for a sustained bandwidth deficit.

```text
ESP-NOW callback -> fixed-size queue -> serial writer
                                            |
                                            v
                                  Python binary decoder
                                            |
                                            v
                                  JSON / console / PlotJuggler UDP
```

1. **Receive callback (`telemetry-receiver/src/main.cpp`)**:
   - Validate received length, header magic/type, `header.length == sizeof(TelemetryPacket)`, exact total frame length, and `pkt.protocol_version == TELEMETRY_PROTOCOL_VERSION`. Reject malformed/unsupported packets before copying their payload.
   - Copy each validated 192-byte frame, receiver arrival timestamp, and receiver sequence number into a preallocated FreeRTOS queue with `xQueueSend(..., 0)`. Start with 32 entries (6400 bytes of items before queue overhead). Never enqueue a pointer into the callback's temporary buffer.
   - Increment a receiver sequence number for every validated arrival, including arrivals dropped when the queue is full. On full queue, drop the incoming frame, increment an overflow counter, and return immediately. Never wait for space, format JSON, allocate memory, or write serial data in this callback.
   - Protect counters shared with the writer using atomics or short critical sections. Sequence gaps identify loss after receiver validation (queue overflow or subsequent serial/host loss); they do not measure ESP-NOW air loss without a robot-generated sequence number.
2. **Serial writer (`loop()` or one dedicated lower-priority task)**:
   - Drain the queue and write framed binary records using fixed buffers. This is the only application writer to the serial port; remove application text/JSON diagnostics from that byte stream. Handle partial writes before starting another record.
   - Replace the receiver's current `loop() { delay(10); }` with a writer that blocks on `xQueueReceive(queue, &frame, portMAX_DELAY)` when idle and processes queued records without a fixed delay per packet. A batch-based implementation must drain all available records before waiting again; one record followed by `delay(10)` cannot sustain four robots at 30 Hz. If using a dedicated writer task, an idle Arduino loop may still sleep independently.
   - Use COBS encoding followed by a `0x00` delimiter. The decoded serial envelope is, in order: `serial_version` (u8, initially 1), `receiver_sequence` (u32), `receiver_timestamp_us` (u32, captured on arrival), `receiver_queue_drops` (u32, cumulative overflow count sampled by the writer), `payload_length` (u16), original `PacketHeader + TelemetryPacket` bytes, and CRC-32 (u32).
   - All integer fields are little-endian; telemetry floats are IEEE-754 binary32. CRC-32 uses the CRC-32/ISO-HDLC variant compatible with Python `zlib.crc32`, covers all decoded bytes preceding the CRC, and is stored little-endian. Implement explicit serialization/decoding and size assertions rather than relying on native struct alignment on the host.
   - For protocol v1, the decoded record is `15 + 192 + 4 = 211` bytes, at most 213 bytes including COBS framing and delimiter. Four robots at 30 Hz require at most **25,560 bytes/s**, about 55.5% of the **46,080 bytes/s** UART capacity. Keep this budget updated when the packet layout changes.
   - Serial congestion may cause counted queue drops, but must never block radio reception or grow an unbounded backlog. Expose cumulative drops in subsequent records so they remain visible after the host resumes. Without hardware flow control, a stalled host can instead lose data in the USB-UART/host buffers while the receiver keeps transmitting; report serial decode errors and sequence gaps separately from receiver queue drops.
3. **Python forwarder (`telemetry-receiver/scripts/read_telemetry.py`)**:
   - Replace JSON `readline()` ingestion with bounded binary chunk reads and a streaming COBS decoder. Handle split/coalesced reads; reject oversized, malformed, bad-CRC, bad-length, or unsupported-version records. Discard through the next delimiter on framing errors and resume at the following record, including after boot text or a mid-record connection.
   - Decode the serial envelope and versioned telemetry with explicit little-endian layouts. Preserve receiver arrival time and robot time separately from host processing time; account for receiver timestamp wraparound and reset decoder/time tracking on reconnect.
   - Build the existing JSON field names and the new `sensors` array on the computer. Preserve robot filtering, console display, UDP destination options, and PlotJuggler namespacing. Format sensor health in the console, including `RANGE_INVALID` as unreliable distance rather than a bus fault, and forward all diagnostics (`health`, `age_ms`, `error_count`, `consecutive_errors`, `last_error`) plus receiver queue-drop/sequence-gap diagnostics. Include `raw_range_mm`, `raw_status`, `raw_internal_status`, and `raw_age_ms` for each distance slot in JSON/UDP and console diagnostics; represent missing raw records explicitly. Use these exported fields to record open-space, dark-wall, and close-range validation runs on the computer.
   - Rate-limit human-readable console output independently of packet decoding and UDP forwarding so terminal rendering does not throttle acquisition. A `--quiet` run must support all four robots at 30 Hz.

Native USB on a replacement receiver is not required for this data rate. Keep the existing hardware and baud rate as the rollout baseline.

## Configuration additions (`sniegbaltite/src/config.h`)

Add the following constants to namespace `config`:

```cpp
namespace config {
  // Sensor freshness limits
  constexpr uint32_t sensor_freshness_imu_us = 50000;    // 50 ms max age
  constexpr uint32_t sensor_freshness_tof_us = 100000;   // 100 ms max age
  constexpr uint16_t sensor_max_consecutive_errors = 3;

  // Recovery timings
  constexpr uint32_t sensor_recovery_delay_initial_ms = 500;
  constexpr uint32_t sensor_recovery_delay_max_ms = 5000;
  constexpr uint32_t sensor_recovery_healthy_ms = 2000;
  constexpr uint32_t sensor_bus_timeout_ms = 25;
  constexpr uint32_t tof_timeout_ms = 100;
  constexpr uint32_t tof_resume_grace_us = 100000; // No-result recovery only
  constexpr uint8_t imu_max_init_attempts_per_boot = 3; // Includes startup

  // IMU OFFLINE indication (also add to Rosepink's config)
  constexpr uint8_t imu_offline_led_r = 128;
  constexpr uint8_t imu_offline_led_g = 32;
  constexpr uint8_t imu_offline_led_b = 64;
  constexpr uint32_t imu_offline_blink_interval_ms = 250; // Each on/off phase

  // Initial stationary IMU calibration only
  constexpr uint16_t imu_min_calib_samples = 50;
  constexpr uint32_t imu_calib_extension_ms = 5000;

  // FreeRTOS tasks
  constexpr uint32_t sensor_task_stack_size = 4096;
  constexpr UBaseType_t sensor_task_priority = 2;
}
```

## Cross-robot migration scope

All robots are updated and reflashed as part of this rollout:

Every populated `SensorData` / `IMUSensorData` slot needs an acquisition adapter that updates values, timestamps, health, and error counters together according to the field semantics above. Renaming numeric field accesses alone is insufficient. Preserve last good values on failure; never stamp an error fallback as a successful sample. Initialize unused slots as `ABSENT` with timestamp zero. Adapt only measured distances to `.distance_mm`; numeric configuration thresholds remain unchanged.

Every robot also needs control-side availability integration. Evaluate freshness even when no new measurement arrives, and gate all distance-dependent steering, obstacle detection, corner anticipation, speed selection, and maneuver decisions. Retained last-good values are diagnostic data while unavailable, not permission to drive. Apply required-sensor gating during countdown and a zero-throttle override in every active maneuver, through the robot's existing actuator limiting path. Adding metadata alone does not complete migration.

### Bus ownership and synchronous recovery on other robots

- The named `main_sensor_task` / `side_tof_task` architecture and snapshot API describe Sniegbaltite. Across all robots, each I²C bus must have one exclusive owning task/context for acquisition, initialization, and recovery; do not add a recovery worker while leaving concurrent reads on that bus in `loop()`.
- Rosepink's main-bus worker must own both BMI088 and front/diagonal ToF operations, including ToF initialization/reinitialization. Move the corresponding `Wire` acquisition out of the control loop as part of that handoff. Its existing side-bus worker remains the exclusive side-bus owner. Keep the control loop responsive during blocking initialization through consistent snapshots.
- Nakts and Banana may retain a single-loop bus owner. Before every initial or repeated ToF initialization, configure the hardware bus timeout and call `setTimeout()` on the sensor before `init()`, including after reconstructing a driver or restarting the bus. Apply the same 500 ms, 1 s, 2 s, up-to-5 s recovery backoff; never retry initialization on every control tick. Nakts needs a runtime recovery path added; Banana's current immediate retry must be reordered.
- If a single-loop robot needs synchronous runtime recovery, mark the affected data unavailable and hold a recovery-pending motion inhibit before starting blocking work. Continue normal control ticks, apply the inhibit, and write actuator outputs while the existing slew limiter ramps down. Start recovery only after the actual commanded motor output(s) are zero and that zero has been written to the driver. A zero target or one deceleration update is not sufficient. On Banana, check both final wheel commands after steering mix and deadband handling, not just `current_throttle`.
- Once stopped and the retry deadline is due, perform one bounded recovery attempt after actuator output handling. During backoff, continue normal control, sensing of usable devices, and BLE handling. Resume motion only after the normal required-sensor gate passes and no pending blocking recovery would interrupt active control. This stopped-before-recovery rule is for synchronous control-loop recovery; S3 sensor workers retain the documented snapshot/freshness behavior.

| Robot | Required distance data for motion | Fallback |
| --- | --- | --- |
| Sniegbaltite / Rosepink | Fresh healthy front and at least one diagonal | Single-diagonal repulsion; omit unavailable side estimates; disable unavailable/uncalibrated IMU guidance |
| Nakts | Fresh healthy front and at least one diagonal | Single-diagonal repulsion; front loss or both diagonals lost inhibits motion |
| Banana | Fresh healthy front ToF; retain normal local analog IR acquisition/control | Front loss inhibits both drive motors; absent diagonal distance slots do not inhibit motion. Analog ADC readings have no inferred ToF health status |

1. **`sniegbaltite`**:
   - Implements full dual-task sensor architecture (`sensors.h` / `sensors.cpp`) with pre-seeded snapshot queues.
   - Populates all 8 diagnostic slots in `TelemetryPacket`.
   - Removes `Serial.begin()` and all `Serial.print*` calls in favor of ESP-NOW telemetry and BLE diagnostics.
2. **`rosepink`**:
   - Update measured `dist_*` usages in `main.cpp` to `.distance_mm`, and populate status metadata in front ToF and side VL53L5CX acquisition paths. Derived side distances inherit the actual source frame timestamp and validity, not the control tick time.
   - Update `state.imu` field accesses and driver calls to use `.measurement`, with checked acquisition updating the enclosing IMU status. Preserve the initial gyro biases and flat pitch across recovery and later countdowns here as well.
   - Apply the degraded control/readiness policy to all distance consumers, including file-local derived side estimates. A main-bus worker must own IMU and front/diagonal ToF acquisition and all blocking reinitialization, preserving exclusive bus ownership and keeping the control loop responsive. Accept the front/diagonal acquisition gap and resulting freshness-based motion inhibit while IMU initialization runs. Add the same three-call initialization budget per boot to Rosepink's configuration; if recovery is needed after exhaustion, disable the IMU until reboot.
   - Update `send_telemetry()`: sets `protocol_version = 1`, fills slots 0–2 (front ToFs) and slot 7 (IMU); side slots 3–6 mapped from VL53L5CX health status.
3. **`nakts`**:
   - Update measured `dist_*` usages in `main.cpp` to `.distance_mm`, and update front/left/right ToF status metadata on acquisition and failure.
   - Slots without sensors marked `SensorHealth::ABSENT`.
   - Implement freshness/readiness gating, one-diagonal fallback, and required-sensor motion inhibit in all active states. Failed or silent front acquisition must not leave `straight_throttle` enabled using a retained distance.
   - Add bounded runtime ToF recovery with timeouts set before initialization, exponential backoff, and the stopped-before-synchronous-recovery ordering above.
4. **`banana`**:
   - Adapt the front ToF to `state.dist_front.distance_mm` and maintain its status metadata. Clearing an old value during a maneuver must not manufacture a fresh successful measurement.
   - Move raw/filtered left and right analog IR readings and their filter state into `banana/src/main.cpp` local variables (file-local where persistence is needed), using ADC-specific names such as `ir_left_adc` / `ir_right_adc`. Update steering, obstacle detection, and serial debugging to use these variables directly, preserving the existing higher-ADC-means-closer behavior.
   - Do not store analog values in any `RobotState::SensorData`. Leave left/right distance slots and other unpopulated slots `ABSENT`, with zero timestamps and zero placeholder distances; the analog sensors remain available only to Banana's local logic and serial diagnostics.
   - Adapt the disabled telemetry code to the new protocol and `.imu.measurement` layout as well, without enabling it or putting ADC values in millimeter fields. Banana's required-sensor policy must reflect its actual front ToF plus local analog control; do not apply the S3 diagonal-distance readiness gate to its absent left/right distance slots.
   - Gate front-dependent obstacle/speed decisions by health and freshness. Apply front-loss motion inhibit to both final motor commands after mixing/deadband handling so minimum motor throttle cannot turn an inhibited command back into motion; retain smooth deceleration through the existing limiter.
   - Replace immediate per-tick ToF initialization with pending recovery: first ramp both motor outputs to zero and write them, then initialize with preconfigured timeouts and exponential backoff.
5. **`shared/src/ble/commands.cpp`**:
   - Update `cmd_dist` and `cmd_state` to access `.distance_mm`.
   - Add `cmd_sensors` to print all 8 sensor health statuses and error counts, including the distinct `RANGE_INVALID` quality state. Include raw VL53L0X range, full/internal status, and raw age from the copied state, using bounded paged replies within the BLE notification buffer.
   - Render `ABSENT` distance slots as unavailable rather than displaying zero as a measured distance. Banana's analog diagnostics stay in its local serial output.
6. **`telemetry-receiver`**:
   - Replace callback JSON output with the 32-entry receive queue and single binary serial writer. Implement the versioned COBS/CRC envelope at 460800 baud and update `read_telemetry.py` to decode it and generate JSON on the computer.

## Implementation phases

1. **Shared types, protocol, and checked drivers**:
   - Update `shared/src/state.h` (`SensorData`, `SensorStatus`, `RobotState`).
   - Update `shared/src/protocol.h` (`SensorDiagnostic`, `RawTofDiagnostic`, `TelemetryPacket` v1), updating all packet sizes and host layouts together.
   - Refactor `shared/src/driver/bmi088.h` and `.cpp` for checked sample acquisition, fusion from validated samples, explicit orientation/timing reset, and `set_gyro_biases(float bx, float by, float bz)` in rad/s. Retain synchronous initialization in the sensor-owner task; replace unchecked runtime acquisition and initial orientation seeding.
   - Add the pinned repository-managed VL53L0X fork with checked transport and host-only address adoption; update PlatformIO dependency selection on all affected robots. Keep provenance/license and exclude generated library copies from changes.
   - Update `shared/src/ble/commands.cpp`.
2. **Sensor tasks implementation (`sniegbaltite`)**:
   - Implement `sniegbaltite/src/sensors.h` and `sensors.cpp` with dual-task architecture and pre-seeded snapshot queues (no `Serial` prints).
   - Implement checked ToF reads, corrected raw-status classification, `RANGE_INVALID` handling without reset loops, separate result-cadence/usable-distance timers, non-XSHUT address probing (`0x30` before `0x29`), and bus recovery GPIO pulsing with `OUTPUT_OPEN_DRAIN`.
   - Implement IMU fusion, orientation generation, initial calibration command results, and committed calibration storage independent of driver lifetime. Use `BMI088Driver::set_gyro_biases` to restore stored biases without invoking calibration.
   - Publish failures before recovery and implement bounded recovery operations, command retries/deduplication, and mandatory delay on scheduling overruns. Allow blocking IMU reset/self-test/settling waits in the main sensor task. Resume ToF acquisition and pending command processing after each attempt; enforce retry backoff and freshness-based motion inhibit during acquisition gaps. Enforce the three-call IMU initialization budget per boot, including successful calls; latch it disabled when further recovery is required after exhaustion. Preserve ToF backoff across brief successes and apply the bounded owner-pause grace without changing measurement freshness.
3. **Control loop integration (`sniegbaltite/src/main.cpp`)**:
   - Remove legacy ToF task, `Wire` calls, `Serial.begin()`, and all `Serial.print*` calls from `main.cpp`.
   - Integrate `get_main_snapshot()` and `get_side_snapshot()`.
   - Implement degraded control policy (including single-diagonal steering fallback), slew-limited zero-target-throttle override (`state.target_throttle = 0.0f` ramped via `throttle_limiter`), inhibit entry/exit with maneuver cancellation and PID history reset, initial calibration deadline, and countdown readiness gate. Preserve committed flat pitch across orientation-generation changes.
   - Add the priority pink IMU `OFFLINE` indication through the existing asynchronous LED task, with one owner for all RGB LED writes.
   - Update `send_telemetry()`.
4. **Adapter updates for remaining targets**:
   - Adapt `rosepink`, `nakts`, `banana` acquisition/status paths and every control consumer to the new `RobotState`. Implement their required-sensor readiness gates, degraded steering, and motion inhibits; move Banana's analog readings to local ADC variables. Keep Rosepink's blocking IMU recovery in a sensor-owner worker with the same accepted acquisition-gap behavior.
   - Give Rosepink's main-bus worker exclusive ownership of front/diagonal ToF acquisition/recovery too. Add/reorder Nakts and Banana runtime recovery so outputs reach zero before blocking initialization, with timeouts configured first and retries governed by backoff.
   - Extend Rosepink's LED task/configuration with the same pink IMU `OFFLINE` indication and priority rules as Sniegbaltite.
   - Implement the receiver's bounded callback queue, overflow accounting, and single COBS/CRC binary serial writer. Update `read_telemetry.py` for streaming binary decoding and host-generated JSON while preserving its existing user options.
5. **Verification**:
   - Build all targets with `pio run`.
   - Validate live telemetry, fault injection (unplugging sensors), and recovery transitions on hardware.
   - Measure failure-publication-to-inhibit latency and disconnect-to-inhibit latency separately, including a front failure immediately followed by blocking recovery. Exercise inhibition in `RUNNING`, `REVERSE`, and `REVERSE_TURN`; verify `target_throttle` drops to zero on the next tick and `current_throttle` ramps smoothly down and back up via `throttle_limiter` without abrupt ESC braking.
   - Inject repeated device/bus restarts and IMU acquisition gaps while moving and on an incline. Verify committed gyro biases and flat pitch stay unchanged, no calibration samples are accumulated, and yaw resets do not trigger false wrong-way recovery. Repeat with BLE stop/start after initial calibration.
   - Force blocking IMU-only restart/self-test sequences on Sniegbaltite and Rosepink. Record initialization duration and front/diagonal sample age. Verify the control loop and side acquisition remain responsive, unchanged distance timestamps become stale after 100 ms, motion is slew-limited to zero, and fresh required distances restore readiness after acquisition resumes. Verify a failed IMU attempt enters backoff with ToF acquisition and command processing running between attempts. Separately inject a stuck shared bus and verify bus recovery.
   - Keep the IMU unavailable through three initialization attempts, including a case where initialization succeeds but checked samples never arrive. Verify it becomes `OFFLINE`, racing continues on fresh required distances, and no further IMU traffic/retry pauses occur until MCU reboot, including after BLE stop/start and shared-bus recovery. Repeat at boot and verify calibration cancellation is acknowledged before degraded driving begins.
   - Alternate brief IMU success with renewed failure and include shared-bus recovery: verify the count never resets, at most three initialization calls occur per boot, a successful third call continues operating, and a subsequent recovery need latches it off. For ToF device and bus recovery, inject isolated good results between failures and verify backoff advances toward 5 s rather than resetting to 500 ms; only 2 s of sustained acquisition resets it.
   - After blocking IMU initialization, verify ready ToF results are consumed without false timeout/reset. Repeat with a responsive but silent ToF: it must time out by the bounded resume-grace deadline. Repeated scheduling overruns must not extend that deadline; existing transport faults must still escalate immediately. Check that usable-distance timestamps and the 100 ms control freshness gate are unchanged throughout.
   - Interrupt each timed maneuver with sensor-loss inhibit longer than its normal duration. Verify it is canceled, no new maneuver starts while inhibited, PID/PD history does not accumulate, and resume uses fresh inputs/debounce without a derivative kick or expired maneuver transition. Check BLE stop during inhibit, countdown/calibration deadlines, and smooth throttle output through both edges.
   - Verify pink blinking starts when IMU health becomes `OFFLINE` on both IMU robots, uses 250 ms on/off phases, and remains visible during racing, idle, countdown, BLE stop/start, and simultaneous ToF faults. Verify transient non-`OFFLINE` states do not select pink and successful recovery before retry exhaustion restores the normal LED mode.
   - Inject NACK/short-read failures specifically into BMI088 sample transactions and ToF ready/status/range/interrupt-clear transactions. Verify no failed sample reaches fusion/calibration or advances its timestamp. Exercise non-XSHUT recovery at both `0x30` and `0x29`, checking that host-address adoption emits no I²C address write.
   - Verify ToF raw-status mapping with controlled input, including minimum-range failures (8/10), no-result status 7, hardware failures (1/2/3), and 8190/8191 sentinels. Repeated quality failures must show `RANGE_INVALID`, preserve the last usable timestamp, and apply unavailable-sensor control without incrementing hardware/transport fault counts or starting recovery. Distinguish complete but unreliable results from a sensor that never produces a result. Exercise the real sensors in open space, near dark/angled walls, strong light, and too-close conditions before track rollout.
   - Compare checked BMI088 outputs with the existing driver during known tilts/rotations. Verify both sample types retain `(X, -Y, -Z)` mapping with the existing gyro rad/s-to-deg/s conversion applied exactly once; build all targets with the BolderFlight compatibility guards still present when initialization uses that dependency.
   - On Nakts and Banana, inject a ToF failure while motor outputs are nonzero and trace actuator writes versus initialization entry. Verify multiple deceleration ticks run until all motor outputs are zero, zero is written before `init()`, bus/sensor timeouts are already set, and failed attempts respect backoff. Verify Rosepink's main bus has no control-loop I²C access after worker ownership is introduced.
   - Exercise insufficient initial samples, queue-full sends, repeated command IDs, abort during a pending command, and calibration deadline expiry. Verify acknowledged cancellation precedes degraded driving and that later recovery never starts calibration while moving.
   - Force repeated acquisition/recovery overruns; verify tasks block between bounded steps, do not run catch-up bursts, leave time for the watchdog idle task, and maintain safe stack high-water marks (`uxTaskGetStackHighWaterMark()`).
   - Verify populated sensor slots advance sample timestamps and report faults on every robot; verify Banana retains its analog steering behavior and reports unused distance slots as `ABSENT` without ADC values in distance telemetry.
   - On each robot, disconnect the front sensor after a clear-track sample and separately stop measurement production while leaving the device responsive. Verify countdown cannot start and active motion cannot continue on cached distance values. On Sniegbaltite, Rosepink, and Nakts, verify one-diagonal fallback, both-diagonal inhibit, and exclusion of stale side estimates. On Banana, verify both motor outputs reach zero despite the motor deadband and absent diagonal slots do not prevent normal starts.
   - Verify binary encoding/decoding with shared known-byte fixtures (field order, little-endian numbers, CRC, and COBS), fragmented/coalesced input, corrupt/truncated/oversized records, unsupported versions, boot text, and reconnects. The decoder must recover at subsequent valid frame boundaries without unbounded buffering.
   - Include distinct left/right raw ranges and status bytes in shared fixtures. Verify raw age advances independently of usable-distance age, 8190/8191 survive unchanged, partial reads preserve the prior raw record, and absent/non-VL53L0X slots show no raw sample. Capture real open-space/dark-wall/close-range results through Python and compare with BLE diagnostics; note that 30 Hz telemetry samples the latest ranging result rather than logging every frame.
   - Feed the receiver four robot streams at 30 Hz with full sensor diagnostics at 460800 baud. Require no sustained queue growth or receiver queue drops during normal host consumption. Pause/slow the host and force serial congestion to exercise overflow; verify the callback remains nonblocking, drop counts are reported after resumption, and decoding/PlotJuggler forwarding recover. Measure queue high-water mark and end-to-end latency using receiver arrival timestamps rather than only host dequeue time.

## Acceptance criteria

- On Sniegbaltite, every I²C call on `Wire` runs exclusively in `main_sensor_task`; every I²C call on `Wire1` runs exclusively in `side_tof_task`. Across all robots, each bus has a single exclusive owning task/context; any setup-to-worker handoff completes before concurrent tasks start using it.
- All `Serial` logging is removed from `sniegbaltite` (`main.cpp` and `sensors.cpp`), relying on ESP-NOW telemetry and BLE for live debugging.
- A fault or re-init on `Wire1` does not restart `Wire`, make its owner wait for side recovery, or cause main-bus samples to exceed their freshness limits. Measure and record main-task cadence and scheduling jitter during side-bus fault injection.
- Blocking IMU initialization/recovery is allowed on Sniegbaltite and Rosepink and may pause front/diagonal acquisition for approximately 400 ms or longer. The control loop stays responsive, cached distances expire at 100 ms, and temporary slew-limited motion inhibit is accepted. Side acquisition continues independently; main-bus distance reads and command processing resume after each attempt, including failed attempts during retry backoff.
- IMU initialization runs at most three times per MCU boot, including startup and successful recovery calls. A successful final attempt stays usable; failure of that attempt or a later need for recovery latches `OFFLINE` until reboot. Both IMU robots continue racing when required distances are available, with wrong-way detection and slope hold disabled and no further IMU retry pauses. Pending calibration cancellation is still acknowledged before initial degraded driving.
- An `OFFLINE` IMU produces a visible pink blink (250 ms on / 250 ms off) on Sniegbaltite and Rosepink, overriding other LED modes without blocking control or acquisition. After retry exhaustion, the indication persists until reboot, including while racing and after BLE stop/start.
- Sniegbaltite's 100 Hz control loop never blocks on I²C and always peeks a consistent snapshot (pre-seeded as `INITIALIZING` at `sensors::init()`). Rosepink's main-bus worker similarly keeps acquisition/recovery outside control. Single-loop Nakts/Banana recovery begins only after all commanded motor outputs are zero and written; finite timeouts and backoff bound those stopped recovery attempts.
- Sensor disconnects or bus stalls cannot be mistaken for clear space on any robot (`distance_mm` is gated by availability checks at every control consumer). Each robot enforces its required-sensor policy during countdown and all active maneuvers; retained clear-track samples never authorize continued motion.
- Valid distances beyond the configured control range clamp to its maximum with `health = OK`. Unreliable signal/phase/minimum-range results and 8190/8191 sentinels are never blanket-classified as clear space: they remain unavailable to control, without inducing hardware reset loops solely for poor measurement quality.
- A published front ToF failure (or loss of both diagonals) inhibits driving by the next control tick (nominally 10 ms plus measured scheduling jitter); stalled acquisition inhibits driving on the next tick after the 100 ms freshness deadline. `state.target_throttle` is immediately set to zero while `state.current_throttle` ramps smoothly down via `throttle_limiter` to prevent harsh ESC braking. Loss of a single diagonal uses one-sided wall repulsion. Side ToF or IMU loss does not directly inhibit driving, but blocking IMU recovery may indirectly inhibit it when required distance samples expire.
- Initial committed gyro biases and flat pitch remain unchanged through repeated sensor/bus recovery and subsequent countdowns. Orientation discontinuities reset heading references, never calibration values. No calibration runs after driving has begun.
- Known owner pauses do not falsely reset healthy ToFs: ready results are consumed first and any no-result grace is bounded to 100 ms after return. Scheduling overruns cannot keep extending it, and measurement freshness never receives grace. ToF recovery backoff resets only after 2 s of sustained acquisition, not one good sample.
- Sensor-loss inhibit cancels active timed maneuvers, suppresses new maneuver decisions and PID accumulation, and resumes from fresh inputs with reset history. Existing smooth throttle limiting, BLE stop, countdown deadlines, and calibration acknowledgement requirements remain effective.
- Initial calibration either completes with a matching acknowledgement or is explicitly aborted and acknowledged before degraded driving; command failures and stale completion flags cannot bypass readiness gating.
- Full sensor diagnostics (health, age, error counts, raw VL53L0X range/status/raw age) are visible in ESP-NOW telemetry, `read_telemetry.py`, and BLE `cmd_sensors`. Raw results remain distinct from usable distances, including retained results after transport failure and explicit no-raw-record markers for non-VL53L0X slots.
- Four robot streams at 30 Hz fit the existing receiver's 460800-baud link using queued binary serial transport. JSON formatting runs on the computer. The writer waits on the queue when idle and does not impose a per-record 10 ms delay. The ESP-NOW callback never performs serial writes or waits for queue space; congestion causes bounded, counted drops and the decoder recovers from malformed/partial records.
- Checked driver transport failures prevent measurement/fusion/calibration updates. Driver changes and dependency pins are reproducible from the repository, with no edits required under `.pio/libdeps`.
- No watchdog resets occur during sensor or bus recovery, including repeated overruns; every acquisition cycle and bounded recovery step allows a blocking yield.
- Banana's analog IR values remain local ADC readings and never populate distance `SensorData` or millimeter telemetry fields; unused distance slots report `ABSENT`.
- All four robot firmware targets and `telemetry-receiver` compile cleanly.
