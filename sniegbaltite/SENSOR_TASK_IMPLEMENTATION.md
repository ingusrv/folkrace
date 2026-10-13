# Sensor tasks — all-project rollout

Implemented on `revolution` for Sniegbaltite, Rosepink, Nakts, Banana, and the
telemetry receiver/Python reader. All targets now use the same SensorData and
telemetry v1 layouts; the temporary FOLKRACE_SENSOR_TASKS switch is removed.

## Implemented

- Separate core-0, priority-2 owners for Wire (BMI088/front/diagonals) and Wire1
  (four side ToFs), with pre-seeded length-one snapshot queues and nonblocking
  control-side peeks. All I2C, initialization, calibration and recovery belong
  to the owning task. Task/queue creation failure inhibits autonomous motion.
- Checked register writes and byte-count-validated reads, sticky first ToF
  transaction errors, raw result retention, range-quality classification,
  measurement timestamps and independent result-cadence timers. The Pololu
  fork is stored in shared/vendor, with revision and license; generated
  dependencies are not modified. BolderFlight initialization is pinned to
  1.0.1; checked runtime acquisition preserves its (X, -Y, -Z) axes and units.
- Device/group/bus recovery, retained-address probing, GPIO clock recovery,
  capped exponential backoff and a two-second sustained-health reset window.
  Known blocking pauses receive bounded no-result grace without extending
  control freshness. Tasks yield between recovery steps and after overruns.
- Lifetime three-call IMU initialization budget, including successful calls.
  Exhaustion latches OFFLINE until reboot. Calibration commands remain live
  even after exhaustion. Initial biases and flat pitch survive recovery;
  orientation-generation changes reset heading references without recalibration.
- Ordered calibration command IDs, matching acknowledgements, queue-full retry,
  explicit cancellation, and the bounded initial stationary calibration window.
  Subsequent countdowns never recalibrate after motion was permitted.
- Front + at least one diagonal readiness, one-wall steering fallback,
  unavailable side exclusion, optional IMU guidance, maneuver cancellation and
  PID reset on inhibition, steering hold and slew-limited throttle deceleration.
- One independent LED owner; OFFLINE pink (250 ms phases) overrides countdown
  amber and ToF recovery indications. No Serial calls in main.cpp/sensors.cpp.
- Telemetry v1 (184-byte payload / 192-byte radio frame), health/raw diagnostics,
  BLE `sensors [0..7]` and `taskstats`.

## Other robots and receiver

- Rosepink uses the same main-bus owner, initial calibration handshake, IMU
  retry budget, motion gate, and asynchronous LED indications as Sniegbaltite.
  A separate Wire1 owner publishes derived front/rear side estimates with the
  source frame timestamp and quality. Its checked SparkFun 1.0.3 fork retains
  driver storage across retries, checks short reads/writes, preserves the first
  error, bounds polling, and yields during firmware upload. Empty/unreliable
  frames are RANGE_INVALID; wrapped-target status 6 is excluded. Valid targets
  beyond the control range remain valid and derived distances clamp to range.
- Nakts owns I2C in its control loop and supports front-plus-one-diagonal
  readiness and single-wall fallback. Banana keeps analog IR ADC/filter values
  local and marks unused distance/IMU slots ABSENT. Its required sensor is the
  front ToF. Existing disabled ESP-NOW transmission remains disabled on Banana;
  Nakts remains BLE/serial only.
- Both C3 robots latch inhibition when a recovery attempt is due, continue
  actuator updates until zero is written, then perform bounded synchronous
  recovery. Banana limits both final wheel commands after mixing and deadband,
  including pivot inhibition and smooth restart. Retry backoff does not stall
  the control loop or prevent sensing usable devices.
- The receiver validates exact 192-byte radio frames and v1 before copying to
  its 32-item static queue. Overflow drops the incoming record and increments
  a counter. The callback never writes serial or waits for queue space. A single
  writer blocks on the empty queue, handles partial writes, and emits COBS/CRC32
  records with sequence, arrival time, and queue-drop count at 460800 baud.
- Python decodes bounded binary chunks, rejects malformed/unsupported records,
  resynchronizes at delimiters, and forwards all diagnostics as JSON. Existing
  CLI/UDP/filter/namespace options remain. Console output is limited to 5 Hz per
  robot; decoding and UDP forwarding continue for every record. Receiver time,
  epoch, sequence gaps, decode errors, and host time are separate diagnostics.

## Telemetry compatibility

Firmware v1 and the new binary receiver/reader must be deployed together.
The old JSON-line reader is incompatible. No devices have been flashed as part
of the source changes. A full frame is 192 bytes, a decoded serial record is
211 bytes, and COBS framing adds at most two bytes: four 30 Hz streams require
25,560 bytes/s of the 46,080 bytes/s 8N1 UART capacity. This is the calculated
budget; hardware congestion/queue behavior remains to be measured.

Run `python3 ../telemetry/scripts/read_telemetry.py PORT 460800`
with pyserial installed. `--quiet` suppresses per-packet console formatting.

`sensors` sends one bounded notification per slot; `sensors 0` requests just
front. `age` is last usable sample age; `raw_age` is last complete acknowledged
VL53L0X result age. 65535 means no sample. `last_mm` and raw values are retained
for diagnostics after failures and never authorize driving. `RANGE_INVALID`
is unreliable ranging, distinct from a transport/hardware error.

`taskstats` reports each task's minimum free stack bytes, maximum iteration
execution time and start-to-start period, control-loop maximum period, and
last blocking IMU initialization duration. Maxima include boot/recovery;
reboot between comparative fault tests. These counters are instrumentation,
not measured hardware acceptance results.

## Verification

Run from Sniegbaltite:

```sh
pio run
sh tests/host/run.sh
python3 ../telemetry/tests/test_decoder.py
```

Host tests use the real policy, startup handshake, owners, and checked drivers
with deterministic bus/clock/vendor-initializer doubles. They cover quality
statuses/sentinels, metadata retention/saturation, freshness/wraparound,
single-diagonal fallback, smooth inhibit in all maneuver states, command
retry/cancellation/matching, IMU budget across brief successes, recovery grace,
backoff, and checked NACK/short-read transactions and axis scaling.
Additional tests exercise the actual VL53L5CX transport, bounded silent-device
initialization, retained storage, side-fit quality, Banana pivot deceleration
and resume, and a C++-generated serial fixture decoded in Python. Serial tests
cover every split position, coalesced input, corruption, unsupported versions,
oversized records, boot text, wraparound, reboot/reconnect, and 1200 records
representing ten seconds at four robots × 30 Hz. They cannot establish sensor
electrical behavior or RTOS scheduling margins.

Hardware validation remains pending: disconnect and silent-sensor injection,
publication-to-inhibit versus physical disconnect latency, motor deceleration
travel, independent bus faults, actual BMI088 sign comparison, initial
calibration/restarts on inclines, pink LED priority, stack margins/watchdog
behavior under repeated recovery, and open-space/dark-wall/too-close ranging.
The unchanged 100 ms distance freshness rule stops on stale or unavailable
front returns. See the Sniegbaltite open-space policy below for the explicit
exception for observed no-return sentinels. Validate on the track
before adopting any more permissive quality policy.

All five PlatformIO targets and the host suites are required rollout checks.
Receiver congestion, physical wheel stop ordering, side-sensor firmware-upload
recovery, and task stack/watchdog measurements still require hardware.

## Sniegbaltite open-space racing policy (2026-10-13)

Live receiver capture confirmed that an open straight produces fresh checked
front/diagonal results of 8191 with internal status 4 and 8190 with status 6,
without transport errors. Treating each as unavailable made the car stop and
blink purple. Sniegbaltite now opts into `config::tof_allow_no_return`: only
status 4/6 combined with the exact 8190/8191 sentinel becomes `NO_RETURN` (health
value 8). The control distance is assumed to be sensor_max_raw and its timestamp
advances with each complete checked/acknowledged result. BLE and Python preserve
the distinct quality label and unchanged raw range/status. Packet sizes and
protocol version remain unchanged.

This is a racing assumption, not proof of open space: a difficult nearby target
can produce the same weak return. Hardware wall-detection validation is still
required. Other robots retain strict rejection by default. Non-sentinel weak
results, minimum-range/unknown statuses, hardware/transport errors and silent or
stale samples remain unavailable. A no-return sample expires at the same 100 ms
limit and does not authorize driving after disconnect. Normal valid samples
immediately restore OK with the measured distance.

## Sniegbaltite pole avoidance and recovery — revised 2026-10-14

The CSV `../telemetry/scripts/csv/2026-10-14_00-27-08.csv` exposed an overly
persistent steering floor: at 16003 ms the front measured 239 mm, the left side
31 mm, and the right side no-return/max range, yet steering was -0.984 toward
the left wall. At 16533 ms reverse still followed the old passing direction
instead of the newly available space. The previous 650 mm onset and 200 ms
post-obstacle/post-reverse steering hold are superseded by these changes.

Front avoidance starts at 550 mm and reaches full demand at 180 mm. One passing
pulse is bounded to 180 ms or 18 degrees of continuous, calibrated IMU yaw.
Without usable IMU continuity, the time bound remains. A still-close front does
not renew the pulse; ordinary wall following takes control. Front clearance
releases the pulse immediately, with 50 mm rearming hysteresis. There is no
post-obstacle or post-reverse forced steering tail. Corner throttle reduction
still uses current front proximity, and open-space NO_RETURN behavior remains.

Fresh side-wall pressure takes priority before and after PID, including during
a pulse. Front anticipation is removed from the baseline shared guidance for
Sniegbaltite so that its unbounded blend cannot defeat the pulse limit or wall
repulsion. At recovery entry the passing direction is recomputed from current
fresh diagonal/side clearances; an old pole latch cannot force recovery toward
a newly blocked wall.

Normal recovery still uses up to 200 ms straight backing after the reverse
command starts, then up to 250 ms turning, with front-clearance and 30-degree
yaw exits. A 900 ms overall bound includes the throttle ramp. Steering centers
while the commanded throttle is still negative when forward driving resumes;
PID/debounce history resets, and sensor-loss inhibition cancels the maneuver.
These are command timers and front-clearance checks, not measured travel distance.

The recording also established Sniegbaltite's yaw convention: right steering
produced positive gyro_z (median +108.8 deg/s), and left steering negative
(median -116.3 deg/s) during forward RUNNING samples with |steering| > 0.5 and
throttle > 0.15. CW progress therefore advances the yaw maximum and CCW the
minimum. Wrong-way deviation and turnaround steering now use this convention.
The INPUT_PULLUP switch maps on/LOW to CLOCKWISE and off/HIGH (default) to
COUNTER_CLOCKWISE, both at boot and during live direction changes.
Previously the first normal left-turn sequence (5433–11433 ms, yaw -0.1 to
-227.5 degrees) incorrectly accumulated a wrong-way deviation.

`tests/host/navigation_test.cpp` covers recorded wall-conflict/escape fixtures,
mirrored wall priority, pulse yaw/time limits, no tail or repeated pulse on a
continuously close front, stale-side exclusion, distinct clearance frames,
reverse timing/IMU discontinuities, timestamp wraparound, normal CW/CCW yaw
progress, and genuinely opposite heading detection. On-track trajectories and
physical overshoot still require validation after reflashing. Navigation changes
are scoped to Sniegbaltite; other robots need their own yaw-sign validation.

### Earlier onset adjustment from the 00:53:47 recording

The top portion of `../telemetry/scripts/csv/2026-10-14_00-53-47.csv` shows
front distance falling from 587 mm at 6758 ms to 483 mm at 6788 ms. Following
the user's successful track test, onset increased from 500 to 550 mm for a
modest earlier response. The 180 ms / 18-degree pulse limits, wall priority,
180 mm full-demand threshold, and reverse behavior retain their previous tuning.
