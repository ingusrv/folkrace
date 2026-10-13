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

Run `python3 ../telemetry-receiver/scripts/read_telemetry.py PORT 460800`
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
python3 ../telemetry-receiver/tests/test_decoder.py
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
The unchanged 100 ms distance freshness rule deliberately stops on unreliable
front returns, even when those occur on an open straight. Validate on the track
before adopting any more permissive quality policy.

All five PlatformIO targets and the host suites are required rollout checks.
Receiver congestion, physical wheel stop ordering, side-sensor firmware-upload
recovery, and task stack/watchdog measurements still require hardware.
