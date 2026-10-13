"""Bounded COBS/CRC serial v1 decoder. All wire layouts are little-endian."""
import math
import struct
import zlib

ROBOTS = {1: "sniegbaltite", 2: "rosepink", 3: "banana", 4: "nakts"}
STATES = ("IDLE", "COUNTDOWN", "RUNNING", "REVERSE", "REVERSE_TURN", "WRONG_WAY_RECOVERY")
HEALTH = ("ABSENT", "INIT", "OK", "STALE", "ERROR", "OFFLINE", "RECOVER", "RANGE_INVALID", "NO_RETURN")
ERRORS = ("NONE", "NACK", "TIMEOUT", "SHORT_READ", "BUS_ERROR", "INIT_FAILED", "CONFIG_FAILED", "MEASUREMENT_TIMEOUT", "HARDWARE_ERROR")
SLOTS = ("front", "right", "left", "side_fl", "side_rl", "side_fr", "side_rr", "imu")
FLOATS = ("target_steering", "current_steering", "target_throttle", "current_throttle",
          "pid_error", "pid_integral", "pid_derivative", "accel_x", "accel_y", "accel_z",
          "gyro_x", "gyro_y", "gyro_z", "roll", "pitch", "yaw")


def cobs_decode(encoded):
    result = bytearray()
    offset = 0
    while offset < len(encoded):
        code = encoded[offset]
        offset += 1
        if not code or offset + code - 1 > len(encoded):
            raise ValueError("invalid COBS run")
        result.extend(encoded[offset:offset + code - 1])
        offset += code - 1
        if code < 255 and offset < len(encoded):
            result.append(0)
    return bytes(result)


def decode_record(encoded):
    raw = cobs_decode(encoded)
    if len(raw) != 211:
        raise ValueError("serial length")
    if zlib.crc32(raw[:-4]) != struct.unpack_from("<I", raw, 207)[0]:
        raise ValueError("CRC")
    version, sequence, arrival, drops, length = struct.unpack_from("<BIIIH", raw)
    if version != 1 or length != 192:
        raise ValueError("serial version/length")
    magic, kind, robot, packet_length = struct.unpack_from("<IBBH", raw, 15)
    if magic != 0x4B4C4F46 or kind != 0 or packet_length != 184:
        raise ValueError("radio header")
    p = memoryview(raw)[23:207]
    version, time_ms, state, flags = struct.unpack_from("<BIBB", p)
    if version != 1:
        raise ValueError("telemetry version")
    data = {"robot": ROBOTS.get(robot, "unknown"), "robot_id": robot,
            "protocol_version": version, "time": time_ms, "robot_time_ms": time_ms,
            "state_id": state, "state": STATES[state] if state < len(STATES) else "UNKNOWN",
            "flags": flags, "all_required_sensors_ok": bool(flags & 8),
            "receiver_sequence": sequence, "receiver_timestamp_us": arrival,
            "receiver_queue_drops": drops}
    data.update(zip(("dist_" + name for name in SLOTS[:7]), struct.unpack_from("<7H", p, 7)))
    # Invalid floats cannot produce non-standard JSON or break a UDP consumer.
    values = struct.unpack_from("<16f", p, 21)
    data.update(zip(FLOATS, (v if math.isfinite(v) else None for v in values)))
    data["accel"] = [data[n] for n in FLOATS[7:10]]
    data["gyro"] = [data[n] for n in FLOATS[10:13]]
    data["euler"] = [data[n] for n in FLOATS[13:16]]
    sensors = []
    for i, name in enumerate(SLOTS):
        health, age, errors, streak, error = struct.unpack_from("<BHHHB", p, 85 + i * 8)
        item = {"slot": name, "health": health,
                "health_name": HEALTH[health] if health < len(HEALTH) else "UNKNOWN",
                "age_ms": None if age == 65535 else age, "error_count": errors,
                "consecutive_errors": streak, "last_error": error,
                "last_error_name": ERRORS[error] if error < len(ERRORS) else "UNKNOWN"}
        if i < 7:
            distance, status, raw_age = struct.unpack_from("<HBH", p, 149 + i * 5)
            present = raw_age != 65535
            item.update(raw_range_mm=distance if present else None,
                        raw_status=status if present else None,
                        raw_internal_status=(status & 0x78) >> 3 if present else None,
                        raw_age_ms=raw_age if present else None)
        sensors.append(item)
    data["sensors"] = sensors
    return data


class StreamDecoder:
    """At most 212 encoded bytes buffered; discard overflow through delimiter.

    Sequence gaps are receiver/serial/host loss, not radio-air loss. A backwards
    sequence or receiver-clock jump starts a new receiver epoch. Reconnect creates
    a new decoder; timestamps within an epoch use receiver arrival, not host reads.
    """
    def __init__(self):
        self.buffer = bytearray()
        self.discard = False
        self.decode_errors = 0
        self.sequence_gaps = 0
        self.previous_sequence = None
        self.previous_arrival = None
        self.elapsed_us = 0
        self.epoch = 0

    def feed(self, chunk):
        packets = []
        for value in chunk:
            if value:
                if not self.discard:
                    if len(self.buffer) == 212:
                        self.buffer.clear()
                        self.discard = True
                        self.decode_errors += 1
                    else:
                        self.buffer.append(value)
                continue
            if self.discard:
                self.discard = False
                continue
            if not self.buffer:
                continue
            try:
                data = decode_record(self.buffer)
            except ValueError:
                self.decode_errors += 1
            else:
                seq = data["receiver_sequence"]
                arrival = data["receiver_timestamp_us"]
                if self.previous_sequence is not None:
                    delta = (seq - self.previous_sequence) & 0xFFFFFFFF
                    dt = (arrival - self.previous_arrival) & 0xFFFFFFFF
                    if delta == 0 or delta >= 0x80000000 or dt >= 0x80000000:
                        self.epoch += 1
                        self.elapsed_us = 0
                    else:
                        self.sequence_gaps += delta - 1
                        self.elapsed_us += dt
                self.previous_sequence = seq
                self.previous_arrival = arrival
                data.update(receiver_time_s=self.elapsed_us / 1e6,
                            receiver_epoch=self.epoch,
                            receiver_sequence_gaps=self.sequence_gaps,
                            serial_decode_errors=self.decode_errors)
                packets.append(data)
            finally:
                self.buffer.clear()
        return packets
