#!/usr/bin/env python3
"""
Folkrace Telemetry Serial Reader & PlotJuggler UDP Forwarder
Decodes COBS/CRC binary records from the ESP32 receiver and forwards
host-generated JSON datagrams to PlotJuggler via UDP.
"""

import argparse
import atexit
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import socket
import sys
import time
from telemetry_decoder import SLOTS, StreamDecoder


ROBOT_COLORS = {
    "sniegbaltite": "\033[96m",  # Bright Cyan
    "rosepink": "\033[95m",      # Bright Magenta
    "banana": "\033[93m",        # Bright Yellow
    "nakts": "\033[94m",         # Bright Blue
}
COLOR_RESET = "\033[0m"

ROBOT_PORT_OFFSETS = {
    "sniegbaltite": 0,
    "rosepink": 1,
    "banana": 2,
    "nakts": 3,
}

CSV_BASE_FIELDS = [
    "timestamp",
    "host_time_s",
    "robot",
    "robot_id",
    "protocol_version",
    "time",
    "robot_time_ms",
    "state",
    "state_id",
    "flags",
    "flag_obstacle_detected",
    "flag_ble_connected",
    "flag_clockwise",
    "flag_sensors_ok",
    "flag_slope",
    "receiver_sequence",
    "receiver_timestamp_us",
    "receiver_queue_drops",
    "receiver_time_s",
    "receiver_epoch",
    "receiver_sequence_gaps",
    "serial_decode_errors",
    "dist_front",
    "dist_right",
    "dist_left",
    "dist_side_fl",
    "dist_side_rl",
    "dist_side_fr",
    "dist_side_rr",
    "target_steering",
    "current_steering",
    "target_throttle",
    "current_throttle",
    "battery_voltage_v",
    "pid_error",
    "pid_integral",
    "pid_derivative",
    "accel_x",
    "accel_y",
    "accel_z",
    "gyro_x",
    "gyro_y",
    "gyro_z",
    "roll",
    "pitch",
    "yaw",
]

CSV_SENSOR_FIELDS = []
for slot in SLOTS:
    CSV_SENSOR_FIELDS.extend([
        f"sensor_{slot}_health",
        f"sensor_{slot}_health_name",
        f"sensor_{slot}_age_ms",
        f"sensor_{slot}_error_count",
        f"sensor_{slot}_consecutive_errors",
        f"sensor_{slot}_last_error",
        f"sensor_{slot}_last_error_name",
    ])
    if slot != "imu":
        CSV_SENSOR_FIELDS.extend([
            f"sensor_{slot}_raw_range_mm",
            f"sensor_{slot}_raw_status",
            f"sensor_{slot}_raw_internal_status",
            f"sensor_{slot}_raw_age_ms",
        ])

CSV_FIELDNAMES = CSV_BASE_FIELDS + CSV_SENSOR_FIELDS


def format_csv_value(val):
    if val is None:
        return ""
    if isinstance(val, float):
        if math.isnan(val) or math.isinf(val):
            return ""
        s = f"{val:.2f}"
        if s == "-0.00":
            s = "0.00"
        return s
    return val


def telemetry_to_row(data):
    flags = data.get("flags", 0) or 0
    row = {
        "timestamp": format_csv_value(data.get("timestamp")),
        "host_time_s": format_csv_value(data.get("host_time_s", data.get("timestamp"))),
        "robot": format_csv_value(data.get("robot")),
        "robot_id": format_csv_value(data.get("robot_id")),
        "protocol_version": format_csv_value(data.get("protocol_version")),
        "time": format_csv_value(data.get("time")),
        "robot_time_ms": format_csv_value(data.get("robot_time_ms", data.get("time"))),
        "state": format_csv_value(data.get("state")),
        "state_id": format_csv_value(data.get("state_id")),
        "flags": format_csv_value(flags),
        "flag_obstacle_detected": int(bool(flags & 1)),
        "flag_ble_connected": int(bool(flags & 2)),
        "flag_clockwise": int(bool(flags & 4)),
        "flag_sensors_ok": int(bool(flags & 8)),
        "flag_slope": int(bool(flags & 16)),
        "receiver_sequence": format_csv_value(data.get("receiver_sequence")),
        "receiver_timestamp_us": format_csv_value(data.get("receiver_timestamp_us")),
        "receiver_queue_drops": format_csv_value(data.get("receiver_queue_drops")),
        "receiver_time_s": format_csv_value(data.get("receiver_time_s")),
        "receiver_epoch": format_csv_value(data.get("receiver_epoch")),
        "receiver_sequence_gaps": format_csv_value(data.get("receiver_sequence_gaps")),
        "serial_decode_errors": format_csv_value(data.get("serial_decode_errors")),
        "dist_front": format_csv_value(data.get("dist_front")),
        "dist_right": format_csv_value(data.get("dist_right")),
        "dist_left": format_csv_value(data.get("dist_left")),
        "dist_side_fl": format_csv_value(data.get("dist_side_fl")),
        "dist_side_rl": format_csv_value(data.get("dist_side_rl")),
        "dist_side_fr": format_csv_value(data.get("dist_side_fr")),
        "dist_side_rr": format_csv_value(data.get("dist_side_rr")),
        "target_steering": format_csv_value(data.get("target_steering")),
        "current_steering": format_csv_value(data.get("current_steering")),
        "target_throttle": format_csv_value(data.get("target_throttle")),
        "current_throttle": format_csv_value(data.get("current_throttle")),
        "battery_voltage_v": format_csv_value(data.get("battery_voltage_v")),
        "pid_error": format_csv_value(data.get("pid_error")),
        "pid_integral": format_csv_value(data.get("pid_integral")),
        "pid_derivative": format_csv_value(data.get("pid_derivative")),
        "accel_x": format_csv_value(data.get("accel_x")),
        "accel_y": format_csv_value(data.get("accel_y")),
        "accel_z": format_csv_value(data.get("accel_z")),
        "gyro_x": format_csv_value(data.get("gyro_x")),
        "gyro_y": format_csv_value(data.get("gyro_y")),
        "gyro_z": format_csv_value(data.get("gyro_z")),
        "roll": format_csv_value(data.get("roll")),
        "pitch": format_csv_value(data.get("pitch")),
        "yaw": format_csv_value(data.get("yaw")),
    }

    sensors = data.get("sensors") or []
    sensor_by_slot = {s.get("slot"): s for s in sensors if isinstance(s, dict)}

    for slot in SLOTS:
        s = sensor_by_slot.get(slot, {})
        row[f"sensor_{slot}_health"] = format_csv_value(s.get("health"))
        row[f"sensor_{slot}_health_name"] = format_csv_value(s.get("health_name"))
        row[f"sensor_{slot}_age_ms"] = format_csv_value(s.get("age_ms"))
        row[f"sensor_{slot}_error_count"] = format_csv_value(s.get("error_count"))
        row[f"sensor_{slot}_consecutive_errors"] = format_csv_value(s.get("consecutive_errors"))
        row[f"sensor_{slot}_last_error"] = format_csv_value(s.get("last_error"))
        row[f"sensor_{slot}_last_error_name"] = format_csv_value(s.get("last_error_name"))
        if slot != "imu":
            row[f"sensor_{slot}_raw_range_mm"] = format_csv_value(s.get("raw_range_mm"))
            row[f"sensor_{slot}_raw_status"] = format_csv_value(s.get("raw_status"))
            row[f"sensor_{slot}_raw_internal_status"] = format_csv_value(s.get("raw_internal_status"))
            row[f"sensor_{slot}_raw_age_ms"] = format_csv_value(s.get("raw_age_ms"))

    return row


class CsvLogger:
    def __init__(self, directory=None):
        if directory is None:
            directory = Path(__file__).resolve().parent / "csv"
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)

        timestamp_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.filepath = self.directory / f"{timestamp_str}.csv"
        counter = 1
        while self.filepath.exists():
            self.filepath = self.directory / f"{timestamp_str}_{counter}.csv"
            counter += 1

        self.file = open(self.filepath, mode="w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.file, fieldnames=CSV_FIELDNAMES)
        self.writer.writeheader()
        self.file.flush()
        self.closed = False
        print(f"Logging telemetry to CSV: {self.filepath}")

    def log(self, data):
        if self.closed or not self.file:
            return
        row = telemetry_to_row(data)
        self.writer.writerow(row)
        self.file.flush()

    def close(self):
        if not self.closed and self.file:
            try:
                self.file.flush()
                self.file.close()
                print(f"Telemetry log saved and closed: {self.filepath}")
            finally:
                self.closed = True


def parse_args():
    parser = argparse.ArgumentParser(
        description="Read Folkrace multi-robot telemetry over serial and forward to PlotJuggler via UDP."
    )
    parser.add_argument(
        "port_pos",
        nargs="?",
        default=None,
        metavar="PORT",
        help="Serial port (e.g. /dev/ttyUSB0 or /dev/ttyACM0)",
    )
    parser.add_argument(
        "baud_pos",
        nargs="?",
        type=int,
        default=None,
        metavar="BAUD",
        help="Baud rate (default: 460800)",
    )
    parser.add_argument(
        "-p",
        "--port",
        dest="port_opt",
        default=None,
        help="Serial port (default: /dev/ttyUSB0)",
    )
    parser.add_argument(
        "-b",
        "--baud",
        dest="baud_opt",
        type=int,
        default=None,
        help="Baud rate (default: 460800)",
    )
    parser.add_argument(
        "--udp-ip",
        default="127.0.0.1",
        help="Target UDP IP address for PlotJuggler (default: 127.0.0.1)",
    )
    parser.add_argument(
        "--udp-port",
        type=int,
        default=9870,
        help="Target UDP base port for PlotJuggler (default: 9870)",
    )
    parser.add_argument(
        "--multi-port",
        action="store_true",
        help="Forward each robot to a dedicated UDP port (e.g. 9870 for sniegbaltite, 9871 for rosepink)",
    )
    parser.add_argument(
        "--robot",
        default=None,
        help="Filter telemetry to a specific robot name (e.g. --robot sniegbaltite)",
    )
    parser.add_argument(
        "--flat",
        action="store_true",
        help="Send flat JSON keys without namespacing under robot name",
    )
    parser.add_argument(
        "--no-udp",
        action="store_true",
        help="Disable forwarding to UDP",
    )
    parser.add_argument(
        "-q",
        "--quiet",
        action="store_true",
        help="Suppress printing telemetry to console",
    )
    parser.add_argument(
        "--csv",
        action="store_true",
        help="Enable saving telemetry to a CSV file (default: disabled)",
    )
    parser.add_argument(
        "--csv-dir",
        default=None,
        help="Directory to save telemetry CSV files (implies --csv, default: csv/ next to script)",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    port = args.port_opt or args.port_pos or "/dev/ttyUSB0"
    baud = args.baud_opt or args.baud_pos or 460800
    filter_robot = args.robot.lower() if args.robot else None

    # Initialize UDP socket if forwarding is enabled
    sock = None
    if not args.no_udp:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if args.multi_port:
            print(f"PlotJuggler UDP stream: multi-port mode starting at {args.udp_ip}:{args.udp_port}")
            for rname, offset in ROBOT_PORT_OFFSETS.items():
                print(f"  -> {rname:<12}: UDP port {args.udp_port + offset}")
        else:
            print(f"PlotJuggler UDP stream: forwarding to {args.udp_ip}:{args.udp_port}")
            print(f"  -> In PlotJuggler: Streaming -> Start 'UDP Server', Protocol: JSON, Port: {args.udp_port}")
            if not args.flat:
                print("  -> Robot curves will be namespaced: /sniegbaltite/..., /rosepink/...")

    csv_logger = None
    if args.csv or args.csv_dir:
        csv_logger = CsvLogger(args.csv_dir)
        atexit.register(csv_logger.close)

    import serial
    start_time = time.monotonic()
    last_console = {}
    ser = None
    try:
        while True:
            try:
                if ser is None:
                    print(f"Opening {port} at {baud} baud...", file=sys.stderr)
                    ser = serial.Serial(port, baud, timeout=0.1)
                    decoder = StreamDecoder()  # Reconnect resets framing and receiver time.
                    last_console.clear()
                chunk = ser.read(min(4096, max(1, ser.in_waiting)))
                for data in decoder.feed(chunk):
                    robot_name = data["robot"]
                    if filter_robot and robot_name != filter_robot:
                        continue
                    host_timestamp = time.monotonic() - start_time
                    data["timestamp"] = host_timestamp
                    data["host_time_s"] = host_timestamp
                    if csv_logger:
                        csv_logger.log(data)
                    if sock:
                        target_port = args.udp_port
                        if args.multi_port:
                            target_port += ROBOT_PORT_OFFSETS.get(robot_name, len(ROBOT_PORT_OFFSETS))
                        payload = data if args.flat else {"timestamp": host_timestamp, robot_name: data}
                        try:
                            sock.sendto(json.dumps(payload, allow_nan=False).encode("utf-8"),
                                        (args.udp_ip, target_port))
                        except OSError:
                            pass
                    if not args.quiet and host_timestamp - last_console.get(robot_name, -1) >= 0.2:
                        last_console[robot_name] = host_timestamp
                        color = ROBOT_COLORS.get(robot_name, "\033[97m")
                        sensors = []
                        for item in data["sensors"]:
                            text = f"{item['slot']}={item['health_name']} age={item['age_ms']}"
                            text += f" err={item['last_error_name']}/{item['error_count']}"
                            if "raw_age_ms" in item:
                                text += (" raw=none" if item["raw_age_ms"] is None else
                                         f" raw={item['raw_range_mm']}/{item['raw_status']:02x}"
                                         f"(code={item['raw_internal_status']},age={item['raw_age_ms']})")
                            sensors.append(text)
                        battery = data["battery_voltage_v"]
                        battery_text = f"{battery:.2f}V" if battery is not None else "?V"
                        throt = data["current_throttle"]
                        throt_str = f"{throt:.2f}" if isinstance(throt, (int, float)) and math.isfinite(throt) else str(throt)
                        if throt_str == "-0.00":
                            throt_str = "0.00"
                        print(f"[{color}{robot_name}{COLOR_RESET}] {data['time']}ms Bat:{battery_text} "
                              f"{data['state']} F={data['dist_front']} "
                              f"throttle={throt_str} "
                              f"drops={data['receiver_queue_drops']} "
                              f"gaps={data['receiver_sequence_gaps']} "
                              f"decode_errors={data['serial_decode_errors']} " + " | ".join(sensors))
            except (serial.SerialException, OSError) as error:
                print(f"Serial connection lost: {error}; retrying...", file=sys.stderr)
                if ser:
                    ser.close()
                ser = None
                time.sleep(1)
    except KeyboardInterrupt:
        print("\nExiting telemetry reader.", file=sys.stderr)
    finally:
        if csv_logger:
            csv_logger.close()
        if ser:
            ser.close()
        if sock:
            sock.close()


if __name__ == "__main__":
    main()
