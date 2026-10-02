#!/usr/bin/env python3
"""
Folkrace Telemetry Serial Reader & PlotJuggler UDP Forwarder
Reads JSON lines emitted by the ESP32 telemetry-receiver over Serial
and forwards them as JSON datagrams to PlotJuggler via UDP.
"""

import argparse
import json
import socket
import sys
import time
import serial


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

    print(f"Opening {port} at {baud} baud...")
    try:
        ser = serial.Serial(port, baud, timeout=1.0)
    except serial.SerialException as e:
        print(f"Failed to open serial port: {e}")
        sys.exit(1)

    time.sleep(1.0)  # Wait for serial reset if applicable
    start_time = time.time()
    print("Listening for telemetry packets (Ctrl+C to exit)...")

    try:
        while True:
            raw_line = ser.readline()
            if not raw_line:
                continue

            try:
                line_str = raw_line.decode("utf-8", errors="replace").strip()
                if not line_str.startswith("{"):
                    if line_str and not args.quiet:
                        print(f"[ESP32] {line_str}")
                    continue

                data = json.loads(line_str)
                robot_name = data.get("robot", "unknown").lower()

                # Filter by robot if requested
                if filter_robot and robot_name != filter_robot:
                    continue

                # Synchronized race timestamp across all robots (seconds from script start)
                host_timestamp = time.time() - start_time
                data["robot_time_ms"] = data.get("time", 0)
                data["timestamp"] = host_timestamp

                # Expand arrays into explicit named scalar fields for cleaner PlotJuggler curve names
                if "accel" in data and isinstance(data["accel"], list) and len(data["accel"]) == 3:
                    data["accel_x"], data["accel_y"], data["accel_z"] = data["accel"]
                if "gyro" in data and isinstance(data["gyro"], list) and len(data["gyro"]) == 3:
                    data["gyro_x"], data["gyro_y"], data["gyro_z"] = data["gyro"]
                if "euler" in data and isinstance(data["euler"], list) and len(data["euler"]) == 3:
                    data["roll"], data["pitch"], data["yaw"] = data["euler"]

                # Forward to PlotJuggler via UDP
                if sock:
                    target_port = args.udp_port
                    if args.multi_port:
                        port_offset = ROBOT_PORT_OFFSETS.get(robot_name, len(ROBOT_PORT_OFFSETS))
                        target_port = args.udp_port + port_offset

                    # Structure payload: namespaced unless --flat is requested
                    if args.flat:
                        payload_dict = data
                    else:
                        payload_dict = {
                            "timestamp": host_timestamp,
                            robot_name: data,
                        }

                    try:
                        udp_payload = json.dumps(payload_dict).encode("utf-8")
                        sock.sendto(udp_payload, (args.udp_ip, target_port))
                    except OSError:
                        pass

                if not args.quiet:
                    color = ROBOT_COLORS.get(robot_name, "\033[97m")
                    flags = data.get("flags", 0)
                    dir_str = "CW " if (flags & 4) else "CCW"
                    print(
                        f"[{color}{robot_name:<12}{COLOR_RESET}] "
                        f"[{data.get('time', 0):>7} ms] "
                        f"Dir: {dir_str} "
                        f"State: {data.get('state', 'UNKNOWN'):<12} "
                        f"Dist: (F:{data.get('dist_front', 0):>4}, R:{data.get('dist_right', 0):>4}, L:{data.get('dist_left', 0):>4}) "
                        f"Side: (FL:{data.get('dist_side_fl', 0):>4}, RL:{data.get('dist_side_rl', 0):>4}, FR:{data.get('dist_side_fr', 0):>4}, RR:{data.get('dist_side_rr', 0):>4}) "
                        f"Steer: {data.get('current_steering', 0):>6.2f} "
                        f"Throt: {data.get('current_throttle', 0):>6.2f} "
                        f"PID err: {data.get('pid_error', 0):>7.4f}"
                    )
            except json.JSONDecodeError:
                pass
    except KeyboardInterrupt:
        print("\nExiting telemetry reader.")
    finally:
        ser.close()
        if sock:
            sock.close()


if __name__ == "__main__":
    main()
