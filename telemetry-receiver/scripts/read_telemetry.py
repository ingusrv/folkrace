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


def parse_args():
    parser = argparse.ArgumentParser(
        description="Read Folkrace telemetry over serial and forward to PlotJuggler via UDP."
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
        help="Target UDP port for PlotJuggler (default: 9870)",
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

    # Initialize UDP socket if forwarding is enabled
    sock = None
    if not args.no_udp:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print(f"PlotJuggler UDP stream: forwarding to {args.udp_ip}:{args.udp_port}")
        print("  -> In PlotJuggler: Streaming -> Start 'UDP Server', Protocol: JSON, Port: 9870")

    print(f"Opening {port} at {baud} baud...")
    try:
        ser = serial.Serial(port, baud, timeout=1.0)
    except serial.SerialException as e:
        print(f"Failed to open serial port: {e}")
        sys.exit(1)

    time.sleep(1.0)  # Wait for serial reset if applicable
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

                # Add explicit timestamp in seconds for PlotJuggler time axis
                if "timestamp" not in data and "time" in data:
                    data["timestamp"] = data["time"] / 1000.0

                # Expand arrays into explicit named scalar fields for cleaner PlotJuggler curve names
                if "accel" in data and isinstance(data["accel"], list) and len(data["accel"]) == 3:
                    data["accel_x"], data["accel_y"], data["accel_z"] = data["accel"]
                if "gyro" in data and isinstance(data["gyro"], list) and len(data["gyro"]) == 3:
                    data["gyro_x"], data["gyro_y"], data["gyro_z"] = data["gyro"]
                if "euler" in data and isinstance(data["euler"], list) and len(data["euler"]) == 3:
                    data["roll"], data["pitch"], data["yaw"] = data["euler"]

                # Forward to PlotJuggler via UDP
                if sock:
                    try:
                        udp_payload = json.dumps(data).encode("utf-8")
                        sock.sendto(udp_payload, (args.udp_ip, args.udp_port))
                    except OSError:
                        pass

                if not args.quiet:
                    print(
                        f"[{data.get('time', 0):>7} ms] "
                        f"State: {data.get('state', 'UNKNOWN'):<12} "
                        f"Dist: (F:{data.get('dist_front', 0):>4}, R:{data.get('dist_right', 0):>4}, L:{data.get('dist_left', 0):>4}) "
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
