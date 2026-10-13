#!/usr/bin/env python3
"""
Folkrace Telemetry Serial Reader & PlotJuggler UDP Forwarder
Decodes COBS/CRC binary records from the ESP32 receiver and forwards
host-generated JSON datagrams to PlotJuggler via UDP.
"""

import argparse
import json
import socket
import sys
import time
from telemetry_decoder import StreamDecoder


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
                        print(f"[{color}{robot_name}{COLOR_RESET}] {data['time']}ms "
                              f"{data['state']} F={data['dist_front']} "
                              f"throttle={data['current_throttle']} "
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
        if ser:
            ser.close()
        if sock:
            sock.close()


if __name__ == "__main__":
    main()
