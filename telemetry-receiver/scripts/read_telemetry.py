#!/usr/bin/env python3
"""
Folkrace Telemetry Serial Reader
Reads JSON lines emitted by the ESP32 telemetry-receiver over Serial.
"""

import sys
import json
import serial
import time

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 460800

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
                line_str = raw_line.decode('utf-8', errors='replace').strip()
                if not line_str.startswith('{'):
                    # Skip boot logs or debug messages
                    continue

                data = json.loads(line_str)
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

if __name__ == "__main__":
    main()
