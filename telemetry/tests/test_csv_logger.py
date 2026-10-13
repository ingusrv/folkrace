import csv
from datetime import datetime
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from read_telemetry import (
    CSV_BASE_FIELDS,
    CSV_FIELDNAMES,
    CsvLogger,
    format_csv_value,
    parse_args,
    telemetry_to_row,
)
from telemetry_decoder import StreamDecoder


class CsvLoggerTests(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.csv_dir = Path(self.temp_dir.name) / "test_csv_dir"

    def tearDown(self):
        self.temp_dir.cleanup()

    def test_creates_dir_and_timestamped_file(self):
        self.assertFalse(self.csv_dir.exists())
        logger = CsvLogger(directory=self.csv_dir)
        try:
            self.assertTrue(self.csv_dir.exists())
            self.assertTrue(logger.filepath.exists())
            # Filename should match YYYY-MM-DD_HH-MM-SS.csv format
            name = logger.filepath.name
            self.assertTrue(name.endswith(".csv"))
            # Parse datetime from filename (excluding .csv)
            dt_part = name[:-4].split("_")
            self.assertEqual(len(dt_part), 2)
            datetime.strptime(f"{dt_part[0]}_{dt_part[1]}", "%Y-%m-%d_%H-%M-%S")
        finally:
            logger.close()
            self.assertTrue(logger.closed)
            self.assertTrue(logger.file.closed)

    def test_file_closed_on_close(self):
        logger = CsvLogger(directory=self.csv_dir)
        path = logger.filepath
        logger.close()
        self.assertTrue(logger.file.closed)
        self.assertTrue(logger.closed)
        # Verify header was written
        with open(path, mode="r", newline="", encoding="utf-8") as f:
            reader = csv.reader(f)
            header = next(reader)
            self.assertEqual(header, CSV_FIELDNAMES)

    def test_logs_telemetry_row(self):
        logger = CsvLogger(directory=self.csv_dir)
        path = logger.filepath
        sample_data = {
            "robot": "sniegbaltite",
            "robot_id": 1,
            "protocol_version": 1,
            "time": 12345,
            "robot_time_ms": 12345,
            "timestamp": 12.345,
            "host_time_s": 12.345,
            "state": "RUNNING",
            "state_id": 2,
            "flags": 0b00000101,  # bit 0 (obstacle) and bit 2 (clockwise)
            "receiver_sequence": 42,
            "receiver_timestamp_us": 999999,
            "receiver_queue_drops": 0,
            "receiver_time_s": 1.23,
            "receiver_epoch": 0,
            "receiver_sequence_gaps": 0,
            "serial_decode_errors": 0,
            "dist_front": 500,
            "dist_right": 200,
            "dist_left": 300,
            "dist_side_fl": 150,
            "dist_side_rl": 160,
            "dist_side_fr": 170,
            "dist_side_rr": 180,
            "target_steering": 0.25,
            "current_steering": 0.20,
            "target_throttle": 0.8,
            "current_throttle": 0.75,
            "pid_error": -0.05,
            "pid_integral": 0.01,
            "pid_derivative": 0.02,
            "accel_x": 1.0,
            "accel_y": 0.0,
            "accel_z": 9.8,
            "gyro_x": 0.1,
            "gyro_y": -0.1,
            "gyro_z": 0.05,
            "roll": 1.5,
            "pitch": -2.0,
            "yaw": 45.0,
            "sensors": [
                {
                    "slot": "front",
                    "health": 2,
                    "health_name": "OK",
                    "age_ms": 10,
                    "error_count": 0,
                    "consecutive_errors": 0,
                    "last_error": 0,
                    "last_error_name": "NONE",
                    "raw_range_mm": 512,
                    "raw_status": 0,
                    "raw_internal_status": 0,
                    "raw_age_ms": 5,
                }
            ],
        }
        logger.log(sample_data)
        logger.close()

        with open(path, mode="r", newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            rows = list(reader)
            self.assertEqual(len(rows), 1)
            row = rows[0]
            self.assertEqual(row["robot"], "sniegbaltite")
            self.assertEqual(row["state"], "RUNNING")
            self.assertEqual(row["flag_obstacle_detected"], "1")
            self.assertEqual(row["flag_clockwise"], "1")
            self.assertEqual(row["flag_ble_connected"], "0")
            self.assertEqual(row["dist_front"], "500")
            self.assertEqual(row["target_steering"], "0.25")
            self.assertEqual(row["sensor_front_health_name"], "OK")
            self.assertEqual(row["sensor_front_raw_range_mm"], "512")
            self.assertEqual(row["sensor_imu_health_name"], "")

    def test_logs_fixture_record(self):
        from test_decoder import DecoderTests
        DecoderTests.setUpClass()
        data, = StreamDecoder().feed(DecoderTests.fixture)
        data["timestamp"] = 0.123
        data["host_time_s"] = 0.123
        logger = CsvLogger(directory=self.csv_dir)
        logger.log(data)
        logger.close()

        with open(logger.filepath, mode="r", newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            rows = list(reader)
            self.assertEqual(len(rows), 1)
            row = rows[0]
            self.assertEqual(row["robot"], "rosepink")
            self.assertEqual(row["state"], "REVERSE_TURN")
            self.assertEqual(row["dist_left"], "789")
            self.assertEqual(row["target_steering"], "-0.75")
            self.assertEqual(row["accel_x"], "9.50")
            self.assertEqual(row["gyro_z"], "-3.25")
            self.assertEqual(row["yaw"], "179.50")
            self.assertEqual(row["sensor_right_raw_range_mm"], "8190")
            self.assertEqual(row["sensor_left_raw_range_mm"], "8191")
            self.assertEqual(row["sensor_left_raw_status"], str(0x50))
            self.assertEqual(row["sensor_left_raw_internal_status"], "10")
            self.assertEqual(row["sensor_side_fl_raw_range_mm"], "")
            self.assertEqual(row["sensor_imu_health_name"], "RANGE_INVALID")
            self.assertEqual(row["sensor_imu_last_error_name"], "MEASUREMENT_TIMEOUT")

    def test_float_formatting_two_decimals(self):
        self.assertEqual(format_csv_value(0.12345), "0.12")
        self.assertEqual(format_csv_value(0.1), "0.10")
        self.assertEqual(format_csv_value(0.0), "0.00")
        self.assertEqual(format_csv_value(-0.0001), "0.00")
        self.assertEqual(format_csv_value(-1.5), "-1.50")
        self.assertEqual(format_csv_value(None), "")
        self.assertEqual(format_csv_value(float("nan")), "")
        self.assertEqual(format_csv_value(42), 42)
        self.assertEqual(format_csv_value("test"), "test")

    def test_filename_collision_handling(self):
        logger1 = CsvLogger(directory=self.csv_dir)
        logger2 = CsvLogger(directory=self.csv_dir)
        try:
            self.assertNotEqual(logger1.filepath, logger2.filepath)
            self.assertTrue(logger1.filepath.exists())
            self.assertTrue(logger2.filepath.exists())
        finally:
            logger1.close()
            logger2.close()

    def test_cli_args_parsing(self):
        sys.argv = ["read_telemetry.py"]
        args = parse_args()
        self.assertFalse(args.csv)
        self.assertIsNone(args.csv_dir)

        sys.argv = ["read_telemetry.py", "--csv", "--csv-dir", "/tmp/custom_csv"]
        args = parse_args()
        self.assertTrue(args.csv)
        self.assertEqual(args.csv_dir, "/tmp/custom_csv")


if __name__ == "__main__":
    unittest.main()
