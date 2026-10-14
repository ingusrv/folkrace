import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import time
import unittest
import zlib

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from telemetry_decoder import MAX_ENCODED_SIZE, StreamDecoder, cobs_decode


def cobs_encode(raw):
    result = bytearray([0])
    code_at, code = 0, 1
    for value in raw:
        if value:
            result.append(value)
            code += 1
            if code != 255:
                continue
        result[code_at] = code
        code_at, code = len(result), 1
        result.append(0)
    result[code_at] = code
    return bytes(result) + b"\0"


class DecoderTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with tempfile.TemporaryDirectory() as directory:
            exe = str(Path(directory) / "fixture")
            subprocess.run(["c++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
                            "-I" + str(ROOT.parent / "shared/src"),
                            str(ROOT / "tests/fixture.cpp"), "-o", exe], check=True)
            cls.fixture = subprocess.check_output([exe])
        cls.raw = cobs_decode(cls.fixture[:-1])

    def record(self, seq=None, arrival=None, change=None):
        raw = bytearray(self.raw)
        if seq is not None:
            struct.pack_into("<I", raw, 1, seq)
        if arrival is not None:
            struct.pack_into("<I", raw, 5, arrival)
        if change:
            raw[change[0]] = change[1]
        struct.pack_into("<I", raw, len(raw) - 4, zlib.crc32(raw[:-4]))
        return cobs_encode(raw)

    def test_cpp_fixture_fields(self):
        data, = StreamDecoder().feed(self.fixture)
        self.assertEqual(data["robot"], "rosepink")
        self.assertEqual(data["time"], 0x12345678)
        self.assertEqual(data["state"], "REVERSE_TURN")
        self.assertEqual(data["dist_left"], 789)
        self.assertEqual(data["target_steering"], -0.75)
        self.assertEqual(data["accel_x"], 9.5)
        self.assertEqual(data["gyro_z"], -3.25)
        self.assertEqual(data["yaw"], 179.5)
        self.assertEqual(data["protocol_version"], 2)
        self.assertEqual(data["battery_voltage_v"], 7.625)
        self.assertEqual(data["receiver_queue_drops"], 7)
        self.assertIsNone(data["sensors"][0]["age_ms"])
        self.assertEqual(data["sensors"][1]["raw_range_mm"], 8190)
        self.assertEqual(data["sensors"][2]["raw_range_mm"], 8191)
        self.assertEqual(data["sensors"][2]["raw_status"], 0x50)
        self.assertEqual(data["sensors"][2]["raw_internal_status"], 10)
        self.assertIsNone(data["sensors"][3]["raw_range_mm"])
        self.assertEqual(data["sensors"][7]["health_name"], "RANGE_INVALID")
        self.assertEqual(data["sensors"][7]["last_error_name"], "MEASUREMENT_TIMEOUT")
        json.dumps(data, allow_nan=False)

    def test_no_return_quality_is_preserved(self):
        # Serial envelope 15 + header 8 + telemetry diagnostics offset 85.
        data, = StreamDecoder().feed(self.record(change=(108, 8)))
        self.assertEqual(data["sensors"][0]["health"], 8)
        self.assertEqual(data["sensors"][0]["health_name"], "NO_RETURN")

    def test_all_split_positions_and_coalescing(self):
        for cut in range(len(self.fixture) + 1):
            decoder = StreamDecoder()
            packets = decoder.feed(self.fixture[:cut])
            packets += decoder.feed(self.fixture[cut:] + self.fixture)
            self.assertEqual(len(packets), 2)
        decoder = StreamDecoder()
        self.assertEqual(sum(len(decoder.feed(bytes([b]))) for b in self.fixture), 1)

    def test_corruption_versions_length_boot_text_resync(self):
        bad_crc = bytearray(self.raw)
        bad_crc[42] ^= 1
        bad_frames = [b"boot output\r\n\0", b"\xffshort\0", b"a" * 10000 + b"\0",
                      self.fixture[20:], self.fixture[:100] + b"\0",
                      cobs_encode(bad_crc)]
        for offset, value in [(0, 2), (13, 191), (15, 0), (19, 1), (21, 183), (23, 1)]:
            bad_frames.append(self.record(change=(offset, value)))
        for bad in bad_frames:
            decoder = StreamDecoder()
            packets = decoder.feed(bad + self.fixture)
            self.assertEqual(len(packets), 1)
            self.assertGreater(decoder.decode_errors, 0)
            self.assertLessEqual(len(decoder.buffer), MAX_ENCODED_SIZE)
        decoder = StreamDecoder()
        decoder.feed(b"x" * 100000)
        self.assertEqual(len(decoder.buffer), 0)
        self.assertEqual(len(decoder.feed(b"\0" + self.fixture)), 1)

    def test_time_sequence_wrap_drops_and_reboot(self):
        decoder = StreamDecoder()
        records = decoder.feed(self.fixture + self.record(1, 48))
        self.assertEqual(records[-1]["receiver_sequence_gaps"], 2)
        self.assertAlmostEqual(records[-1]["receiver_time_s"], 64 / 1e6)
        self.assertEqual(records[-1]["receiver_epoch"], 0)
        reboot, = decoder.feed(self.record(0, 10))
        self.assertEqual(reboot["receiver_epoch"], 1)
        self.assertEqual(reboot["receiver_time_s"], 0)
        reconnected, = StreamDecoder().feed(self.record(0, 10))
        self.assertEqual(reconnected["receiver_sequence_gaps"], 0)

    def test_four_robot_bandwidth_and_decoder_throughput(self):
        self.assertLessEqual(len(self.fixture) * 4 * 30, 26040)
        stream = b"".join(self.record(i, i * 8333) for i in range(1200))
        start = time.monotonic()
        packets = StreamDecoder().feed(stream)
        self.assertEqual(len(packets), 1200)
        self.assertLess(time.monotonic() - start, 10)  # Ten seconds of serial data.
        self.assertEqual(packets[-1]["receiver_sequence_gaps"], 0)


if __name__ == "__main__":
    unittest.main()
