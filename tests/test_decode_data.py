#!/usr/bin/env python3

import csv
import importlib.util
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "tools" / "decode_data.py"
spec = importlib.util.spec_from_file_location("decode_data", MODULE_PATH)
decode_data = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(decode_data)


class DecodeDataTests(unittest.TestCase):
    def test_crc_reference_vector(self):
        self.assertEqual(decode_data.crc("123456789"), 0x29B1)

    def test_accepts_leading_zero_checksum(self):
        payload = "TEST,16,000000,+00.000000,+000.000000,1,01,3000,0000,+00"
        self.assertEqual(f"{decode_data.crc(payload):04X}", "0753")
        capture = b"noise$$$$" + payload.encode() + b"*0753\n"
        with tempfile.TemporaryDirectory() as directory:
            capture_path = Path(directory) / "capture.txt"
            capture_path.write_bytes(capture)
            frames = decode_data.analyse_data(capture_path)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0][0], "TEST")

    def test_sample_capture_decodes(self):
        frames = decode_data.analyse_data(ROOT / "tools" / "with_pips_data.txt")
        self.assertEqual(len(frames), 165)

    def test_cycle_delta_handles_midnight(self):
        frames = [
            ["X", "1", "235959", "0", "0", "0", "0", "0", "0", "0"],
            ["X", "2", "000001", "0", "0", "0", "0", "0", "0", "0"],
        ]
        self.assertEqual(decode_data.cycle_deltas(frames), [2])

    def test_csv_output_contains_only_crc_valid_frames(self):
        valid = "PICO,42,123456,+51.536248,-000.207353,1234,08,3175,0000,+18"
        invalid = "PICO,43,123457,+51.536249,-000.207354,1235,07,3174,0001,+17"

        with tempfile.TemporaryDirectory() as directory:
            directory_path = Path(directory)
            capture_path = directory_path / "capture.txt"
            csv_path = directory_path / "frames.csv"
            capture_path.write_bytes(
                f"$${valid}*{decode_data.crc(valid):04X}\n$${invalid}*0000\n".encode()
            )

            self.assertEqual(
                decode_data.main([str(capture_path), "--csv-output", str(csv_path)]),
                0,
            )

            with csv_path.open(newline="", encoding="utf-8") as csv_file:
                rows = list(csv.reader(csv_file))

        self.assertEqual(
            rows,
            [
                [
                    "payload_name",
                    "sentence_id",
                    "utc_time",
                    "latitude_deg",
                    "longitude_deg",
                    "altitude_m",
                    "satellites",
                    "voltage_mv",
                    "op_status",
                    "temperature_c",
                ],
                valid.split(","),
            ],
        )

    def test_decode_op_status_nominal(self):
        status = decode_data.decode_op_status(0)
        self.assertEqual(status.raw, 0)
        self.assertEqual(status.fix_attempts, 0)
        self.assertEqual(status.config_status, 0)
        self.assertEqual(status.config_status_text, "ok")
        self.assertEqual(status.poll_status, 0)
        self.assertEqual(status.poll_status_text, "ok")
        self.assertFalse(status.degraded)
        self.assertFalse(status.is_degraded)
        self.assertFalse(status.measurement_error)
        self.assertFalse(status.radio_measurement_failed)
        self.assertEqual(status.format(), "attempts=0, config=ok, poll=ok, radio=ok")
        self.assertEqual(str(status), status.format())

    def test_decode_op_status_fields_and_formats(self):
        s1 = decode_data.decode_op_status(0x0016)
        self.assertEqual(s1.fix_attempts, 1)
        self.assertEqual(s1.config_status_text, "transient-error")
        self.assertEqual(s1.poll_status_text, "retry-exhausted")
        self.assertFalse(s1.degraded)
        self.assertFalse(s1.measurement_error)

        s2 = decode_data.decode_op_status("0x00FB")
        self.assertEqual(s2.fix_attempts, 15)
        self.assertEqual(s2.config_status_text, "retry-exhausted")
        self.assertEqual(s2.poll_status_text, "degraded")
        self.assertTrue(s2.degraded)
        self.assertTrue(s2.is_degraded)
        self.assertFalse(s2.measurement_error)

        s3 = decode_data.decode_op_status("0004")
        self.assertEqual(s3.fix_attempts, 0)
        self.assertEqual(s3.config_status_text, "transient-error")
        self.assertEqual(s3.poll_status_text, "ok")
        self.assertFalse(s3.degraded)

        s4 = decode_data.decode_op_status(0x0120)
        self.assertEqual(s4.fix_attempts, 2)
        self.assertTrue(s4.measurement_error)
        self.assertTrue(s4.radio_measurement_failed)
        self.assertIn("radio=failed", s4.format())

    def test_decode_op_status_negative_rejected(self):
        with self.assertRaises(ValueError):
            decode_data.decode_op_status(-1)

    def test_summarize_op_status(self):
        frames = [
            ["PICO", "1", "120000", "+00.0", "+00.0", "100", "05", "3000", "0000", "+20"],
            ["PICO", "2", "120030", "+00.0", "+00.0", "100", "05", "3000", "0004", "+20"],
            ["PICO", "3", "120100", "+00.0", "+00.0", "100", "05", "3000", "0251", "+20"],
            ["PICO", "4", "120130", "+00.0", "+00.0", "100", "05", "3000", "0288", "+20"],
        ]
        summary = decode_data.summarize_op_status(frames)
        self.assertEqual(summary["total"], 4)
        self.assertEqual(summary["min_attempts"], 0)
        self.assertEqual(summary["max_attempts"], 15)
        self.assertEqual(summary["degraded"], 1)
        self.assertEqual(summary["config_errors"], 2)
        self.assertEqual(summary["poll_errors"], 1)
        self.assertEqual(summary["radio_errors"], 1)

    def test_cli_decode_status_flag(self):
        import io
        from contextlib import redirect_stdout

        payload = "PICO,42,123456,+51.536248,-000.207353,1234,08,3175,0004,+18"
        capture = f"$${payload}*{decode_data.crc(payload):04X}\n".encode()
        with tempfile.TemporaryDirectory() as directory:
            capture_path = Path(directory) / "capture.txt"
            capture_path.write_bytes(capture)

            out = io.StringIO()
            with redirect_stdout(out):
                exit_code = decode_data.main(
                    [str(capture_path), "--print-frames", "--decode-status"]
                )
            self.assertEqual(exit_code, 0)
            output = out.getvalue()
            self.assertIn(
                "op_status: attempts=0, config=transient-error, poll=ok, radio=ok", output
            )
            self.assertIn("op_status summary:", output)


if __name__ == "__main__":
    unittest.main()
