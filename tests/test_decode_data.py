#!/usr/bin/env python3

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
        with tempfile.NamedTemporaryFile() as handle:
            handle.write(capture)
            handle.flush()
            frames = decode_data.analyse_data(Path(handle.name))
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


if __name__ == "__main__":
    unittest.main()
