import re
import sys
import tempfile
import unittest
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import simulate_tracker as sim  # noqa: E402
from decode_data import analyse_data, crc  # noqa: E402


class TrackerSimulatorTests(unittest.TestCase):
    def test_constants_track_firmware_configuration(self):
        main_h = (ROOT / "firmware" / "main.h").read_text()
        status_h = (ROOT / "firmware" / "status_word.h").read_text()
        sleep_h = (ROOT / "firmware" / "sleep_policy.h").read_text()
        energy_c = (ROOT / "firmware" / "energy.c").read_text()

        self.assertEqual(
            re.search(r'#define PAYLOAD_NAME "([^"]+)"', main_h).group(1), sim.PAYLOAD_NAME
        )
        self.assertIn("#define GPS_FIX_ATTEMPTS_MAX       0x0FU", status_h)
        self.assertIn("#define HIGH_ALTITUDE_SLEEP_THRESHOLD_M 3000U", sleep_h)
        self.assertIn("Total delay is 30.720s", energy_c)
        self.assertEqual(sim.GPS_FIX_ATTEMPTS_MAX, 15)
        self.assertEqual(sim.HIGH_ALTITUDE_SLEEP_THRESHOLD_M, 3000)
        self.assertEqual(sim.AWU_INTERVAL_SECONDS, 30.720)

    def test_frame_matches_known_capture_vector(self):
        fix = sim.TrackerFix(
            utc=datetime(2026, 1, 1, 22, 41, 22, tzinfo=timezone.utc),
            altitude_m=31,
            latitude_e7=515362480,
            longitude_e7=-2073530,
            satellites=4,
            voltage_mv=3175,
            temperature_c=18,
            op_status=4,
        )
        frame, _ = sim.build_frame(fix, 35)
        expected = "$$ICSPACE14,35,224122,+51.536248,-000.207353,31,04,3175,0004,+18*B3A7"
        self.assertEqual(frame, expected)
        data, checksum = frame[2:].split("*")
        self.assertEqual(f"{crc(data):04X}", checksum)

    def test_sleep_threshold_matches_firmware(self):
        self.assertEqual(sim.sleep_intervals_for_altitude(3000), 1)
        self.assertEqual(sim.sleep_intervals_for_altitude(3001), 2)

    def test_gps_loss_retains_last_fix_and_recovers(self):
        results = sim.run_simulation(sim.synthetic_profile()[:11], gps_loss_cycles={8, 9, 10})
        self.assertEqual(results[7].fix.altitude_m, results[6].fix.altitude_m)
        self.assertEqual(results[7].fix.utc, results[6].fix.utc)
        self.assertEqual(results[7].gps_attempts, 15)
        self.assertEqual(results[7].fix.op_status, 0x00F3)
        self.assertEqual(results[10].fix.altitude_m, results[10].truth.altitude_m)
        self.assertTrue(results[10].gps_acquired)

    def test_measurement_and_tx_faults_are_visible(self):
        results = sim.run_simulation(
            sim.synthetic_profile()[:5],
            measurement_fail_cycles={5},
            tx_fail_cycles={4},
        )
        self.assertFalse(results[3].tx_success)
        self.assertEqual(results[4].fix.voltage_mv, 0)
        self.assertEqual(results[4].fix.temperature_c, 0)
        self.assertTrue(results[4].fix.op_status & sim.OP_STATUS_MEASUREMENT_ERROR)

    def test_simulated_capture_is_decoder_compatible(self):
        results = sim.run_simulation(
            sim.synthetic_profile()[:12],
            gps_loss_cycles={8, 9, 10},
            tx_fail_cycles={12},
        )
        payload = "".join(result.raw_frame for result in results if result.tx_success)
        with tempfile.TemporaryDirectory() as directory:
            capture = Path(directory) / "simulated.txt"
            capture.write_text(payload, encoding="ascii")
            frames = analyse_data(capture)
        self.assertEqual(len(frames), sum(result.tx_success for result in results))

    def test_slow_success_reports_retry_state(self):
        result = sim.run_simulation(sim.synthetic_profile()[:1], gps_success_attempt=4)[0]
        self.assertEqual(result.gps_attempts, 4)
        self.assertEqual(result.fix.op_status, 0x0042)


if __name__ == "__main__":
    unittest.main()
