#!/usr/bin/env python3
"""Hardware-independent PicoTracker flight-cycle simulator."""

from __future__ import annotations

import argparse
import binascii
from dataclasses import dataclass, replace
from datetime import datetime, timedelta, timezone
from pathlib import Path

PAYLOAD_NAME = "ICSPACE14"
GPS_FIX_ATTEMPTS_MAX = 15
HIGH_ALTITUDE_SLEEP_THRESHOLD_M = 3000
AWU_INTERVAL_SECONDS = 30.720
OP_STATUS_OK = 0
OP_STATUS_TRANSIENT_ERROR = 1
OP_STATUS_RETRY_EXHAUSTED = 2
OP_STATUS_DEGRADED = 3
OP_STATUS_MEASUREMENT_ERROR = 0x0100


@dataclass(frozen=True)
class FlightPoint:
    altitude_m: int
    latitude_e7: int
    longitude_e7: int
    satellites: int
    voltage_mv: int
    temperature_c: int

@dataclass(frozen=True)
class TrackerFix:
    utc: datetime
    altitude_m: int
    latitude_e7: int
    longitude_e7: int
    satellites: int
    voltage_mv: int = 0
    temperature_c: int = 0
    op_status: int = 0


@dataclass(frozen=True)
class CycleResult:
    cycle: int
    truth: FlightPoint
    fix: TrackerFix
    gps_acquired: bool
    gps_attempts: int
    sleep_intervals: int
    tx_success: bool
    measurement_ok: bool
    frame: str
    raw_frame: str

    @property
    def sleep_seconds(self) -> float:
        return self.sleep_intervals * AWU_INTERVAL_SECONDS


def gps_status_pack(fix_attempts: int, config_status: int, poll_status: int) -> int:
    return ((fix_attempts & 0x0F) << 4) | ((config_status & 0x03) << 2) | (poll_status & 0x03)

def poll_status_for_success(attempts: int) -> int:
    if attempts <= 1:
        return OP_STATUS_OK
    return OP_STATUS_RETRY_EXHAUSTED if ((attempts - 1) % 3 == 0) else OP_STATUS_TRANSIENT_ERROR


def sleep_intervals_for_altitude(altitude_m: int) -> int:
    return 2 if altitude_m > HIGH_ALTITUDE_SLEEP_THRESHOLD_M else 1


def format_latitude(value_e7: int) -> str:
    value_e6 = abs(value_e7) // 10
    sign = "-" if value_e7 < 0 else "+"
    digits = f"{value_e6:08d}"
    return f"{sign}{digits[:2]}.{digits[2:]}"


def format_longitude(value_e7: int) -> str:
    value_e6 = abs(value_e7) // 10
    sign = "-" if value_e7 < 0 else "+"
    digits = f"{value_e6:09d}"
    return f"{sign}{digits[:3]}.{digits[3:]}"


def format_temperature(value: int) -> str:
    magnitude = min(abs(value), 99)
    return f"{'-' if value < 0 else '+'}{magnitude:02d}"


def build_frame(fix: TrackerFix, sentence_id: int) -> tuple[str, str]:
    data = ",".join(
        [
            PAYLOAD_NAME,
            str(sentence_id),
            fix.utc.strftime("%H%M%S"),
            format_latitude(fix.latitude_e7),
            format_longitude(fix.longitude_e7),
            str(fix.altitude_m),
            f"{fix.satellites:02d}",
            f"{fix.voltage_mv:04d}",
            f"{fix.op_status:04d}",
            format_temperature(fix.temperature_c),
        ]
    )
    checksum = binascii.crc_hqx(data.encode("ascii"), 0xFFFF)
    frame = f"$${data}*{checksum:04X}"
    raw_frame = f"  $$$${data}*{checksum:04X}\n\n"
    return frame, raw_frame


def synthetic_profile() -> list[FlightPoint]:
    altitudes = [
        0, 500, 1500, 3000, 5000, 8000, 12000, 16000, 20000,
        24000, 28000, 30000, 28000, 22000, 16000, 10000, 4000, 1000,
    ]
    points: list[FlightPoint] = []
    for index, altitude in enumerate(altitudes):
        points.append(
            FlightPoint(
                altitude_m=altitude,
                latitude_e7=515000000 + index * 5000,
                longitude_e7=-1000000 - index * 3000,
                satellites=8 + (index % 3),
                voltage_mv=3250 - index * 4,
                temperature_c=18 - altitude // 1000,
            )
        )
    return points

def run_simulation(
    points: list[FlightPoint],
    *,
    gps_loss_cycles: set[int] | None = None,
    measurement_fail_cycles: set[int] | None = None,
    tx_fail_cycles: set[int] | None = None,
    gps_success_attempt: int = 1,
    config_status: int = OP_STATUS_OK,
) -> list[CycleResult]:
    gps_loss_cycles = gps_loss_cycles or set()
    measurement_fail_cycles = measurement_fail_cycles or set()
    tx_fail_cycles = tx_fail_cycles or set()
    current_fix: TrackerFix | None = None
    utc = datetime(2026, 1, 1, 12, 0, 0, tzinfo=timezone.utc)
    results: list[CycleResult] = []

    for cycle, truth in enumerate(points, start=1):
        gps_acquired = cycle not in gps_loss_cycles
        if gps_acquired:
            attempts = gps_success_attempt
            poll_status = poll_status_for_success(attempts)
            current_fix = TrackerFix(
                utc=utc,
                altitude_m=truth.altitude_m,
                latitude_e7=truth.latitude_e7,
                longitude_e7=truth.longitude_e7,
                satellites=truth.satellites,
            )
        else:
            attempts = GPS_FIX_ATTEMPTS_MAX
            poll_status = OP_STATUS_DEGRADED
            if current_fix is None:
                current_fix = TrackerFix(utc=utc.replace(hour=0, minute=0, second=0), altitude_m=0,
                                         latitude_e7=0, longitude_e7=0, satellites=0)

        measurement_ok = cycle not in measurement_fail_cycles
        measurement_status = 0 if measurement_ok else OP_STATUS_MEASUREMENT_ERROR
        voltage_mv = truth.voltage_mv if measurement_ok else 0
        temperature_c = truth.temperature_c if measurement_ok else 0
        op_status = gps_status_pack(attempts, config_status, poll_status) | measurement_status
        current_fix = replace(
            current_fix,
            voltage_mv=voltage_mv,
            temperature_c=temperature_c,
            op_status=op_status,
        )

        frame, raw_frame = build_frame(current_fix, cycle)
        sleep_intervals = sleep_intervals_for_altitude(current_fix.altitude_m)
        tx_success = cycle not in tx_fail_cycles
        results.append(
            CycleResult(
                cycle=cycle,
                truth=truth,
                fix=current_fix,
                gps_acquired=gps_acquired,
                gps_attempts=attempts,
                sleep_intervals=sleep_intervals,
                tx_success=tx_success,
                measurement_ok=measurement_ok,
                frame=frame,
                raw_frame=raw_frame,
            )
        )
        utc += timedelta(seconds=sleep_intervals * AWU_INTERVAL_SECONDS)

    return results


def parse_cycle_set(value: str) -> set[int]:
    cycles: set[int] = set()
    if not value:
        return cycles
    for token in value.split(","):
        token = token.strip()
        if not token:
            continue
        if "-" in token:
            start_text, end_text = token.split("-", 1)
            start, end = int(start_text), int(end_text)
            if start < 1 or end < start:
                raise argparse.ArgumentTypeError(f"invalid cycle range: {token}")
            cycles.update(range(start, end + 1))
        else:
            cycle = int(token)
            if cycle < 1:
                raise argparse.ArgumentTypeError(f"invalid cycle: {token}")
            cycles.add(cycle)
    return cycles


def scenario_faults(name: str) -> tuple[set[int], set[int], set[int]]:
    if name == "nominal":
        return set(), set(), set()
    if name == "gps-loss":
        return {8, 9, 10}, set(), set()
    if name == "faults":
        return {8, 9, 10}, {5, 14}, {12}
    raise ValueError(name)


def print_results(results: list[CycleResult], print_frames: bool = False) -> None:
    print("cycle  gps       truth_m  fix_m  tries  status  sleep      meas  tx")
    for result in results:
        gps_state = "OK" if result.gps_acquired else "DEGRADED"
        tx_state = "sent" if result.tx_success else "FAILED"
        measurement_state = "OK" if result.measurement_ok else "FAIL"
        print(
            f"{result.cycle:>5}  {gps_state:<8}  {result.truth.altitude_m:>7}  "
            f"{result.fix.altitude_m:>5}  {result.gps_attempts:>5}  "
            f"{result.fix.op_status:04d}    {result.sleep_intervals}x{AWU_INTERVAL_SECONDS:.3f}s  "
            f"{measurement_state:<4}  {tx_state}"
        )
        if print_frames:
            print(f"       {result.frame}")

    degraded = sum(not result.gps_acquired for result in results)
    measurement_failures = sum(not result.measurement_ok for result in results)
    tx_failures = sum(not result.tx_success for result in results)
    double_sleep = sum(result.sleep_intervals == 2 for result in results)
    total_sleep = sum(result.sleep_seconds for result in results)
    print(
        f"Summary: cycles={len(results)} degraded={degraded} measurement_failures={measurement_failures} "
        f"tx_failures={tx_failures} double_sleep={double_sleep} sleep_time={total_sleep:.3f}s"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=("nominal", "gps-loss", "faults"), default="nominal")
    parser.add_argument("--cycles", type=int, help="limit the synthetic flight to the first N cycles")
    parser.add_argument("--gps-loss", default="", help="additional GPS-loss cycles, e.g. 8-10,14")
    parser.add_argument("--measurement-fail", default="", help="measurement-failure cycles")
    parser.add_argument("--tx-fail", default="", help="radio transmit-failure cycles")
    parser.add_argument("--gps-attempts", type=int, choices=range(1, 16), default=1,
                        help="successful GPS acquisition attempt (1-15)")
    parser.add_argument("--print-frames", action="store_true", help="print each generated telemetry frame")
    parser.add_argument("--capture", type=Path, help="write transmitted raw frames to a decoder-compatible capture")
    args = parser.parse_args()

    points = synthetic_profile()
    if args.cycles is not None:
        if args.cycles < 1 or args.cycles > len(points):
            parser.error(f"--cycles must be between 1 and {len(points)}")
        points = points[:args.cycles]

    gps_loss, measurement_fail, tx_fail = scenario_faults(args.scenario)
    try:
        gps_loss |= parse_cycle_set(args.gps_loss)
        measurement_fail |= parse_cycle_set(args.measurement_fail)
        tx_fail |= parse_cycle_set(args.tx_fail)
    except (ValueError, argparse.ArgumentTypeError) as exc:
        parser.error(str(exc))

    results = run_simulation(
        points,
        gps_loss_cycles=gps_loss,
        measurement_fail_cycles=measurement_fail,
        tx_fail_cycles=tx_fail,
        gps_success_attempt=args.gps_attempts,
    )

    print(f"PicoTracker simulator: scenario={args.scenario} cycles={len(results)}")
    print("Synthetic control/telemetry model; not an RF, power, or atmospheric physics simulation.")
    print_results(results, print_frames=args.print_frames)

    if args.capture:
        args.capture.parent.mkdir(parents=True, exist_ok=True)
        transmitted = "".join(result.raw_frame for result in results if result.tx_success)
        args.capture.write_text(transmitted, encoding="ascii")
        print(f"Wrote {sum(result.tx_success for result in results)} transmitted frames to {args.capture}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
