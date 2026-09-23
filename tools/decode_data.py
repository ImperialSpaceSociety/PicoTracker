#!/usr/bin/env python3
"""Decode PicoTracker telemetry captures and verify CRC-16 checksums."""

from __future__ import annotations

import argparse
import csv
import json
from collections.abc import Iterable
from pathlib import Path

POLYNOMIAL = 0x1021
PRESET = 0xFFFF
FIELD_COUNT = 10
CSV_FIELDS = [
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
]

GPS_FIX_ATTEMPTS_MAX = 15
OP_STATUS_ERROR_MASK = 0x03
OP_STATUS_CFG_SHIFT = 2
OP_STATUS_FIX_SHIFT = 4
OP_STATUS_OK = 0
OP_STATUS_TRANSIENT_ERROR = 1
OP_STATUS_RETRY_EXHAUSTED = 2
OP_STATUS_DEGRADED = 3
OP_STATUS_MEASUREMENT_ERROR = 0x0100

CONFIG_STATUS_NAMES = {
    OP_STATUS_OK: "ok",
    OP_STATUS_TRANSIENT_ERROR: "transient-error",
    OP_STATUS_RETRY_EXHAUSTED: "retry-exhausted",
}

POLL_STATUS_NAMES = {
    OP_STATUS_OK: "ok",
    OP_STATUS_TRANSIENT_ERROR: "transient-error",
    OP_STATUS_RETRY_EXHAUSTED: "retry-exhausted",
    OP_STATUS_DEGRADED: "degraded",
}


class DecodedOpStatus:
    """Decoded diagnostic fields from a 16-bit packed operational status word."""

    def __init__(
        self,
        raw: int,
        fix_attempts: int,
        config_status: int,
        config_status_text: str,
        poll_status: int,
        poll_status_text: str,
        degraded: bool,
        measurement_error: bool,
    ) -> None:
        self.raw = raw
        self.fix_attempts = fix_attempts
        self.config_status = config_status
        self.config_status_text = config_status_text
        self.poll_status = poll_status
        self.poll_status_text = poll_status_text
        self.degraded = degraded
        self.measurement_error = measurement_error

    @property
    def is_degraded(self) -> bool:
        return self.degraded

    @property
    def radio_measurement_failed(self) -> bool:
        return self.measurement_error

    def format(self) -> str:
        radio_text = "failed" if self.measurement_error else "ok"
        return (
            f"attempts={self.fix_attempts}, "
            f"config={self.config_status_text}, "
            f"poll={self.poll_status_text}, "
            f"radio={radio_text}"
        )

    def __str__(self) -> str:
        return self.format()

    def __repr__(self) -> str:
        return (
            f"DecodedOpStatus(raw={self.raw}, fix_attempts={self.fix_attempts}, "
            f"config_status={self.config_status}, config_status_text={self.config_status_text!r}, "
            f"poll_status={self.poll_status}, poll_status_text={self.poll_status_text!r}, "
            f"degraded={self.degraded}, measurement_error={self.measurement_error})"
        )

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, DecodedOpStatus):
            return NotImplemented
        return (
            self.raw == other.raw
            and self.fix_attempts == other.fix_attempts
            and self.config_status == other.config_status
            and self.config_status_text == other.config_status_text
            and self.poll_status == other.poll_status
            and self.poll_status_text == other.poll_status_text
            and self.degraded == other.degraded
            and self.measurement_error == other.measurement_error
        )


def decode_op_status(value: int | str) -> DecodedOpStatus:
    """Decode 16-bit packed telemetry operational status into diagnostic fields."""
    if isinstance(value, str):
        cleaned = value.strip()
        if cleaned.lower().startswith("0x"):
            raw = int(cleaned, 16)
        else:
            raw = int(cleaned, 10)
    else:
        raw = int(value)

    if raw < 0:
        raise ValueError(f"Operational status cannot be negative: {value}")

    fix_attempts = (raw >> OP_STATUS_FIX_SHIFT) & GPS_FIX_ATTEMPTS_MAX
    config_status = (raw >> OP_STATUS_CFG_SHIFT) & OP_STATUS_ERROR_MASK
    poll_status = raw & OP_STATUS_ERROR_MASK
    measurement_error = bool(raw & OP_STATUS_MEASUREMENT_ERROR)

    config_text = CONFIG_STATUS_NAMES.get(config_status, f"unknown({config_status})")
    poll_text = POLL_STATUS_NAMES.get(poll_status, f"unknown({poll_status})")
    degraded = poll_status == OP_STATUS_DEGRADED

    return DecodedOpStatus(
        raw=raw,
        fix_attempts=fix_attempts,
        config_status=config_status,
        config_status_text=config_text,
        poll_status=poll_status,
        poll_status_text=poll_text,
        degraded=degraded,
        measurement_error=measurement_error,
    )


def format_op_status(status: int | str | DecodedOpStatus) -> str:
    """Return a human-readable interpretation string for an op_status value."""
    if not isinstance(status, DecodedOpStatus):
        status = decode_op_status(status)
    return status.format()


def summarize_op_status(frames: Iterable[list[str]]) -> dict[str, int]:
    """Aggregate operational-status diagnostics across decoded telemetry frames."""
    statuses = [decode_op_status(frame[8]) for frame in frames if len(frame) > 8]
    if not statuses:
        return {
            "total": 0,
            "min_attempts": 0,
            "max_attempts": 0,
            "degraded": 0,
            "config_errors": 0,
            "poll_errors": 0,
            "radio_errors": 0,
        }
    attempts = [s.fix_attempts for s in statuses]
    return {
        "total": len(statuses),
        "min_attempts": min(attempts),
        "max_attempts": max(attempts),
        "degraded": sum(1 for s in statuses if s.degraded),
        "config_errors": sum(1 for s in statuses if s.config_status != OP_STATUS_OK),
        "poll_errors": sum(1 for s in statuses if s.poll_status != OP_STATUS_OK),
        "radio_errors": sum(1 for s in statuses if s.measurement_error),
    }


def _initial(value: int) -> int:
    crc = 0
    value <<= 8
    for _ in range(8):
        crc = ((crc << 1) ^ POLYNOMIAL) if (crc ^ value) & 0x8000 else crc << 1
        value <<= 1
    return crc & 0xFFFF


_CRC_TABLE = [_initial(i) for i in range(256)]


def _update_crc(crc: int, value: int) -> int:
    index = ((crc >> 8) ^ value) & 0xFF
    return ((crc << 8) ^ _CRC_TABLE[index]) & 0xFFFF


def crc(text: str) -> int:
    value = PRESET
    for char in text:
        value = _update_crc(value, ord(char))
    return value


def bytes_from_file(filename: Path, chunk_size: int = 4096) -> Iterable[int]:
    with filename.open("rb") as capture:
        while chunk := capture.read(chunk_size):
            yield from chunk


def format_frame(data: str, fields: list[str], decode_status: bool = False) -> str:
    if decode_status and len(fields) > 8:
        return f"{data}  [op_status: {format_op_status(fields[8])}]"
    return data


def analyse_data(
    file_path: Path, print_frames: bool = False, decode_status: bool = False
) -> list[list[str]]:
    frames: list[list[str]] = []
    recent = [None, None, None]
    data = ""
    checksum = ""
    adding_data = False
    adding_checksum = False

    for byte in bytes_from_file(file_path):
        if adding_checksum and len(checksum) == 4:
            expected = f"{crc(data):04X}"
            if expected == checksum.upper():
                fields = data.split(",")
                if len(fields) == FIELD_COUNT:
                    frames.append(fields)
                    if print_frames:
                        print(format_frame(data, fields, decode_status=decode_status))
            data = ""
            checksum = ""
            adding_checksum = False

        char = chr(byte)
        if adding_checksum:
            checksum += char

        if adding_data and char == "*":
            adding_data = False
            adding_checksum = True
        elif adding_data:
            data += char

        recent[0], recent[1], recent[2] = recent[1], recent[2], byte
        if recent[0] == 36 and recent[1] == 36 and recent[2] != 36:
            adding_data = True
            data = chr(recent[2])

    if adding_checksum and len(checksum) == 4:
        expected = f"{crc(data):04X}"
        if expected == checksum.upper():
            fields = data.split(",")
            if len(fields) == FIELD_COUNT:
                frames.append(fields)
                if print_frames:
                    print(format_frame(data, fields, decode_status=decode_status))

    return frames


def _time_to_seconds(value: str) -> int:
    return int(value[0:2]) * 3600 + int(value[2:4]) * 60 + int(value[4:6])


def cycle_deltas(frames: list[list[str]]) -> list[int]:
    deltas: list[int] = []
    previous = None
    for frame in frames:
        current = _time_to_seconds(frame[2])
        if previous is not None:
            delta = current - previous
            if delta < 0:
                delta += 24 * 3600
            deltas.append(delta)
        previous = current
    return deltas


def write_csv(frames: Iterable[list[str]], output_path: Path) -> None:
    with output_path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.writer(output)
        writer.writerow(CSV_FIELDS)
        writer.writerows(frames)


def write_json(frames: Iterable[list[str]], output_path: Path) -> None:
    """Write accepted telemetry fields as strings, preserving their raw values."""
    records = [dict(zip(CSV_FIELDS, frame, strict=True)) for frame in frames]
    with output_path.open("w", encoding="utf-8") as output:
        json.dump(records, output, indent=2)
        output.write("\n")


def plot_deltas(datasets: list[tuple[str, list[int]]]) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit(
            "Plotting requires: python -m pip install -r tools/requirements.txt"
        ) from exc

    for label, deltas in datasets:
        if deltas:
            plt.hist(deltas, bins="auto", alpha=0.5, label=label)
    plt.xlabel("cycle duration (s)")
    plt.ylabel("frequency")
    plt.legend()
    plt.show()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("files", nargs="*", type=Path, help="raw DL-fldigi capture files")
    parser.add_argument("--plot", action="store_true", help="plot cycle-duration histograms")
    parser.add_argument(
        "--print-frames", action="store_true", help="print each valid telemetry frame"
    )
    parser.add_argument(
        "--csv-output", type=Path, help="write valid telemetry frames to this CSV file"
    )
    parser.add_argument(
        "--json-output", type=Path, help="write valid telemetry frames to this JSON file"
    )
    parser.add_argument(
        "--decode-status",
        action="store_true",
        help="decode operational status (op_status) into human-readable diagnostics",
    )
    args = parser.parse_args(argv)

    files = args.files or [Path(__file__).with_name("with_pips_data.txt")]
    datasets: list[tuple[str, list[int]]] = []
    accepted_frames: list[list[str]] = []

    for file_path in files:
        frames = analyse_data(
            file_path, print_frames=args.print_frames, decode_status=args.decode_status
        )
        accepted_frames.extend(frames)
        deltas = cycle_deltas(frames)
        datasets.append((file_path.name, deltas))
        print(f"{file_path}: {len(frames)} valid frames")
        if deltas:
            print(f"  cycle duration: min={min(deltas)}s max={max(deltas)}s")
        if args.decode_status and frames:
            summary = summarize_op_status(frames)
            print(
                f"  op_status summary: attempts={summary['min_attempts']}..{summary['max_attempts']} "
                f"degraded={summary['degraded']} "
                f"config_errors={summary['config_errors']} "
                f"poll_errors={summary['poll_errors']} "
                f"radio_errors={summary['radio_errors']}"
            )

    if args.csv_output:
        write_csv(accepted_frames, args.csv_output)
        print(f"{args.csv_output}: wrote {len(accepted_frames)} valid frames")

    if args.json_output:
        write_json(accepted_frames, args.json_output)
        print(f"{args.json_output}: wrote {len(accepted_frames)} valid frames")

    if args.plot:
        plot_deltas(datasets)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
