#!/usr/bin/env python3
"""Decode PicoTracker telemetry captures and verify CRC-16 checksums."""

from __future__ import annotations

import argparse
import csv
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


def analyse_data(file_path: Path, print_frames: bool = False) -> list[list[str]]:
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
                        print(data)
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
                    print(data)

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
    args = parser.parse_args(argv)

    files = args.files or [Path(__file__).with_name("with_pips_data.txt")]
    datasets: list[tuple[str, list[int]]] = []
    accepted_frames: list[list[str]] = []

    for file_path in files:
        frames = analyse_data(file_path, print_frames=args.print_frames)
        accepted_frames.extend(frames)
        deltas = cycle_deltas(frames)
        datasets.append((file_path.name, deltas))
        print(f"{file_path}: {len(frames)} valid frames")
        if deltas:
            print(f"  cycle duration: min={min(deltas)}s max={max(deltas)}s")

    if args.csv_output:
        write_csv(accepted_frames, args.csv_output)
        print(f"{args.csv_output}: wrote {len(accepted_frames)} valid frames")

    if args.plot:
        plot_deltas(datasets)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
