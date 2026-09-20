# Tools

Development and analysis helpers for PicoTracker telemetry data are kept here.

## Telemetry decoder

`decode_data.py` scans raw DL-fldigi captures, validates the four-digit telemetry CRC, and reports valid frames and cycle timing. Basic decoding uses only the Python standard library.

Run it against the included sample capture:

```sh
python3 tools/decode_data.py
```

Pass one or more capture files explicitly, or add `--print-frames` to print each accepted frame:

```sh
python3 tools/decode_data.py capture.txt --print-frames
```

Use `--csv-output` to write all CRC-valid frames from the supplied captures to a
CSV file with headings matching the documented telemetry fields:

```sh
python3 tools/decode_data.py capture.txt --csv-output flight-data.csv
```

Invalid frames are omitted. When multiple captures are supplied, their accepted
frames are written to the same CSV file in command-line order.

Plotting is optional. Install the pinned plotting dependency and use `--plot`:

```sh
python3 -m pip install -r tools/requirements.txt
python3 tools/decode_data.py --plot
```

`with_pips_data.txt` is retained as a sample captured dataset and is also exercised by the host regression tests.

## Tracker simulator

`simulate_tracker.py` runs a deterministic hardware-independent flight cycle, including GPS degraded mode, sleep cadence, status packing, telemetry generation, and optional fault injection. From the repository root run:

```sh
make simulate
make simulate SIM_ARGS="--scenario gps-loss --print-frames"
```

See [`docs/simulator.md`](../docs/simulator.md) for scenarios, fault injection, capture generation, and fidelity limits.

## Release metadata helper

`release.py` validates release tags against `VERSION` and `CHANGELOG.md` and renders the notes used by the automated release workflow. From the repository root:

```sh
make release-check RELEASE_TAG=vX.Y.Z
make release-notes RELEASE_TAG=vX.Y.Z
```

See [`docs/release-automation.md`](../docs/release-automation.md) for the publication workflow.
