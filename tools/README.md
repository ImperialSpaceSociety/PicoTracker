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

Plotting is optional. Install the pinned plotting dependency and use `--plot`:

```sh
python3 -m pip install -r tools/requirements.txt
python3 tools/decode_data.py --plot
```

`with_pips_data.txt` is retained as a sample captured dataset and is also exercised by the host regression tests.
