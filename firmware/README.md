# Firmware

This directory contains the STM8 firmware for the PicoTracker HC12-based tracker.

## Main files

- `main.c` / `main.h` - application startup, GPS acquisition, measurements, and tracker control flow.
- `gps.c` / `gps.h` - u-blox GPS configuration, polling, and fix handling.
- `telemetry.c` / `telemetry.h` - telemetry generation and transmission control.
  See [`../docs/telemetry-format.md`](../docs/telemetry-format.md) for the transmitted field order.
- `energy.c` / `energy.h` - power and sleep-related functions.
- `si_trx.c` / `si_trx.h` - Si4463 radio interface.
- `spi_bitbang.c` / `spi_bitbang.h` - software SPI interface used by the radio code.
- `rtty.c` / `rtty.h` - RTTY support.
- `HC12Board.h` - HC12 board and STM8 hardware definitions.
- `HC12Tracker.ewp` - IAR Embedded Workbench project file.

Additional headers and helpers provide numeric formatting, UBX parsing, status packing, telemetry CRC/field formatting, radio measurement conversion, and sleep-policy logic.

## Development

See [`../docs/development.md`](../docs/development.md) for build, programming, and debugging notes. Generated IAR workspace, debugger, log, and build-output files are intentionally excluded by the repository `.gitignore`.
