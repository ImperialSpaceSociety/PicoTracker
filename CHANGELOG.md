# Changelog

All notable changes to PicoTracker are documented here.

## 1.4.0 - 2026-09-13

### Added

- Host-side regression tests for numeric formatting, UBX protocol handling, telemetry CRC and field formatting, status packing, radio temperature conversion, sleep policy, and telemetry decoding.
- Bounded GPS acquisition and radio command failure handling.
- Explicit UBX checksum, payload-length, and navigation-fix validation.
- Reorganized project documentation, test infrastructure, and release-preparation material.
- Added a repeatable SDCC STM8 structural compile/link and flash/RAM size check.

### Changed

- Radio initialization now follows a reset and `POWER_UP` sequence before normal commands.
- Oscillator source and frequency configuration are centralized.
- Altitude-dependent sleep behavior is explicit and host-tested.
- Telemetry decoder runs without third-party dependencies for normal decoding.
- Replaced floating-point radio synthesizer arithmetic with exact integer fixed-point calculations and removed unused firmware helpers to reduce flash use.

### Fixed

- High-altitude and longitude numeric formatting overflow.
- Potential UBX payload-buffer overflow, malformed ACK acceptance, and acceptance of corrupted packets.
- Maximum-length telemetry frame buffer sizing and signed 8-bit transmission-length limits.
- Unbounded GPS, UART, clock-switch, and Si4463 peripheral waits in normal operation.
- Optimizer-removable GPS and radio timing delays.
- Radio ADC and RTTY channel-switch failures that could otherwise be reported or transmitted silently.
- Degraded GPS acquisition overwriting the last accepted navigation solution.
- Signed radio-temperature conversion, field saturation, and TCXO power-down handling.

## 1.3 - 2019-03-07

Historical release tagged `v1.3` in the repository.
