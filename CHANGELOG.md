# Changelog

All notable changes to PicoTracker are documented here.

## Unreleased - 1.4.0-dev

### Added

- Host-side regression tests for numeric formatting, UBX protocol handling, telemetry CRC and field formatting, status packing, radio temperature conversion, sleep policy, and telemetry decoding.
- Bounded GPS acquisition and radio command failure handling.
- Explicit UBX checksum, payload-length, and navigation-fix validation.
- Reorganized project documentation, test infrastructure, and release-preparation material.

### Changed

- Radio initialization now follows a reset and `POWER_UP` sequence before normal commands.
- Oscillator source and frequency configuration are centralized.
- Altitude-dependent sleep behavior is explicit and host-tested.
- Telemetry decoder runs without third-party dependencies for normal decoding.

### Fixed

- High-altitude and longitude numeric formatting overflow.
- Potential UBX payload-buffer overflow and acceptance of corrupted packets.
- Unbounded GPS and Si4463 wait loops in normal operation.
- Signed radio-temperature conversion and TCXO power-down handling.

## 1.3 - 2019-03-07

Historical release tagged `v1.3` in the repository.
