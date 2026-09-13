# Host tests

These tests exercise hardware-independent firmware and tooling logic on the development machine. Current coverage includes:

- fixed-width numeric formatting
- telemetry latitude, longitude, and temperature formatting
- maximum telemetry-frame capacity and dynamic field layout
- telemetry CRC reference and captured-frame vectors
- GPS operational-status packing
- u-blox checksum and NAV-PVT fix-validity rules
- synthetic UBX byte-stream packets, checksum rejection, payload bounds, and strict ACK/NAK parsing
- radio temperature conversion
- radio synthesizer fixed-point calculations and supported frequency bands
- GPS altitude conversion and clamp behavior
- altitude-dependent sleep policy
- Python telemetry decoder CRC handling and sample-capture decoding
- hardware-independent simulator behaviour, fault recovery, telemetry vectors, and decoder-compatible captures
- release metadata, release-note extraction, and IAR project-reference integrity

From the repository root, run the complete host suite with:

```sh
make test
```

The host tests complement the STM8/IAR build and physical hardware validation; they do not replace target testing.

## STM8 structural target check

When SDCC with STM8 support is installed, run:

```sh
make stm8
```

This compiles and links the production C sources with SDCC's STM8 backend and checks the STM8S003F3 flash/RAM window. A compatibility shim supplies the IAR register names and removes IAR-specific interrupt declarations, so the generated image is **not flashable** and does not replace the required IAR build or hardware validation. It provides an independent STM8 data-model, code-generation, cross-module link, and approximate size check.
