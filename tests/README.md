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
- altitude-dependent sleep policy
- Python telemetry decoder CRC handling and sample-capture decoding
- release metadata and IAR project-reference integrity

Run the complete host suite with:

```sh
cd tests
make test
```

The host tests complement the STM8/IAR build and physical hardware validation; they do not replace target testing.
