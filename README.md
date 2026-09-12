# PicoTracker

[![Host tests](https://github.com/ImperialSpaceSociety/PicoTracker/actions/workflows/host-tests.yml/badge.svg)](https://github.com/ImperialSpaceSociety/PicoTracker/actions/workflows/host-tests.yml)

PicoTracker is a lightweight, low-cost high-altitude balloon tracking platform built around an STM8 processor, HC-12/Si4463 radio, u-blox GPS receiver, and 433 MHz RTTY telemetry. The project was originally developed by the Imperial College Space Society.

### Project status

Active maintenance resumed in 2026. The current development version is [`1.4.0-dev`](VERSION). Firmware correctness, regression testing, documentation, and release controls have been substantially strengthened, but a release tag will not be created until the target IAR build and hardware-validation gates pass. The original hardware and firmware date from 2018–2019, so anyone building a unit today should revalidate component availability, prices, tooling, and test assumptions.

### Maintainer

This repository is maintained and administered by [Sylvester Kaczmarek](https://SylvesterKaczmarek.com). Questions about maintenance, proposed technical changes, release coordination, or collaboration can be raised through GitHub issues where appropriate or directed to the maintainer through the website.

## Repository layout

- [`firmware/`](firmware/) - STM8 tracker firmware
- [`test_firmware/`](test_firmware/) - historical diagnostic projects; not release-qualified
- [`hardware/`](hardware/) - hardware design files
- [`cad/`](cad/) - mechanical/CAD files
- [`docs/`](docs/) - project guides and technical references
- [`tools/`](tools/) - supporting scripts
- [`tests/`](tests/) - host-side regression tests for firmware logic

### Key documentation

- [`docs/development.md`](docs/development.md) - development, programming, and debugging
- [`docs/gps.md`](docs/gps.md) - GPS hardware and integration notes
- [`docs/hc12.md`](docs/hc12.md) - HC-12 radio and processor notes
- [`docs/telemetry-format.md`](docs/telemetry-format.md) - transmitted telemetry field order
- [`docs/status-word.md`](docs/status-word.md) - operational-status field layout
- [`tests/README.md`](tests/README.md) - host-test scope and usage
- [`docs/release-checklist.md`](docs/release-checklist.md) - release gates and publication checklist
- [`docs/hardware-validation.md`](docs/hardware-validation.md) - target hardware validation matrix
- [`CHANGELOG.md`](CHANGELOG.md) - development and release history

## Verification

GitHub Actions runs the host regression suite on pushes to `master` and on pull requests. Coverage includes telemetry formatting, maximum frame sizing and CRCs, status packing, UBX payload and ACK/NAK parsing, GPS fix validity, radio temperature conversion, sleep policy, repository metadata, IAR project-file references, and the Python telemetry decoder. A separate SDCC structural target check also compiles and links the production sources with an STM8 backend and verifies that the resulting structural image remains within the STM8S003F3 flash/RAM window. That check uses compatibility shims and is not flashable. The remaining release gates are the native IAR STM8 build and completion of the hardware validation matrix.

## Objectives

The Pico balloon tracker was designed as an easy-to-build, low-cost entry point for small helium-filled balloons carrying a tracking payload.

- Low mass: less than 25 g
- Low cost: historically targeted below £15
- Compatible with existing HAB tracking systems using 433 MHz RTTY
- Quick build using off-the-shelf modules where possible

## Launch footage

[![PicoTracker high-altitude balloon launch](https://img.youtube.com/vi/OtdXHd_AjtY/hqdefault.jpg)](https://www.youtube.com/watch?v=OtdXHd_AjtY)

Launch footage from one of the PicoTracker high-altitude balloon flights. Click the preview to watch the video.

## Design

### GPS

The design uses a u-blox M8-based GPS module suitable for high-altitude balloon use. The original modules contained a u-blox G8030 chip with a TCXO, battery, external flash, LNA, and SAW filter. The original plan was to remove the metal shield and ceramic antenna and replace the antenna with a guitar-wire antenna and ground plane. Historical module mass was approximately 1 g to 13.5 g depending on module and antenna.

### 433 MHz radio and processor

The tracker uses an HC12 Si4463-based radio with an STM8S003F3 processor. The HC12 module is normally supplied with firmware for a serial interface, but its processor can be reprogrammed for the tracker. The STM8S003F3 has 8 KB of flash and 1 KB of RAM, so firmware size remains an important constraint. Historical module mass was approximately 1 g.

### Battery

The battery supply is based on the earlier [BatteryAAA](https://github.com/ribbotson/rlabTelemetryTx/tree/master/Hardware/BatteryAAA) design, using a AAA lithium primary cell and boost converter to provide 3.3 V to the GPS, radio, and processor. Historical mass was approximately 8 g for the battery and 3.5 g for the holder and boost converter.

### Tracker body

The tracker body can be 3D printed or made from polystyrene foam. The original antenna arrangement placed the GPS antenna at the top and the 433 MHz radio antenna at the bottom.

### Firmware

The firmware was derived from earlier Bristol SEDS pico-tracker work and adapted for the more constrained STM8 processor. See [`docs/development.md`](docs/development.md) for programming and debugging notes.

### Tracking

The tracker was designed to work with existing high-altitude balloon tracking networks and software using 433 MHz RTTY. Reception can be performed with an SDR receiver, with RTTY decoding handled by compatible HAB software.

## Design challenges

- **Power consumption:** the original modules were not specifically designed for low-power operation, so firmware-based power management is important.
- **Low temperature:** operation at high altitude can expose the electronics to temperatures around -50 °C, requiring validation of oscillators, power systems, and packaging.

## Contributing

Small maintenance fixes, documentation improvements, test notes, and well-scoped technical contributions are welcome. See [`CONTRIBUTING.md`](CONTRIBUTING.md).

## License

PicoTracker uses separate licenses for software and hardware design material:

- Software and firmware: [MIT License](LICENSES/MIT.txt).
- Hardware design documentation: [CERN Open Hardware Licence v1.2](LICENSES/CERN-OHL-1.2.txt), or a later version where the existing project notice permits it.

See [`LICENSE.md`](LICENSE.md) for the repository license split and applicable notices.
