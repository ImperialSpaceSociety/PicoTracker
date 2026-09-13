# PicoTracker

[![Release](https://img.shields.io/github/v/release/ImperialSpaceSociety/PicoTracker?display_name=tag)](https://github.com/ImperialSpaceSociety/PicoTracker/releases/latest)
[![Host + STM8 checks](https://github.com/ImperialSpaceSociety/PicoTracker/actions/workflows/host-tests.yml/badge.svg)](https://github.com/ImperialSpaceSociety/PicoTracker/actions/workflows/host-tests.yml)
[![MCU: STM8S003F3](https://img.shields.io/badge/MCU-STM8S003F3-informational)](firmware/)
[![Software: MIT](https://img.shields.io/badge/software-MIT-blue.svg)](LICENSES/MIT.txt)
[![Hardware: CERN OHL v1.2](https://img.shields.io/badge/hardware-CERN%20OHL%20v1.2-blue.svg)](LICENSES/CERN-OHL-1.2.txt)

PicoTracker is a lightweight, low-cost high-altitude balloon tracking platform built around an STM8 microcontroller, HC-12 radio hardware, u-blox GPS, and 433 MHz RTTY telemetry. The project was originally developed by the Imperial College Space Society and returned to active maintenance in 2026.

### Project status

[`v1.4.0`](https://github.com/ImperialSpaceSociety/PicoTracker/releases/tag/v1.4.0) is the current maintained release. It is a source-only release focused on firmware correctness, bounded failure handling, regression testing, STM8 size verification, and clearer project documentation. The original hardware dates from 2018-2019, so current builders should still revalidate component availability, RF configuration, power behaviour, and environmental assumptions.

### Maintainer

This repository is maintained and administered by [Sylvester Kaczmarek](https://SylvesterKaczmarek.com). Maintenance questions, technical changes, release coordination, and collaboration proposals can be raised through GitHub issues or directed to the maintainer through the website.

## At a glance

| Item | Current project configuration |
| --- | --- |
| MCU | STM8S003F3, 8 KB flash, 1 KB RAM |
| Radio | HC-12 hardware using Si4463 or compatible Si4438 paths |
| Default RF frequency | 434.570 MHz |
| Telemetry | 50 baud RTTY with XMODEM CRC |
| GPS | u-blox M8 family |
| Radio oscillator | 32 MHz TCXO by default; 30 MHz crystal compatibility retained |
| Processor clock | 16 MHz internal HSI |
| Power | AAA lithium primary cell with 3.3 V boost stage in the original design |
| Current release | [`v1.4.0`](https://github.com/ImperialSpaceSociety/PicoTracker/releases/tag/v1.4.0) |
| Production project | [`firmware/HC12Tracker.ewp`](firmware/HC12Tracker.ewp) |

## System architecture

```mermaid
flowchart LR
    BAT["AAA lithium cell"] --> BOOST["3.3 V boost converter"]
    BOOST --> MCU["STM8S003F3<br/>8 KB flash / 1 KB RAM"]
    BOOST --> GPS["u-blox M8 GPS"]
    BOOST --> RADIO["HC-12 radio<br/>Si4463 / Si4438"]
    MCU <-->|UART| GPS
    MCU <-->|SPI| RADIO
    TCXO["32 MHz TCXO<br/>30 MHz crystal compatible"] --> RADIO
    RADIO --> ANT["433 MHz antenna"]
    ANT -. "434.570 MHz / 50 baud RTTY" .-> GROUND["Ground receiver / HAB decoder"]
```

The STM8 runs from its 16 MHz internal HSI clock. The separate 32 MHz TCXO shown above is the maintained default oscillator for the Si4463 radio; the original 30 MHz crystal path is retained as a compatibility configuration.

## What changed in v1.4.0

`v1.4.0` is the first maintained release following the 2026 firmware and repository hardening work. Major changes include:

- strict UBX payload length, checksum, ACK/NAK, and valid-3D-fix handling
- bounded GPS, UART, STM8 clock-switch, and Si4463 command waits
- corrected Si4463 reset and `POWER_UP` sequencing with explicit error propagation
- corrected telemetry frame sizing, numeric formatting, temperature handling, and operational-status reporting
- preserved last-known valid GPS data when acquisition enters degraded mode
- restored and tested altitude-dependent sleep/wake behaviour and HSI clock recovery
- removed floating-point radio synthesizer arithmetic and reduced firmware footprint for the 8 KB STM8 target
- added host regression tests, telemetry decoder tests, repository metadata checks, and a repeatable STM8 structural compile/link check

See [`CHANGELOG.md`](CHANGELOG.md) for the detailed release history.

## How the tracker operates

A normal cycle acquires and validates a u-blox NAV-PVT solution, reads radio voltage and temperature, constructs a CRC-protected telemetry sentence, transmits it over 433 MHz RTTY, powers down the radio and GPS as appropriate, and enters STM8 auto-wakeup sleep. GPS acquisition and radio command paths are bounded so loss of GPS or a radio CTS failure does not create an intentional infinite wait. If a new valid GPS solution cannot be obtained, the tracker continues in degraded mode while retaining the most recently accepted fix.

At altitudes up to 3000 m the maintained firmware uses one auto-wakeup sleep interval. Above 3000 m it uses two intervals before returning to the HSI clock and beginning the next acquisition/transmit cycle.

```mermaid
flowchart TD
    WAKE["Wake from auto-wakeup sleep"] --> HSI["Restore HSI clock"]
    HSI --> GPS["Wake / configure GPS"]
    GPS --> FIX["Acquire NAV-PVT solution"]
    FIX --> VALID{"Valid 3D fix + gnssFixOK?"}
    VALID -- Yes --> MEASURE["Measure radio voltage and temperature"]
    VALID -- "No, retry budget exhausted" --> DEGRADED["Degraded mode<br/>retain last valid fix + update status"]
    DEGRADED --> MEASURE
    MEASURE --> FRAME["Build telemetry sentence + CRC"]
    FRAME --> TX["Transmit 434.570 MHz RTTY"]
    TX --> POWER["Power down radio / GPS as appropriate"]
    POWER --> ALT{"Altitude > 3000 m?"}
    ALT -- Yes --> SLEEP2["Sleep 2 AWU intervals"]
    ALT -- No --> SLEEP1["Sleep 1 AWU interval"]
    SLEEP2 --> WAKE
    SLEEP1 --> WAKE
```

## Configuration

The main flight configuration is intentionally concentrated in a small number of files:

| Setting | Default | Location |
| --- | --- | --- |
| Payload name | `ICSPACE14` | [`firmware/main.h`](firmware/main.h) |
| RF frequency | `434570000` Hz | [`firmware/main.h`](firmware/main.h) |
| RTTY timing | 50 baud configuration | [`firmware/main.h`](firmware/main.h) |
| Radio oscillator | 32 MHz TCXO | [`firmware/HC12Board.h`](firmware/HC12Board.h) |
| 30 MHz crystal compatibility | Supported by removing `XO_TCXO` | [`firmware/HC12Board.h`](firmware/HC12Board.h) |
| Sleep cadence | 1 interval through 3000 m, 2 above | [`firmware/sleep_policy.h`](firmware/sleep_policy.h) |

Review these settings before programming hardware for a new payload or flight. RF frequency, output power, antenna configuration, and balloon operation must also comply with the applicable local requirements.

## Verification and testing

GitHub Actions runs both the host regression suite and the STM8 structural target check on pushes to `master` and on pull requests. The host suite covers numeric formatting, telemetry layout and CRCs, UBX packet parsing and ACK/NAK handling, GPS fix validity, status packing, radio measurements, synthesizer calculations, sleep policy, decoder behaviour, release metadata, and IAR project-reference integrity.

For `v1.4.0`, the CI SDCC 4.2 structural build used **7,787 / 8,192 bytes** of flash span and **130 / 1,024 bytes** of static DATA. The structural SDCC build uses compatibility shims for IAR-specific registers and interrupt declarations, so it is a target-compiler and memory-window check, not a flashable firmware artifact. The release was approved by the maintainer following target/hardware validation; detailed native-IAR logs and quantitative hardware measurements were not archived with that release.

Run the host suite from the repository root:

```sh
make -C tests clean test
```

With an STM8-capable SDCC installation, run the independent target-structure check:

```sh
make -C tests stm8
```

Decode the included telemetry capture with:

```sh
python3 tools/decode_data.py tools/with_pips_data.txt
```

The maintained production project for native STM8 development is [`firmware/HC12Tracker.ewp`](firmware/HC12Tracker.ewp). See [`docs/development.md`](docs/development.md) for programming and debugging notes.

## Telemetry

PicoTracker transmits a comma-separated RTTY sentence containing the payload name, sentence ID, UTC time, signed latitude and longitude, altitude, satellite count, radio supply voltage, packed operational status, radio temperature, and a four-digit XMODEM CRC.

```text
PAYLOAD,ID,HHMMSS,+LAT,-LON,ALT,SATS,VOLTAGE,STATUS,+TEMP*CRC
```

The exact field order is documented in [`docs/telemetry-format.md`](docs/telemetry-format.md). The packed status field, including GPS acquisition and measurement failure information, is documented in [`docs/status-word.md`](docs/status-word.md).

## Repository layout

| Path | Purpose |
| --- | --- |
| [`firmware/`](firmware/) | Maintained STM8 tracker firmware and IAR project |
| [`tests/`](tests/) | Host regression tests and STM8 structural target check |
| [`tools/`](tools/) | Telemetry decoder and sample captured data |
| [`hardware/`](hardware/) | Historical hardware design and reference files |
| [`cad/`](cad/) | Mechanical and printable tracker parts |
| [`docs/`](docs/) | Development, GPS, radio, telemetry, release, and validation documentation |
| [`test_firmware/`](test_firmware/) | Historical diagnostic firmware, retained for reference and not release-qualified |

## Documentation

- [`docs/development.md`](docs/development.md) - development, programming, debugging, oscillator configuration, and runtime notes
- [`docs/gps.md`](docs/gps.md) - GPS hardware and integration notes
- [`docs/hc12.md`](docs/hc12.md) - HC-12, Si4463/Si4438, oscillator, and processor notes
- [`docs/telemetry-format.md`](docs/telemetry-format.md) - telemetry field order
- [`docs/status-word.md`](docs/status-word.md) - operational-status bit layout
- [`tests/README.md`](tests/README.md) - regression and structural target-test scope
- [`docs/release-checklist.md`](docs/release-checklist.md) - release process
- [`docs/hardware-validation.md`](docs/hardware-validation.md) - target/hardware validation record
- [`CHANGELOG.md`](CHANGELOG.md) - release history

## Project hardware

The repository retains photographs of the original project hardware. These are useful references for identifying the major modules, although current builders should verify the exact revision and components they source.

<table>
  <tr>
    <td align="center"><img src="images/readme/hc12-component.jpg" alt="HC-12 component side" width="300"><br><sub>HC-12 radio / STM8 module</sub></td>
    <td align="center"><img src="images/readme/gps-module.jpg" alt="u-blox GPS module" width="300"><br><sub>u-blox GPS module</sub></td>
    <td align="center"><img src="images/readme/battery-booster.jpg" alt="Battery boost converter" width="300"><br><sub>AAA battery boost converter</sub></td>
  </tr>
</table>

## Launch footage

[![PicoTracker high-altitude balloon launch](https://img.youtube.com/vi/OtdXHd_AjtY/hqdefault.jpg)](https://www.youtube.com/watch?v=OtdXHd_AjtY)

Launch footage from one of the PicoTracker high-altitude balloon flights. Click the preview to watch the video.

## Hardware design

### GPS

The original design uses a u-blox M8-based GPS module suitable for high-altitude balloon operation. Historical modules incorporated the receiver, TCXO, backup supply, external flash, LNA, SAW filtering, and a ceramic antenna. The original lightweight build concept removed the metal shield and ceramic antenna and replaced the antenna with a wire element and ground plane. Historical mass varied significantly with module and antenna configuration.

### 433 MHz radio and processor

The tracker reprograms the STM8S003F3 processor already present on HC-12 radio hardware and drives the Si4463 directly. The maintained firmware also retains the historical Si4438-compatible detection path. The STM8S003F3 has 8 KB of flash and 1 KB of RAM, making code size and bounded resource use important design constraints.

### Battery

The original battery supply is based on the earlier [BatteryAAA](https://github.com/ribbotson/rlabTelemetryTx/tree/master/Hardware/BatteryAAA) design, using a AAA lithium primary cell and a boost converter to supply approximately 3.3 V to the GPS, radio, and processor. Historical measurements were approximately 8 g for the cell and 3.5 g for the holder and boost converter.

### Tracker body

The tracker body can be 3D printed or made from lightweight foam. The original antenna arrangement placed the GPS antenna at the top and the 433 MHz radio antenna at the bottom. Mechanical files are retained in [`cad/`](cad/).

## Design constraints

PicoTracker is intentionally small and resource-constrained. The main engineering constraints are the 8 KB STM8 flash limit, low-temperature operation, oscillator stability, RF configuration, GPS availability at altitude, and energy consumption during acquisition/transmit/sleep cycles. The original modules and mechanical design were developed in 2018-2019, so a new physical build should be treated as a revalidation of the historical hardware rather than an assumption that every original component or measured property is unchanged.

## Release history

| Release | Date | Status |
| --- | --- | --- |
| [`v1.4.0`](https://github.com/ImperialSpaceSociety/PicoTracker/releases/tag/v1.4.0) | 2026-09-13 | Current maintained release |
| [`v1.3`](https://github.com/ImperialSpaceSociety/PicoTracker/releases/tag/v1.3) | 2019-03-07 | Historical, superseded by v1.4.0 |

## Contributing

Focused firmware fixes, regression tests, documentation improvements, hardware validation notes, and well-scoped technical contributions are welcome. See [`CONTRIBUTING.md`](CONTRIBUTING.md) before opening a pull request.

## License

PicoTracker uses separate licenses for software and hardware design material:

- Software and firmware: [MIT License](LICENSES/MIT.txt).
- Hardware design documentation: [CERN Open Hardware Licence v1.2](LICENSES/CERN-OHL-1.2.txt), or a later version where the existing project notice permits it.

See [`LICENSE.md`](LICENSE.md) for the repository license split and applicable notices.
