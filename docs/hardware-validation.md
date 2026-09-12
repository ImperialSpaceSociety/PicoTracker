# Hardware validation matrix

This matrix records the target-validation status used for PicoTracker releases. Host tests and CI complement, but do not replace, target checks.

**v1.4.0 release status (2026-09-13):** approved by the maintainer following target/hardware validation. Detailed IAR version/build-size output, RF measurements, power measurements, environmental measurements, and binary artifacts were not archived with this release. The repository independently records passing host regression tests plus the SDCC STM8 structural compile/link check.

## Build configurations

| Configuration | Intended use | Status |
| --- | --- | --- |
| HC-12 with 32 MHz TCXO | Primary maintained configuration | Maintainer validated |
| HC-12 with original 30 MHz crystal | Compatibility configuration | Maintainer validated |
| QFN radio/select-pin path | Automatic radio detection path | Maintainer validated |
| TSSOP radio/select-pin path | Automatic fallback path | Maintainer validated |

## Required tests

| Test | Acceptance criterion | Status |
| --- | --- | --- |
| IAR Debug build | Clean build, zero errors | Maintainer validated |
| IAR Release build | Clean build, zero errors | Maintainer validated |
| Flash/program | ST-LINK programs and verifies target | Maintainer validated |
| Cold boot | Tracker starts reliably after full power removal | Maintainer validated |
| Radio identification | Supported Si4463/Si4438 device is detected on the correct select pin | Maintainer validated |
| GPS configuration | Required UBX configuration commands are acknowledged | Maintainer validated |
| Valid GPS fix | Valid 3D `gnssFixOK` solution is accepted | Maintainer validated |
| GPS degraded mode | Loss of GPS does not block subsequent telemetry indefinitely | Maintainer validated |
| RTTY transmit | Stable 433 MHz RTTY output at configured baud and frequency | Maintainer validated |
| Pips mode | Startup/pip transmissions complete without deadlock | Maintainer validated |
| Radio failure path | CTS/radio failure exits through bounded error handling | Maintainer validated |
| Telemetry CRC | Captured frame passes decoder CRC validation | Maintainer validated |
| Telemetry fields | Time, position, altitude, status, voltage, and temperature decode plausibly | Maintainer validated |
| Radio measurements | Voltage and temperature readings are plausible across normal range | Maintainer validated |
| Sleep cadence | One AWU interval through 3000 m policy boundary, two above 3000 m | Maintainer validated |
| Clock recovery | MCU returns to HSI after active halt and continues normal operation | Maintainer validated |
| Power consumption | Sleep and active currents are measured and recorded | Maintainer validated |
| Extended run | Repeated acquisition/transmit/sleep cycles complete without lockup | Maintainer validated |
| Low-temperature test | Tracker remains operational at the selected environmental test point | Maintainer validated |

## Validation record

For future releases, record the date, tester, hardware revision, radio package, oscillator, GPS module, battery/power source, IAR version, firmware commit SHA, observed RF frequency, firmware size, and links to captured telemetry or test notes.

For `v1.4.0`, the maintainer supplied the release authorization and reported successful target/hardware operation. The detailed quantitative record above was not retained, so the status is recorded as maintainer-validated rather than reconstructed from unavailable measurements.
