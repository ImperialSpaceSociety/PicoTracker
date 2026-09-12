# Hardware validation matrix

This matrix is the final release gate for PicoTracker firmware. Host tests and CI do not replace these target checks.

**Current status:** target validation pending. No IAR STM8 compiler, ST-LINK, or PicoTracker target hardware was available in the connected maintenance environment when this matrix was created.

## Build configurations

| Configuration | Intended use | Status |
| --- | --- | --- |
| HC-12 with 32 MHz TCXO | Primary maintained configuration | Pending |
| HC-12 with original 30 MHz crystal | Compatibility configuration | Pending |
| QFN radio/select-pin path | Automatic radio detection path | Pending |
| TSSOP radio/select-pin path | Automatic fallback path | Pending |

## Required tests

| Test | Acceptance criterion | Status |
| --- | --- | --- |
| IAR Debug build | Clean build, zero errors | Pending |
| IAR Release build | Clean build, zero errors | Pending |
| Flash/program | ST-LINK programs and verifies target | Pending |
| Cold boot | Tracker starts reliably after full power removal | Pending |
| Radio identification | Supported Si4463/Si4438 device is detected on the correct select pin | Pending |
| GPS configuration | Required UBX configuration commands are acknowledged | Pending |
| Valid GPS fix | Valid 3D `gnssFixOK` solution is accepted | Pending |
| GPS degraded mode | Loss of GPS does not block subsequent telemetry indefinitely | Pending |
| RTTY transmit | Stable 433 MHz RTTY output at configured baud and frequency | Pending |
| Pips mode | Startup/pip transmissions complete without deadlock | Pending |
| Radio failure path | CTS/radio failure exits through bounded error handling | Pending |
| Telemetry CRC | Captured frame passes decoder CRC validation | Pending |
| Telemetry fields | Time, position, altitude, status, voltage, and temperature decode plausibly | Pending |
| Radio measurements | Voltage and temperature readings are plausible across normal range | Pending |
| Sleep cadence | One AWU interval through 3000 m policy boundary, two above 3000 m | Pending |
| Clock recovery | MCU returns to HSI after active halt and continues normal operation | Pending |
| Power consumption | Sleep and active currents are measured and recorded | Pending |
| Extended run | Repeated acquisition/transmit/sleep cycles complete without lockup | Pending |
| Low-temperature test | Tracker remains operational at the selected environmental test point | Pending |

## Validation record

For each release, record the date, tester, hardware revision, radio package, oscillator, GPS module, battery/power source, IAR version, firmware commit SHA, observed RF frequency, firmware size, and links to captured telemetry or test notes.
