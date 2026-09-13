# Project roadmap and extension ideas

PicoTracker `v1.4.0` is a maintained baseline, not an endpoint. The most useful future work is work that preserves the project's small, understandable character while improving reproducibility, observability, hardware accessibility, or flight confidence.

## Near-term contribution areas

### Open and reproducible tooling

- strengthen the SDCC/open-toolchain path beyond the current structural compile/link check
- improve automated size reporting and regression limits
- make telemetry capture, decoding, and analysis easier to reproduce on a clean machine

### Telemetry and ground tooling

- add structured export formats such as CSV or JSON to the decoder
- provide small visualization or analysis tools for captured flights
- improve diagnostic interpretation of the operational-status field

### Firmware assurance

- add targeted boundary and failure-path tests
- continue reducing assumptions around peripheral faults and recovery
- profile stack/RAM use and long-duration behaviour on target hardware

### Hardware refresh

- document a current, purchasable component set for a new build
- revalidate the power stage, oscillator options, RF path, GPS modules, and antennas
- preserve compatibility with historical hardware where that remains useful

### Power and environmental validation

- measure acquisition, transmit, and sleep currents on real hardware
- document cold-temperature behaviour and oscillator stability
- turn measurements into a simple energy budget for flight planning

### Mechanical design

- review the existing CAD for current modules and manufacturing methods
- reduce mass while retaining antenna separation and practical assembly
- document repeatable assembly and strain-relief choices

## Contribution scale

A useful contribution does not have to be large. Good starter work includes decoder features, documentation, regression tests, measurement scripts, and reproducible hardware notes. Intermediate work can cover protocol handling, low-power behaviour, build tooling, or CAD changes. Larger projects include a current hardware revision, a fully open target toolchain, or richer ground-station tooling.

The project should remain understandable enough that a new contributor can trace a telemetry value from sensor or GPS input through firmware, radio transmission, capture, and decoder output.
