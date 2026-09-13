# Getting started

PicoTracker is small enough to understand end to end, but broad enough to expose real embedded-systems work: power, STM8 firmware, GPS protocols, radio control, telemetry, decoding, testing, CAD, and flight validation.

You do not need tracker hardware to make a useful first contribution.

## First 10 minutes

Prerequisites are Git, Python 3, `make`, and a C compiler such as GCC or Clang.

```sh
git clone https://github.com/ImperialSpaceSociety/PicoTracker.git
cd PicoTracker
make demo
make test
```

`make demo` should report `165 valid frames`. If both commands complete successfully, you have already decoded real PicoTracker telemetry and exercised the firmware's host-testable logic.

To inspect the accepted frames themselves:

```sh
make decode DECODE_ARGS=--print-frames
```

No IAR installation, radio, GPS receiver, or STM8 board is required for this path. You can also run `make simulate` to exercise a complete synthetic tracker flight cycle and fault scenarios before touching hardware. For a guided first code-and-test exercise, continue with [`first-project.md`](first-project.md).

## Reproducible environment

If you do not want to install GCC or SDCC on your host, use the repository Dev Container or run `make container-check` with Docker. Both use the same container definition exercised by CI, including cppcheck and Ruff. See [`development-environment.md`](development-environment.md).

## Why this is a useful project to learn on

PicoTracker is a complete constrained system rather than an isolated library. A contribution can touch embedded C, UART and SPI, u-blox UBX packets, RF control, RTTY, CRCs, low-power operation, telemetry analysis, mechanical design, or verification. The STM8S003F3 provides only 8 KB of flash and 1 KB of RAM, so apparently small design decisions have visible consequences.

The project also contains historical flight hardware and captured telemetry, while the maintained firmware now has host regression tests and an independent STM8 structural build check. That makes it possible to learn from real engineering constraints without needing hardware on day one.

## Choose a path

| If you are interested in... | Start with | A useful first objective |
| --- | --- | --- |
| Embedded firmware | [`firmware/main.c`](../firmware/main.c), [`firmware/gps.c`](../firmware/gps.c) | Understand one acquisition/transmit/sleep cycle and add or improve a regression test |
| GPS and protocols | [`firmware/ubx_parser.h`](../firmware/ubx_parser.h), [`docs/gps.md`](gps.md) | Trace one UBX packet from request through checksum and fix validation |
| RF and telemetry | [`firmware/si_trx.c`](../firmware/si_trx.c), [`firmware/telemetry.c`](../firmware/telemetry.c) | Follow a telemetry byte from frame construction to RTTY channel switching |
| Python/data tooling | [`tools/decode_data.py`](../tools/decode_data.py) | Extend the decoder while keeping it dependency-light and tested |
| Testing and reliability | [`tests/`](../tests/) | Add a boundary, failure-path, or regression case that protects production firmware |
| Simulation and fault injection | [`tools/simulate_tracker.py`](../tools/simulate_tracker.py), [`simulator.md`](simulator.md) | Reproduce GPS loss, measurement faults, or telemetry loss without hardware |
| Hardware and RF | [`docs/hc12.md`](hc12.md), [`hardware/`](../hardware/) | Revalidate a component, interface, measurement, or current replacement part |
| Mechanical design | [`cad/`](../cad/) | Review or improve a lightweight mounting or enclosure part |
| Documentation | [`docs/`](./) | Turn a difficult subsystem into a short reproducible guide |

## Understand the system before changing it

The root [`README.md`](../README.md) contains the architecture and firmware-cycle diagrams. The most useful references after that are:

- [`telemetry-format.md`](telemetry-format.md) for the transmitted frame
- [`status-word.md`](status-word.md) for diagnostic telemetry
- [`development.md`](development.md) for STM8 programming and debugging
- [`tests/README.md`](../tests/README.md) for the regression suite and SDCC structural check
- [`roadmap.md`](roadmap.md) for meaningful extension directions

## Make a first contribution

Look for open issues labelled [`good first issue`](https://github.com/ImperialSpaceSociety/PicoTracker/issues?q=is%3Aissue%20is%3Aopen%20label%3A%22good%20first%20issue%22). Pick one narrow problem, make the smallest complete change, add or update tests where appropriate, and run:

```sh
make quality
make test
```

If your change affects target firmware structure and you have an STM8-capable SDCC installation, also run:

```sh
make stm8
```

A good first pull request does not need to redesign the tracker. A clear test, decoder improvement, documentation correction, or small firmware fix that makes the system easier to understand or trust is valuable.

## When hardware becomes useful

Hardware is needed for RF measurements, power profiling, environmental testing, programming the STM8, and validating physical changes. It is not required for most documentation, decoder, regression-test, protocol, or repository work.

If you want to build a tracker, treat the 2018-2019 hardware files as a validated historical design rather than a current bill of materials. Recheck component availability, RF configuration, power behaviour, and local radio/balloon requirements before flight.
