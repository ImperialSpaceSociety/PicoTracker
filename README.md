# Pico Balloon Tracker

A simple, low-cost pico-balloon tracker originally developed by the Imperial College Space Society.

> **Project status:** Active maintenance resumed in 2026. The original design dates from 2018–2019; current work focuses on preserving the hardware and firmware, improving documentation, and making incremental firmware fixes. Anyone building a unit today should revalidate component availability, prices, tooling, and test assumptions.

## Repository layout

- [`firmware/`](firmware/) - STM8 tracker firmware
- [`test_firmware/`](test_firmware/) - historical firmware test projects
- [`hardware/`](hardware/) - hardware design files
- [`cad/`](cad/) - mechanical/CAD files
- [`docs/`](docs/) - technical reference links
- [`docs/development.md`](docs/development.md) - firmware development, programming, and debugging notes
- [`docs/gps.md`](docs/gps.md) - GPS notes
- [`docs/hc12.md`](docs/hc12.md) - HC12 radio notes
- [`docs/new-tracker.md`](docs/new-tracker.md) - later tracker design notes
- [`tools/`](tools/) - supporting scripts

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

## Maintainer

This repository is maintained by [Sylvester Kaczmarek](https://SylvesterKaczmarek.com).

## License

Hardware is licensed under CERN OHL v1.2 or later. No warranty is provided for this documentation, implied or otherwise.

Software is licensed under the MIT License.

MIT License

Copyright (c) 2018 Imperial College Space Society

Derived Software Copyright (c) 2014 Richard Meadows <richardeoin>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
