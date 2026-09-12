# Test firmware

This directory contains historical HC-12 test projects used during PicoTracker development.

- `HC12CW/` uses continuous transmission to check output frequency, frequency drift, and RF output power.
- `HC12MOD/` exercises RTTY modulation, baud rate, and timer behaviour.

These projects are separate from the main tracker firmware in `firmware/`.
For the active tracker firmware, see [`../firmware/`](../firmware/). Build and programming notes are in [`../docs/development.md`](../docs/development.md).

## Release status

These projects are preserved as historical diagnostic firmware and are **not release-qualified**. They contain older copies of radio and power-management code and are excluded from the PicoTracker `v1.4.0` firmware validation gates. Use `firmware/` for maintained production code.
