# Host tests

These tests exercise hardware-independent firmware logic on a desktop C compiler. Current coverage includes numeric telemetry formatting, u-blox UBX checksums and fix validity, radio temperature conversion, and altitude-dependent sleep policy.

Run all host tests from this directory with:

```sh
make test
```

The host tests complement the STM8/IAR build and hardware validation; they do not replace target testing.
