# Hardware-independent tracker simulator

`tools/simulate_tracker.py` models the maintained PicoTracker control and telemetry cycle without requiring STM8 hardware, GPS, radio equipment, or IAR.

It is intended for development, teaching, fault-injection experiments, and regression testing. It is **not** an RF propagation, battery-life, atmospheric, or balloon-dynamics simulator.

## What it models

The simulator mirrors the current firmware behaviour that can be represented deterministically on a host machine:

- bounded GPS acquisition with a maximum of 15 attempts
- degraded mode when no valid GPS fix is obtained
- retention of the last valid GPS position, altitude, satellite count, and UTC time during GPS loss
- fresh voltage and temperature measurements on each cycle
- measurement-failure status and zeroed failed measurements
- packed operational-status values
- one AWU sleep interval through 3000 m and two intervals above 3000 m
- the current 30.720 s AWU interval
- current telemetry formatting and XMODEM CRC
- radio-transmit failures as dropped telemetry frames

## Quick start

From the repository root:

```sh
make simulate
```

The default synthetic flight climbs from the ground through the 3000 m sleep-policy threshold to 30 km and then descends. Each line reports the ground-truth altitude, the altitude currently held by the tracker, GPS acquisition state, attempt count, packed status, sleep cadence, and radio-transmit result.

Use the built-in GPS-loss scenario to observe degraded operation and recovery:

```sh
make simulate SIM_ARGS="--scenario gps-loss --print-frames"
```

A mixed-fault scenario additionally injects radio-measurement and transmit failures:

```sh
make simulate SIM_ARGS="--scenario faults"
```

The simulator is deterministic, so the same inputs produce the same sequence and telemetry.

## Fault injection

Cycle numbers can be supplied as individual values or ranges:

```sh
make simulate SIM_ARGS="--gps-loss 6-8 --measurement-fail 10 --tx-fail 12,14"
```

Successful GPS acquisition can also be delayed to exercise retry-status reporting:

```sh
make simulate SIM_ARGS="--gps-attempts 4"
```

The simulator follows the current firmware's status semantics, including the fact that a successful fix after previous failed polls may still report a transient or retry-exhausted polling state for that cycle.

## Generate a synthetic capture

Write only successfully transmitted frames to a raw capture file:

```sh
make simulate SIM_ARGS="--scenario faults --capture /tmp/picotracker-sim.txt"
make decode CAPTURE=/tmp/picotracker-sim.txt
```

The generated capture uses the same sync prefix, field layout, and CRC convention expected by `tools/decode_data.py`.

## Fidelity and limitations

The synthetic altitude/position profile is deliberately simple and exists to exercise tracker behaviour, not to predict a real balloon trajectory. Simulated UTC advances by the configured AWU sleep duration; active GPS acquisition, telemetry transmission, oscillator switching, and other execution time are not included in that clock.

Key simulator constants are regression-tested against the maintained firmware headers and `energy.c`, and a known historical telemetry frame is used as a formatting/CRC reference vector. Simulated captures are also passed through the real telemetry decoder in the host test suite.

For target timing, RF behaviour, current consumption, GPS performance, and environmental behaviour, physical hardware validation remains authoritative.
