# Operational status field

The telemetry `op_status` value is a packed diagnostic field assembled in `firmware/main.c` and stored as a 16-bit value in `struct gps_fix`.

The currently used low byte is arranged as follows:

| Bits | Meaning |
| --- | --- |
| 7..4 | GPS fix poll attempts, saturated at 15 |
| 3..2 | GPS configuration status |
| 1..0 | GPS polling status |

For the two status subfields, value `0` means no recorded failure, `1` records a transient failure, and `2` records retry exhaustion. For the GPS polling field, value `3` means fix acquisition reached its overall attempt limit and the tracker continued in degraded mode.

The attempt counter is also the acquisition budget: a fix cycle performs at most 15 NAV-PVT polls. If no valid 3D solution is obtained, telemetry continues with polling status `3` instead of blocking indefinitely.
