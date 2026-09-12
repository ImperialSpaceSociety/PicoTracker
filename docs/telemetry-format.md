# Telemetry format

PicoTracker builds a comma-separated RTTY telemetry sentence from the latest GPS fix and radio measurements. The field order is defined by `prepare_tx_buffer()` in `firmware/telemetry.c`, with field sizes and offsets in `firmware/main.h`.

The transmitted data fields are:

1. payload name or callsign from `PAYLOAD_NAME`
2. incrementing sentence ID
3. UTC time as `HHMMSS`
4. signed latitude in decimal degrees
5. signed longitude in decimal degrees
6. altitude in metres
7. number of satellites used in the solution
8. radio supply voltage in millivolts
9. packed operational-status value
10. signed radio temperature
11. XMODEM CRC checksum

Latitude and longitude are derived from the u-blox integer coordinates scaled by `10^7`. The firmware inserts the decimal point when constructing the telemetry sentence.

The operational-status field is formatted as a four-digit decimal value. See [`status-word.md`](status-word.md) for the packed diagnostic bit layout and status meanings.
