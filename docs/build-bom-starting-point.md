# Current Build and BOM Starting Point

The PicoTracker repository preserves a working historical 2018-2019 hardware design. This guide provides a supplier-neutral starting point for new builders to select modern equivalents.

For historical reference, please see the existing files in the following directories:
* [`hardware/`](../hardware/)
* [`cad/`](../cad/)
* [`docs/`](../docs/)

**Important:** Compatibility claims below are marked with their current testing status. Do not assume compatibility for parts marked as *candidate* or *not yet tested* without independent verification.

## Required Functional Blocks

### 1. STM8 / HC-12 Radio
* **Required Characteristics:** 433 MHz operation with an STM8S003F3 and a compatible Si4463/Si4438-family radio path. PicoTracker reprograms the STM8S003F3 and controls the radio directly; UART is associated with the original HC-12 module firmware and is not the primary interface used by the maintained PicoTracker firmware.
* **Historical Part:** HC-12 hardware based on STM8S003F3 and Si4463-compatible radio hardware.
* **Current Candidates:**
  * HC-12-compatible STM8S003F3 / Si4463 hardware: *candidate* - [Si4463 Datasheet](https://www.silabs.com/documents/public/data-sheets/Si4464-63-61-60.pdf)
  * Si4438-compatible hardware path: *candidate*

### 2. GPS
* **Required Characteristics:** u-blox M8-compatible GNSS receiver with support for the UBX protocol used by PicoTracker, including UBX-NAV-PVT, and compatible 3.3 V logic.
* **Historical Part:** M8-based modules such as BN-280 and BN-220.
* **Current Candidates:**
  * BN-280 / BN-220 or equivalent u-blox M8-based module: *candidate*
  * Other u-blox M8-compatible GNSS modules supporting UBX-NAV-PVT: *candidate* - [NEO-M8 Hardware Integration Manual](https://content.u-blox.com/sites/default/files/NEO-M8-series-hardware-integration-manual_UBX-13003538.pdf)

### 3. Power Stage
* **Required Characteristics:** AAA lithium primary-cell input with a low-power boost converter providing approximately 3.3 V for the tracker electronics.
* **Historical Part:** AAA lithium primary cell with a boost-converter power stage as represented by the repository's historical hardware design.
* **Current Candidates:**
  * Equivalent low-quiescent-current boost converter suitable for a single AAA lithium primary cell: *candidate*

### 4. Antennas
* **Required Characteristics:** Antenna suitable for the PicoTracker 433 MHz radio path, with impedance and connection method matched to the selected radio hardware and PCB implementation.
* **Historical Part:** Refer to the repository's historical hardware and CAD files for the original RF implementation.
* **Current Candidates:**
  * 433 MHz antenna compatible with the selected radio implementation: *candidate*
  * Alternative 433 MHz antenna configurations: *not yet tested*

### 5. Programming / Debug Interface
* **Required Characteristics:** SWIM (Single Wire Interface Module) support for programming the STM8S003F3.
* **Historical Part:** ST-Link-compatible STM8 programming/debug interface.
* **Current Candidates:**
  * ST-Link V2 or compatible programmer with STM8 SWIM support: *candidate* - [ST-Link V2 User Manual](https://www.st.com/resource/en/user_manual/dm00026748-stlinkv2-in-circuit-debuggerprogrammer-for-stm8-and-stm32-stmicroelectronics.pdf)

### 6. Mechanical Mounting
* **Required Characteristics:** Lightweight mounting that securely houses the PCB and battery while avoiding unnecessary interference with the RF implementation.
* **Historical Part:** Refer to the existing designs in the [`cad/`](../cad/) directory.
* **Current Candidates:**
  * Enclosure or mounting solution based on the existing CAD design: *candidate*
  * Alternative lightweight enclosure materials and designs: *not yet tested*
