# Current Build and BOM Starting Point

The PicoTracker repository preserves a working historical 2018-2019 hardware design. This guide provides a supplier-neutral starting point for new builders to select modern equivalents.

For historical reference, please see the existing files in the following directories:
* [`hardware/`](../hardware/)
* [`cad/`](../cad/)
* [`docs/`](../docs/)

**Important:** Compatibility claims below are marked with their current testing status. Do not assume compatibility for parts marked as *candidate* or *not yet tested* without independent verification.

## Required Functional Blocks

### 1. STM8 / HC-12 Radio
* **Required Characteristics:** 433 MHz band support, UART interface, STM8S-based architecture for firmware compatibility.
* **Historical Part:** Original HC-12 module (2018-2019 variant).
* **Current Candidates:**
  * Generic HC-12 Module (SI4463 based): *candidate* - [SI4463 Datasheet](https://www.silabs.com/documents/public/data-sheets/Si4464-63-61-60.pdf)
  * SV611 (Requires firmware modification): *not yet tested*

### 2. GPS
* **Required Characteristics:** NMEA 0183 protocol, 9600 baud rate (configurable), 3.3V logic level, active antenna support.
* **Historical Part:** ATGM336H or U-blox NEO-6M.
* **Current Candidates:**
  * ATGM336H-5N: *verified* - [ATGM336H Datasheet](http://www.icofchina.com/d/file/xiazai/2016-12-05/857e4e1a6cde9ad12dc79667087611a2.pdf)
  * U-blox NEO-8M: *candidate* - [NEO-8M Datasheet](https://content.u-blox.com/sites/default/files/NEO-M8-series-hardware-integration-manual_UBX-13003538.pdf)

### 3. Power Stage
* **Required Characteristics:** 3.7V LiPo battery input, regulated 3.3V output, low quiescent current (LDO).
* **Historical Part:** HT7333-A (LDO).
* **Current Candidates:**
  * HT7333-A: *verified* - [HT7333 Datasheet](https://www.holtek.com/documents/10179/116711/HT73xx-Av250.pdf)
  * ME6211: *candidate* - [ME6211 Datasheet](https://datasheet.lcsc.com/lcsc/1809141014_Microne-Nanjing-Micro-One-Elec-ME6211C33M5G-N_C82942.pdf)

### 4. Antennas
* **Required Characteristics:** 433 MHz tuned frequency, 50-ohm impedance, SMA or U.FL connector depending on the mounting choice.
* **Historical Part:** Standard 433 MHz spring antenna or dipole.
* **Current Candidates:**
  * 433 MHz Spring Antenna: *verified*
  * 433 MHz SMA Dipole Antenna: *candidate*

### 5. Programming / Debug Interface
* **Required Characteristics:** SWIM (Single Wire Interface Module) support for STM8.
* **Historical Part:** ST-Link V2 (Clone or original).
* **Current Candidates:**
  * ST-Link V2: *verified* - [ST-Link V2 User Manual](https://www.st.com/resource/en/user_manual/dm00026748-stlinkv2-in-circuit-debuggerprogrammer-for-stm8-and-stm32-stmicroelectronics.pdf)

### 6. Mechanical Mounting
* **Required Characteristics:** Lightweight, securely houses the PCB and battery, minimal RF interference.
* **Historical Part:** 3D printed enclosure (see `cad/` folder).
* **Current Candidates:**
  * Custom 3D printed enclosure (PETG or ABS recommended): *verified*

