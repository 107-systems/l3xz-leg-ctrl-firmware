<a href="https://107-systems.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/107-systems.png" width="15%"></a>
:floppy_disk: Firmware for L3X-Z [leg controller](https://github.com/107-systems/l3xz-hw_leg-controller)
========================================================================================================
[![Check keywords.txt](https://github.com/107-systems/l3xz-fw_leg-controller/actions/workflows/check-keywords-txt.yml/badge.svg)](https://github.com/107-systems/l3xz-fw_leg-controller/actions/workflows/check-keywords-txt.yml)
[![General Formatting Checks](https://github.com/107-systems/l3xz-fw_leg-controller/workflows/General%20Formatting%20Checks/badge.svg)](https://github.com/107-systems/l3xz-fw_leg-controller/actions?workflow=General+Formatting+Checks)
[![Spell Check](https://github.com/107-systems/l3xz-fw_leg-controller/workflows/Spell%20Check/badge.svg)](https://github.com/107-systems/l3xz-fw_leg-controller/actions?workflow=Spell+Check)
[![Compile Examples](https://github.com/107-systems/l3xz-fw_leg-controller/workflows/Compile/badge.svg)](https://github.com/107-systems/l3xz-fw_leg-controller/actions?workflow=Compile)

### How-to-build/upload
```bash
arduino-cli compile -b arduino:samd:nano_33_iot -v leg-controller
arduino-cli upload -b arduino:samd:nano_33_iot -v leg-controller -p /dev/ttyACM0
```

## uavcan settings

specific seetings for the L3X-Z Hexapod can be found here: ???

### Node-ID

every leg controller needs to have its own Node-ID. The Node-ID is stored in the eeprom.

### Subject-ID

all leg controllers can use the same Subject-IDs. The host can differentiate between them by their Node-IDs.

| **Subject-ID** | **direction** | **name**          | **type**    |
|:--------------:|:-------------:|:-----------------:|:-----------:|
| heartbeat      | pub           | heartbeat         | heartbeat   |
| 1001           | pub           | input-voltage     | Real32      |
| 1002           | pub           | AS5048-A-angle    | Real32      |
| 1003           | pub           | AS5048-B-angle    | Real32      |
| 1004           | pub           | bumper            | Bit         |
| 1005           | sub           | LED1              | Bit         |
| 1010           | sub           | update_interval   | Integer16   |

## related repositories
* https://github.com/107-systems/107-Arduino-MCP2515
* https://github.com/107-systems/107-Arduino-AS504x
* https://github.com/107-systems/107-Arduino-UAVCAN
