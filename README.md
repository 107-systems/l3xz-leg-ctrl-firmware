<a href="https://107-systems.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/107-systems.png" width="15%"></a>
:floppy_disk: Firmware for L3X-Z [leg controller](https://github.com/107-systems/l3xz-hw_leg-controller)
========================================================================================================
[![General Formatting Checks](https://github.com/107-systems/l3xz-fw_leg-controller/workflows/General%20Formatting%20Checks/badge.svg)](https://github.com/107-systems/l3xz-fw_leg-controller/actions?workflow=General+Formatting+Checks)
[![Spell Check](https://github.com/107-systems/l3xz-fw_leg-controller/workflows/Spell%20Check/badge.svg)](https://github.com/107-systems/l3xz-fw_leg-controller/actions?workflow=Spell+Check)
[![Compile Examples](https://github.com/107-systems/l3xz-fw_leg-controller/workflows/Compile/badge.svg)](https://github.com/107-systems/l3xz-fw_leg-controller/actions?workflow=Compile)

<p align="center">
  <a href="https://github.com/107-systems/l3xz"><img src="https://raw.githubusercontent.com/107-systems/.github/main/logo/l3xz-logo-memento-mori-github.png" width="40%"></a>
</p>

### How-to-build/upload
```bash
arduino-cli compile -b arduino:samd:nano_33_iot -v .
arduino-cli upload -b arduino:samd:nano_33_iot -v . -p /dev/ttyACM0
```

## OpenCyphal Settings

Specific settings for the L3X-Z Hexapod can be found here: https://github.com/107-systems/l3xz-hw#node-ids .

### Node-ID

Every leg controller needs to have its own Node-ID. The Node-ID is stored in the eeprom. If no eeprom is found, the Node-ID is 101.

### Subject-ID

Some Subject-IDs are the same as with the leg controller. The host can differentiate between them by their Node-IDs.

| **Subject-ID** | **Direction** | **Name**          | **Type**    |
|:--------------:|:-------------:|:-----------------:|:-----------:|
| heartbeat      | pub           | heartbeat         | heartbeat   |
| 1001           | pub           | input-voltage     | Real32      |
| 1002           | pub           | AS5048-A-angle    | Real32      |
| 1003           | pub           | AS5048-B-angle    | Real32      |
| 1004           | pub           | bumper            | Bit         |
| 1005           | sub           | LED1              | Bit         |
| 1010           | sub           | update_interval   | Integer16   |

## Related Repositories
* [107-Arduino-MCP2515](https://github.com/107-systems/107-Arduino-MCP2515)
* [107-Arduino-AS504x](https://github.com/107-systems/107-Arduino-AS504x)
* [107-Arduino-UAVCAN](https://github.com/107-systems/107-Arduino-UAVCAN)
