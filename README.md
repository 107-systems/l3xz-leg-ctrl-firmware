[![General Formatting Checks](https://github.com/107-systems/l3xz-fw_leg-controller/workflows/General%20Formatting%20Checks/badge.svg)](https://github.com/wuehr1999/NavigationOnArduino/actions?workflow=General+Formatting+Checks)

# Firmware for the L3X-Z Hexapod leg controller

https://github.com/107-systems/l3xz-hw_leg-controller

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
