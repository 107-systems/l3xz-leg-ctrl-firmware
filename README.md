<a href="https://107-systems.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/107-systems.png" width="15%"></a>
:floppy_disk: `l3xz-leg-ctrl-firmware`
======================================
<a href="https://opencyphal.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/opencyphal.svg" width="25%"></a>
[![General Formatting Checks](https://github.com/107-systems/l3xz-leg-ctrl-firmware/workflows/General%20Formatting%20Checks/badge.svg)](https://github.com/107-systems/l3xz-leg-ctrl-firmware/actions?workflow=General+Formatting+Checks)
[![Spell Check](https://github.com/107-systems/l3xz-leg-ctrl-firmware/workflows/Spell%20Check/badge.svg)](https://github.com/107-systems/l3xz-leg-ctrl-firmware/actions?workflow=Spell+Check)
[![Compile Examples](https://github.com/107-systems/l3xz-leg-ctrl-firmware/workflows/Compile/badge.svg)](https://github.com/107-systems/l3xz-leg-ctrl-firmware/actions?workflow=Compile)

Firmware for the [L3X-Z](https://github.com/107-systems/l3xz) leg [controller](https://github.com/107-systems/l3xz-hw_leg-controller).

<p align="center">
  <a href="https://github.com/107-systems/l3xz"><img src="https://raw.githubusercontent.com/107-systems/.github/main/logo/l3xz-logo-memento-mori-github.png" width="40%"></a>
</p>

### How-to-build/upload
```bash
arduino-cli compile -b arduino:samd:nano_33_iot -v .
arduino-cli upload -b arduino:samd:nano_33_iot -v . -p /dev/ttyACM0
```
