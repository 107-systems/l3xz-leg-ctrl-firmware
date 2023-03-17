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
arduino-cli compile -b rp2040:rp2040:arduino_nano_connect -v .
arduino-cli upload -b rp2040:rp2040:arduino_nano_connect -v .. -p /dev/ttyACM0
```
**or**
```bash
arduino-cli compile -b rp2040:rp2040:rpipico -v . --build-property compiler.cpp.extra_flags="-DCYPHAL_NODE_INFO_GIT_VERSION=0x$(git rev-parse --short=16 HEAD)"
```
Adding argument `--build-property compiler.cpp.extra_flags="-DCYPHAL_NODE_INFO_GIT_VERSION=0x$(git rev-parse --short=16 HEAD)"` allows to feed the Git hash of the current software version to [107-Arduino-Cyphal](https://github.com/107-systems/107-Arduino-Cyphal) stack from where it can be retrieved via i.e. [yakut](https://github.com/opencyphal/yakut).

### How-to-`yakut`
Configure `can0`:
```bash
sudo ./setup_slcan.sh --remove-all --basename can --speed-code 5 /dev/serial/by-id/usb-Zubax_Robotics_Zubax_Babel_*-if00
```
Configure `yakut`:
```bash
python3 -m pip install yakut
yakut compile https://github.com/OpenCyphal/public_regulated_data_types/archive/refs/heads/master.zip
. setup_yakut.sh
```
