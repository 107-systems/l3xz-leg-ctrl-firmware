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
arduino-cli compile -b rp2040:rp2040:arduino_nano_connect -v . --build-property compiler.cpp.extra_flags="-DCYPHAL_NODE_INFO_GIT_VERSION=0x$(git rev-parse --short=16 HEAD)"
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
Subscribe via `yakut`:
```bash
yakut sub 1001:uavcan.si.unit.angle.Scalar.1.0 --with-metadata
...
---
1001:
  _meta_: {ts_system: 1679050084.745943, ts_monotonic: 91027.725109, source_node_id: 6, transfer_id: 18, priority: nominal, dtype: uavcan.si.unit.angle.Scalar.1.0}
  radian: 3.814626693725586
---
1001:
  _meta_: {ts_system: 1679050084.797002, ts_monotonic: 91027.773986, source_node_id: 6, transfer_id: 19, priority: nominal, dtype: uavcan.si.unit.angle.Scalar.1.0}
  radian: 3.814626693725586
---
...
```
or
```bash
yakut sub 1003:uavcan.primitive.scalar.Bit.1.0 --with-metadata
...
---
1003:
  _meta_: {ts_system: 1681729517.634219, ts_monotonic: 24298.607404, source_node_id: 31, transfer_id: 10, priority: nominal, dtype: uavcan.primitive.scalar.Bit.1.0}
  value: false
```
