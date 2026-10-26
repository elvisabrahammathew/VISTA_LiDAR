# ADLINK native build: Ubuntu 20.04 ARM64

Updated 2026-10-21. The active ADLINK target is Ubuntu 20.04 ARM64 (aarch64),
not Ubuntu 18.04 / Bionic. Windows x64 and Linux x86-64 remain supported.
Historical Ubuntu 18.04 validation reports are retained as evidence only.

## Scope and prerequisites

- Use a vendor-supported OS image/BSP for your ADLINK device. These instructions
  neither upgrade the OS nor establish that a particular image supports its hardware.
- Build natively on ARM64 using a C++17 compiler (for example GCC 9) and
  CMake 3.16 or newer. Presets require CMake 3.20 or newer; the script below
  deliberately uses traditional commands, not presets.
- Initialize the repository's pinned Git submodules. The core camera-free
  build does not require ROS, Python, CUDA or an installed Grafana server.
- RealSense is disabled in this script. This does not disable Unitree or Quanergy.

On the device, install missing tools through its configured package repositories:

```bash
sudo apt update
sudo apt install build-essential cmake git
uname -m
g++ --version
cmake --version
```

`uname -m` must report `aarch64`. OS installation and device drivers are separate
from this build procedure. Do not replace system libraries to run a binary built
on a newer Ubuntu; use a matching native build or target sysroot instead.

## Build and test

From the repository root (the directory containing `embedded_C`):

```bash
git submodule update --init --recursive
bash embedded_C/scripts/build_adlink.sh
```

The script configures Release, tests ON, RealSense OFF, ARM64 architecture guard,
then builds with two jobs and runs the unit suite. A configure, build or test
failure stops the script. It does not start `vista_edge` or open a sensor port.

The new default build directory is:

```text
embedded_C/out/build/linux-adlink-ubuntu20-arm64
```

It is separate from the retired Bionic build cache. Do not point `VISTA_BUILD_DIR`
at a cache from the old OS/toolchain. Existing captures, maps, build folders and
WSL environments are not deleted or migrated automatically.

Optional overrides, applied to a fresh build directory:

```bash
VISTA_BUILD_JOBS=1 bash embedded_C/scripts/build_adlink.sh
VISTA_BUILD_DIR="$HOME/vista-build-ubuntu20-arm64" \
  bash embedded_C/scripts/build_adlink.sh
```

`CC` and `CXX` can select a compiler when configuring a fresh directory.
The shared filesystem wrapper remains unchanged. Standard filesystem is selected
when its header/link probe succeeds; the optional portable backend is not a
separate OS build target.

## Configure and run

```bash
cd embedded_C/out/build/linux-adlink-ubuntu20-arm64
# On first launch, a missing DeviceConfig.txt is created and the app exits.
# Review the generated COM/IP settings, then run this command again:
./vista_edge
```

- The executable is `vista_edge`, not a Windows `.exe`. Keep `DeviceConfig.txt`
  and optional `GrafanaSecret.txt` beside it. They are resolved relative to the
  executable, so launching from another working directory is supported.
- Missing sidecars are created automatically: device settings are embedded from
  `config/DeviceConfig.defaults.txt` at build time; `GrafanaToken:` is left blank.
  Existing files are never overwritten. After creating a device config the app
  exits before starting hardware: review the COM/IP settings, then run it again.
  The executable directory needs write permission only when creating a file.
- Builds never copy or overwrite runtime `DeviceConfig.txt` or
  `GrafanaSecret.txt`. Edit the files beside the executable for deployment.
  The defaults template is for building defaults only, not device configuration.
- Use the actual Linux Unitree serial device, such as `/dev/ttyUSB0` or
  `/dev/ttyACM0`, not `COM3`. Grant the runtime user serial-device access according
  to your device permissions (commonly the `dialout` group).
- Grafana and its plugins can stay on the Windows PC. With Windows at
  `192.168.10.1` and ADLINK at `192.168.10.2`, the Grafana HTTP destination is
  Windows; RoomMap/Gaussian WebSocket panel URLs must point to ADLINK's LAN
  address. Bind listeners to the intended LAN interface and restrict firewall
  access. These listeners are not authenticated public services.
- Keep `GrafanaSecret.txt`, if used, local and out of Git. No dashboard/plugin
  update or application configuration change is required just for this build change.

## Validation boundary

Retiring the old build target does not change the LiDAR reader/decoder, IMU
synchronization, Point-LIO, RoomMap persistence, Gaussian worker or their runtime
settings. The processing optimizations already in the source tree are retained.

Windows/native Linux tests and modern ARM64 cross-build/QEMU tests are regression
checks, not proof of Ubuntu 20.04 ABI compatibility or physical ADLINK behavior.
Final acceptance requires a native build/test on the upgraded device, then live
serial I/O, timing/input-drop checks, map quality and sustained operation.
The build script never performs an OS upgrade or a LiDAR restart.
