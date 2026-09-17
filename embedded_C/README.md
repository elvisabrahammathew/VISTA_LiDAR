# VISTA LiDAR C++

This directory is the C++17 counterpart of the Rust project in `../embedded`.
The Rust source remains unchanged and can be kept as a behavioral reference.

## Architecture

- `1_Platform`: shared MessageBus, bounded broadcast rings, 64-bit topic
  WaitSets, worker threads, and native priority mapping.
- `2_Transport`: Ethernet, librealsense USB, messaging placeholders, and local storage.
- `3_Devices`: common LiDAR interfaces plus Quanergy M8 and RealSense L515 drivers.
- `4_Applications`: preprocessing, logging, and future application workers.
- `models`: sensor-neutral point clouds and LiDAR-specific topic envelopes.

`main.cpp` explicitly chooses and starts the independent Read, Decode,
Preprocessing, RAW Logger, and PCD Logger workers. Each worker registers its own
publisher/subscriber endpoints on the shared MessageBus. RAW and PCD logger
workers write to filenames generated from the local system start time. Capture
runs continuously until Ctrl+C, SIGTERM, or a worker failure requests shutdown.

`main.cpp` includes only `lib.hpp`, but its `main()` body explicitly creates
each selected worker in the order chosen by the user. The
`1_Platform/workers` module contains lifecycle helpers only: validation,
completion results, priority reporting, shutdown, and join/error handling.

Important source locations:

```text
1_Platform/
|-- message_bus/        Typed shared topic registry
|-- pubsub/             Shared rings, subscriber cursors, flags, and drop counters
|-- threading/          OS thread wrapper and priority configuration
`-- workers/            Reusable validation, result, shutdown, and join helpers
2_Transport/peripheral/
|-- ethernet/           Quanergy TCP transport
`-- librealsense_usb/   RealSense USB/SDK transport
3_Devices/lidars/
|-- lidar.cpp/.hpp      Common LiDAR interfaces and worker entry points
|-- quanergym8/         Quanergy M8 driver
|-- realsensel515/      Intel RealSense L515 driver
`-- unitree4d/          Reserved Unitree 4D driver
4_Applications/
|-- object/analytics/
|-- object/detection/
|-- object/tracking/
`-- processing/pointcloud/
```

`pubsub.hpp` and `message_bus.hpp` retain their template implementations in the
headers, while their non-template validation and lifecycle functions are kept
in the matching `.cpp` files.

Each topic stores one immutable `shared_ptr` per retained ring slot. A new
subscriber starts at the next message published, owns an independent cursor and
readiness bit, and never removes data needed by another subscriber. A slow
subscriber moves to the oldest retained slot after an overwrite and records the
number of messages it missed. Publication therefore does not block on a slow
consumer. Sequence values carry an epoch so wrapping the 64-bit value to zero
does not break cursor checks.

Every subscriber worker creates one `WorkerTopicInputs` object. It owns one
64-bit WaitSet and assigns bit 0, bit 1, and so on automatically as that worker
subscribes to additional input topics. Output publishers do not consume WaitSet
bits. In the current implementation each spawned worker maps one-to-one to a
dedicated `std::thread`; no thread pool is used.

## Windows / Visual Studio 2019

```powershell
cmake --preset windows-vs2019
cmake --build --preset windows-debug
ctest --preset windows-debug
./out/build/windows-vs2019/Debug/vista_capture.exe
```

When CMake is not on PATH, use the CMake executable bundled with Visual Studio.

## Ubuntu / Jetson

```bash
cmake --preset linux-gcc
cmake --build --preset linux-debug
ctest --preset linux-debug
./out/build/linux-gcc/vista_capture
```

Librealsense 2.50.0 is built directly from
`third_party/realsense/librealsense-2.50.0` as a static library. The same source
tree is used for Windows x64, Ubuntu x86-64, and Linux ARM64, so no prebuilt
`realsense2.dll` or `librealsense2.so` is required. Each platform generates its
own static library below its CMake build directory.

On Ubuntu/Jetson, install the SDK's native build requirements first. For the
default V4L2 backend this includes `libssl-dev`, `libusb-1.0-0-dev`,
`libudev-dev`, and `pkg-config`; the RealSense udev rules and Jetson kernel
requirements still apply. Set `VISTA_REALSENSE_FORCE_RSUSB_BACKEND=ON` only
when intentionally using librealsense's user-space USB backend.

See `third_party/README.md` for the expected dependency layout and version.

## Device selection

The program does not accept command-line configuration. Edit
`DeviceConfig.txt` before starting it:

```text
# Supported LiDAR: quanergy-m8, realsense-l515

Lidar: quanergy-m8
ReconnectIntervalSeconds(Lidar): 5

SensorIP(quanergym8): 192.168.1.3
TcpPort(quanergym8): 4141

UsbSerial(realsensel515):
DepthWidth(realsensel515): 640
DepthHeight(realsensel515): 480
DepthFps(realsensel515): 30

# Supported Radar: none

Radar: None
```

Use `Lidar: realsense-l515` for the L515. `UsbSerial` is optional: leave its
value blank to use the first matching L515, or enter the serial reported by
librealsense when selecting a specific USB device. The L515 does not use a
Windows COM port.

Each connection attempt and inactive read is limited to five seconds. When the
LiDAR is unavailable, `ReconnectIntervalSeconds(Lidar)` controls how long the
read worker waits before trying again. No extra connection check runs while
sensor data continues to arrive.

There is no duration setting: press Ctrl+C to stop the continuous capture and
let the workers close their log files cleanly.

At startup, the application creates both log names from local system time,
including seconds. For example, a run started at 2026-09-16 13:14:05 writes:

- `D:\MASTER\VISTA\LiDar\Repo\data\raw\20260916_131405.bin`
- `D:\MASTER\VISTA\LiDar\Repo\data\processed\20260916_131405.pcd`

On Linux, the same build rule places these folders under `data/raw` and
`data/processed` beside the repository's `embedded_C` directory.
