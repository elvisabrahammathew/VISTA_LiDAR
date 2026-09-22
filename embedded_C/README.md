# VISTA Edge C++

This directory is the C++17 counterpart of the Rust project in `../embedded`.
The Rust source remains unchanged and can be kept as a behavioral reference.

## Architecture

- `1_Platform`: shared MessageBus, bounded broadcast rings, 64-bit topic
  WaitSets, worker threads, and native priority mapping.
- `2_Transport`: TCP, UDP, serial, librealsense USB, HTTP, and local storage.
- `3_Devices`: common LiDAR interfaces plus Quanergy M8, RealSense L515, and
  Unitree L2 drivers.
- `4_Applications`: preprocessing, logging, and future application workers.
- `models`: sensor-neutral point clouds and LiDAR-specific topic envelopes.

`main.cpp` explicitly chooses and starts the independent Read, Decode,
Preprocessing, RAW Logger, PCD Logger, System Monitor, Grafana Bridge, and
Point-cloud WebSocket workers. Each worker registers its own
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
|-- librealsense_usb/   RealSense USB/SDK transport
|-- serial/             Cross-platform USB serial transport
`-- udp/                Cross-platform sensor UDP transport
2_Transport/messaging/
|-- http.cpp/.hpp       Cross-platform HTTP/1.1 client used by Grafana Live
`-- websocket_server.*  Cross-platform browser WebSocket server
3_Devices/lidars/
|-- lidar.cpp/.hpp      Common LiDAR interfaces and worker entry points
|-- quanergym8/         Quanergy M8 driver
|-- realsensel515/      Intel RealSense L515 driver
`-- unitree4d/          Unitree L2 transport selection and protocol decoder
4_Applications/
|-- grafana_bridge/     Multi-topic Grafana Live output worker
|-- monitoring/         OS, worker, storage, power, and thermal telemetry
|-- pointcloud_websocket/ Dedicated LPC1 3D point-cloud output worker
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

Initialize the pinned librealsense submodule once after cloning the repository:

```powershell
git submodule update --init --recursive
```

```powershell
cmake --preset windows-vs2019
cmake --build --preset windows-debug
ctest --preset windows-debug
./out/build/windows-vs2019/Debug/vista_edge.exe
```

When CMake is not on PATH, use the CMake executable bundled with Visual Studio.

## Ubuntu x86-64

```bash
git submodule update --init --recursive
sudo apt install build-essential cmake pkg-config libusb-1.0-0-dev libudev-dev
cmake --preset linux-x86_64-gcc
cmake --build --preset linux-x86_64-debug
ctest --preset linux-x86_64-debug
./out/build/linux-x86_64-gcc/vista_edge
```

## Linux ARM64 / Jetson

Run the ARM64 preset natively on the Jetson/ADLINK device:

```bash
git submodule update --init --recursive
sudo apt install build-essential cmake pkg-config libusb-1.0-0-dev libudev-dev
cmake --preset linux-arm64-gcc
cmake --build --preset linux-arm64-debug
ctest --preset linux-arm64-debug
./out/build/linux-arm64-gcc/vista_edge
```

The ARM64 preset enables librealsense's RSUSB backend by default. This avoids a
compile-time dependency on patched Jetson UVC kernel drivers. The RealSense
udev rules and USB device permissions are still required. The x86-64 preset
uses the native Linux V4L2 backend by default. Both choices can be overridden
with `VISTA_REALSENSE_FORCE_RSUSB_BACKEND` when configuring a custom build.

Librealsense 2.50.0 is pinned by the repository-level Git submodule at
`../third_party/librealsense` and built as a static library. The same source commit
is used for Windows x64, Ubuntu x86-64, and Linux ARM64, so no prebuilt
`realsense2.dll` or `librealsense2.so` is required. Each platform generates its
own static library below its CMake build directory.

VISTA-owned sources are compiled once through the `vista_edge_core` OBJECT
target and linked directly into `vista_edge` and `vista_edge_tests`. No separate
VISTA-owned static library is generated.

These presets are native builds. Cross-compiling Linux ARM64 from Windows or
x86-64 Linux additionally requires an ARM64 sysroot and CMake toolchain file;
that is intentionally separate from the native Jetson preset.

See `../third_party/README.md` for the expected dependency layout and version.

## Device selection

The program does not accept command-line configuration. Edit
`DeviceConfig.txt` before starting it:

```text
# Supported LiDAR: quanergy-m8, realsense-l515, unitree-l2
Lidar: quanergy-m8
RawLoggingEnabled(Lidar): 0
PointCloudLoggingEnabled(Lidar): 0
ReconnectIntervalSeconds(Lidar): 5
SensorIP(quanergy-m8, unitree-l2): 192.168.1.3
SensorPort(quanergy-m8, unitree-l2): 4141
ConnectionMode(unitree-l2): auto
LocalIP(unitree-l2): 192.168.1.2
LocalPort(unitree-l2): 6201
SerialPort(unitree-l2): COM3
BaudRate(unitree-l2): 4000000
UsbSerial(realsensel515): 123456789012
DepthWidth(realsensel515): 640
DepthHeight(realsensel515): 480
DepthFps(realsensel515): 30

# Supported Radar: none
Radar: None

# Grafana Live output
GrafanaEnabled: true
GrafanaHost: 127.0.0.1
GrafanaPort: 3000
GrafanaNamespace: vista
GrafanaPublishIntervalMilliseconds: 1000
GrafanaRetryIntervalSeconds: 5
SystemMonitorIntervalMilliseconds: 2000

# 3D point-cloud WebSocket for the lidarpointcloud Grafana panel
PointCloudWebSocketEnabled: true
PointCloudWebSocketBindAddress: 127.0.0.1
PointCloudWebSocketPort: 8765
PointCloudWebSocketMaxPoints: 100000
PointCloudWebSocketMaxClients: 4
PointCloudWebSocketPublishIntervalMilliseconds: 100
PointCloudWebSocketRetryIntervalSeconds: 2
```

Use `Lidar: realsense-l515` for the L515. `UsbSerial` is optional: leave its
value blank to use the first matching L515, or enter the serial reported by
librealsense when selecting a specific USB device. The L515 does not use a
Windows COM port.

For the Unitree L2, select `Lidar: unitree-l2`, change the shared `SensorIP`
and `SensorPort` to the L2 UDP endpoint (normally `192.168.1.62:6101`), and
configure the PC endpoint with `LocalIP` and `LocalPort`. `ConnectionMode` can
be `serial`, `udp`, or `auto`. Auto mode probes the configured serial port and
then UDP; the first transport that delivers a complete frame with valid CRC is
kept. On Linux, change `SerialPort` from the Windows example `COM3` to a path
such as `/dev/ttyACM0`.

The Unitree wire protocol is decoded by VISTA-owned portable code rather than
linking the Linux-only `libunilidar_sdk2.a`. The pinned
`../third_party/unilidar_sdk2` submodule remains in the repository as the
official protocol and behavior reference. The same decoder is compiled on
Windows x64, Linux x86-64, and Linux ARM64.

Each connection attempt and inactive read is limited to five seconds. When the
LiDAR is unavailable, `ReconnectIntervalSeconds(Lidar)` controls how long the
read worker waits before trying again. No extra connection check runs while
sensor data continues to arrive.

There is no duration setting: press Ctrl+C to stop the continuous capture and
let the workers close their log files cleanly.

`RawLoggingEnabled(Lidar)` and `PointCloudLoggingEnabled(Lidar)` are independent.
Both default to `0`, so no logger worker starts and no output file is created.
Set either value to `1` to enable only that output.

For enabled outputs, the application creates log names from local system time,
including seconds. For example, a run started at 2026-09-16 13:14:05 can write:

- `D:\MASTER\VISTA\LiDar\Repo\data\raw\20260916_131405.bin`
- `D:\MASTER\VISTA\LiDar\Repo\data\processed\20260916_131405.pcd`

On Linux, the same build rule places these folders under `data/raw` and
`data/processed` beside the repository's `embedded_C` directory.

## Grafana Live

Grafana authentication uses a service-account token from the environment
variable `VISTA_GRAFANA_TOKEN`. The secret is intentionally not stored in
`DeviceConfig.txt` or printed to the console. Create an Editor service account
in Grafana, generate a token, and set it before starting the application.

For Visual Studio without PowerShell or Command Prompt, copy
`GrafanaSecret.example.txt` to `GrafanaSecret.txt`, then edit the local file:

```text
GrafanaToken: glsa_your_generated_token
```

`GrafanaSecret.txt` is ignored by Git and copied beside the executable during
the build. The local secret file takes precedence when both methods are
configured. The value must be the generated secret beginning with `glsa_`, not
the token name displayed in Grafana's token table.

For the current PowerShell session:

```powershell
$env:VISTA_GRAFANA_TOKEN = "glsa_your_token_here"
./out/build/windows-vs2019/Debug/vista_edge.exe
```

To make the token available to Visual Studio, save it as a Windows user
environment variable and restart Visual Studio so the debugger inherits it:

```powershell
[Environment]::SetEnvironmentVariable(
    "VISTA_GRAFANA_TOKEN",
    "glsa_your_token_here",
    "User")
```

On Linux:

```bash
export VISTA_GRAFANA_TOKEN="glsa_your_token_here"
./out/build/linux-x86_64-gcc/vista_edge
```

Never commit the real token to Git. If it is missing, the bridge keeps running
and sends unauthenticated requests so an intentionally anonymous local Grafana
setup remains supported. HTTP 401 errors explicitly identify the environment
variable or local secret file that must be corrected.

When enabled, `grafana-bridge` owns subscriptions to both the decoded and the
processed point-cloud topics. It calculates point counts, processing rate,
latency, queue drops, and XYZ bounds, then pushes one Influx line-protocol
measurement per configured interval to:

```text
http://<GrafanaHost>:<GrafanaPort>/api/live/push/<GrafanaNamespace>
```

The bridge currently sends `pointcloud`, `pipeline_health`, `system_health`,
`worker_health`, `storage_health`, `power_thermal`, and
`grafana_bridge_health`. Future radar, detection, tracking, and analytics
measurements can be added to the same worker without changing `main.cpp`.
A Grafana outage does not stop LiDAR capture: the
worker reports the first failure, keeps draining its topic queues, and retries
using `GrafanaRetryIntervalSeconds`.

The full XYZ/intensity cloud is intentionally sent through the separate
`pointcloud-websocket` worker instead of the HTTP metrics bridge. It subscribes
to `pointcloud/processed`, samples oversized frames to
`PointCloudWebSocketMaxPoints`, combines packetized spinning-LiDAR messages
over `PointCloudWebSocketPublishIntervalMilliseconds`, encodes the
little-endian `LPC1` format, and broadcasts it to the `lidarpointcloud` panel
at:

```text
ws://<PointCloudWebSocketBindAddress>:<PointCloudWebSocketPort>
```

With the default `127.0.0.1` binding, Grafana and the browser must run on the
same computer as `vista_edge`. Use `0.0.0.0` only when remote browser access is
required, then configure the firewall and use the machine's LAN address in the
panel. A disconnected panel or a temporarily unavailable port does not stop
LiDAR acquisition; the worker keeps draining the topic and retries the server.

`system-monitor` publishes independently of the LiDAR, so CPU, memory, worker,
storage, and delivery status remain visible while a sensor is disconnected.
On Linux it also reads compatible thermal, hwmon power, and fan entries from
`/sys`; unsupported values remain zero with `available=0`. Standard Windows
APIs provide CPU and memory, but not portable board power or temperature, so
the same availability rule applies there.

Import `../VISTA_Live_Dashboard.json` from Grafana's **Dashboards > New > Import**
screen. Its Namespace variable defaults to `vista` and must match
`GrafanaNamespace` in `DeviceConfig.txt`. The dashboard uses Grafana's built-in
Live Measurements data source, so no panel or data-source plugin is required.
