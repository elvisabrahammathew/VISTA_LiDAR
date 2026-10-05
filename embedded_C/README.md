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
Point-cloud WebSocket, Ground Processing, Room Mapping, and Room-map WebSocket
workers, independent sensor-frame Live Preprocessing, plus optional LiDAR-inertial odometry for moving mapping. Each worker registers its own
publisher/subscriber endpoints on the shared MessageBus. RAW and PCD logger
workers write to filenames generated from the local system start time. Capture
runs continuously until Ctrl+C, SIGTERM, or a worker failure requests shutdown.

`main.cpp` includes only `lib.hpp`, but its `main()` body explicitly creates
each selected worker in the order chosen by the user. The
`1_Platform/workers` module contains lifecycle helpers only: validation,
completion results, priority reporting, shutdown, and join/error handling.

## Room mapping, free-space rays, and Grafana output

With `LioEnabled: 1`, a ROS-free FAST-LIO adapter aligns IMU/scan timestamps,
initializes while stationary, deskews measurements and estimates a LiDAR-corrected
pose. Only accepted world-coordinate clouds enter preprocessing and room mapping;
missing IMU or bad localization never falls back to a fixed pose. Mount XYZ/RPY
is the initial pose, while LiDAR-to-IMU extrinsics are configured separately.
See [the LIO guide](4_Applications/mapping/lio/README.md) for the common host
monotonic clock, single IMU topic, limits, licensing and hardware validation requirements.
`LioEnabled: 0` retains the fixed-mount flow. No detection/tracking is added.

Preprocessing publishes `pointcloud/cleaned` in world coordinates after finite/range
filtering, mounting (fixed mode only), ROI, and voxel downsampling. These are retained measured
points, not centroid-synthesized rays. Each cleaned frame carries the world
sensor origin; mounting remains active with GroundMode none. Ground Processing
consumes cleaned points and publishes processed points/ground diagnostics. The
PCD logger uses processed world points; mapping keeps the floor. The live panel
instead consumes `pointcloud/cleaned_sensor` from an independent worker, with
finite/range/voxel filtering but NO mount transform, ground removal or world ROI.
It continues when LIO is INITIALIZING, WAITING IMU, DEGRADED or LOST. Grafana's
LiDAR online/count/rate telemetry describes this live branch, not pose validity.
World ground diagnostics remain separate; they are not overlaid on sensor XYZ.
On localization failure the existing Room Map is held; no invalid pose is integrated.
No detection/tracking worker is added in this change.

The IMU topic retains up to 4096 samples (~8 s at 500 Hz) and synchronization/
deskew retain 0.5 s of overlap history. Buffers remain bounded; this does not
guarantee real-time performance under sustained overload. An uncovered IMU
propagation interval after LIO starts latches LOST and requires restart, rather
than silently reseeding the map origin. A stale accepted result is DEGRADED,
not automatically classified as missing IMU. Original fault reasons are retained.
Only the FAST-LIO numerical target uses optimized code in Visual Studio Debug
(debug symbols remain, variables may be optimized out). Other VISTA workers
retain normal Debug runtime checks. Test physical motion and sensor calibration
on target hardware before treating the map as accurate.

### Build a new room map

RoomMapEnabled 1 with RoomMapLoadExisting 0 selects building mode. Only this
mode subscribes to live cleaned geometry. Each voxel needs three hit windows
(default 100 ms) to be shown. Repeated points in one window count once.

Real rays from the acquisition-time world sensor origin to measured endpoints provide free-space
evidence BEFORE the surface. Confirmed voxels behind a person/other occluder
are preserved. Missing frames, missing returns, and merely absent points never
clear confirmed geometry. Centroids are map storage only, not ray endpoints.
A cell is cleared after five free windows by default; hits anywhere in the
same window take priority across packet boundaries. A fresh hit resets its
free-evidence counter. Tentative candidates can still expire.

The DDA traversal avoids corner-only contacts and preserves a surface margin
of at least one voxel diagonal. Rays are sampled to bound CPU work; the actual
cleanup delay may exceed five windows. Unknown/free voxels are NOT allocated.
There is no total candidate/confirmed voxel-count cap. Spatial tiles are written
to disk before LRU eviction from the bounded RAM working set.

### Out-of-core spatial storage and camera-dependent LOD

The mapper uses a sparse octree with bounded representative samples at every
level. This follows Potree's principles of spatial hierarchy, multi-resolution
samples, and camera/frustum selection; it is not the Potree package/file format.
The existing Grafana plugin is reused, rather than introducing a second plugin.

| Key | Default | Meaning |
| --- | --- | --- |
| RoomMapCacheMaxVoxels | 250000 | Maximum voxel records in the RAM tile cache, not the complete map. |
| RoomMapCacheMaxTiles | 128 | Maximum resident spatial tiles, including sparse tiles. |
| RoomMapTileSizeMeters | 2.0 | Tile edge; rounded up to a whole number of map voxels. |
| RoomMapLodPointsPerNode | 128 | Bounded representative samples in each octree node. |
| RoomMapWebSocketMaxPoints | 100000 | Server budget for one camera view, not the stored map. |

One tile must fit RoomMapCacheMaxVoxels and have at most 64 voxels per axis.
For 2 m tiles and 0.05 m voxels, a tile contains at most 40^3 = 64000 cells.
RAM also includes bounded previews, network packets, ray evidence, and index
traversal buffers; the voxel budget is not a byte-accurate whole-process limit.
Disk LOD summaries have a separate bounded 512-node cache, invalidated on node
replacement; this avoids reopening the same summaries for every camera query.
The sparse on-disk index avoids a RAM catalogue proportional to total tiles.

Building creates a uniquely named RoomMap_<session-time>.tiles.<id> directory
beside the session PCD. Voxel evidence and LOD nodes are stored here as the map
grows. Export streams every confirmed point into one ASCII PCD; it does not
construct a whole-map vector and does not export merely the displayed preview.
Large read-only PCDs are streamed into a separate tile index; duplicates and the
original PCD are preserved. Keep the .tiles directories while the application
is running. They are retained after shutdown, not deleted automatically.
This version loads references from PCD, not directly from an old .tiles directory;
automatic crash recovery of partially updated indexes is not implemented.

The shared mapping/room_map_view topic carries a source handle and revision.
The map WebSocket receives a bounded VIEW camera/frustum request per panel,
selects a non-overlapping octree cut, and sends only that view. Near regions get
more detail; distant/out-of-view regions cost less or are omitted. The plugin
replaces its current view instead of accumulating every rendered point forever.
mapping/room_map remains a bounded overview for compatibility, NOT a full map.
It is materialized only while a subscriber exists; late subscribers still receive
an overview on the next publication tick. The normal LOD WebSocket uses only
mapping/room_map_view and does not force a second large preview to be built.
The total map remains available on disk/PCD even when render budgets are small.

This removes the fixed total count, not physical resource limits: disk can fill,
I/O can lag, queues can drop input, and very large PCD exports can take time.
Watch cache evictions, disk tiles, input drops, memory, and STORAGE ERROR on the
dashboard. A storage failure reports an optional map-worker error, not a sensor
shutdown. LOD readers query disk without flushing RAM or holding the accumulator
update mutex. Disk reads and atomic page replacement still share a separate disk
mutex: an eviction/checkpoint can wait for a view query, while resident-tile hit
updates can proceed. These are live disk views, not immutable historical revisions;
during building, geometry may be newer than the most recently published metadata.
Spatial extent is finite (eight signed roots, 24 tile levels); this is not SLAM.

Hits and completed free evidence are processed in tile order to reduce LRU
thrashing. Read-only lookups do not mark pages dirty. Evidence-only page writes
do not rebuild LOD nodes unless confirmed count/samples change. Mapping batches
yield after a completed frame at a 20 ms budget (one frame may exceed it), rather
than draining up to 64 slow frames before reporting status. No extra worker was
added: complete PCD export remains in mapping on freeze/clean shutdown.

Room Map Performance Diagnostics exposes integration/checkpoint/view time,
map/disk lock waits, mapping input drops, and tile/LOD I/O counters. Cumulative
read/write/LOD wall times include decoding/callback work, not just hardware I/O.

Additional TXT settings (building mode only):

| Key | Default | Meaning |
| --- | --- | --- |
| RoomMapFreeSpaceMinObservations | 5 | Distinct free windows before removing a cell; minimum 2. |
| RoomMapRaycastMaxRaysPerWindow | 256 | Maximum sampled rays per observation window. |
| RoomMapRaycastMaxRangeMeters | 30.0 | Maximum traversed free-space range, not mapping's hit range. |
| RoomMapRaycastSurfaceMarginMeters | 0.10 | Stop before the measured surface; at least one voxel diagonal is preserved. |

A hard limit of 250000 voxel traversal steps per window bounds traversal work
even with very fine voxels. Counters distinguish rays, traversal steps,
unprocessed ray work, free checks, cleared confirmed voxels, and missing
world origins. Missing origins permit fusion but disable clearing safely.

RoomMapFreezeAfterSeconds 30 freezes both fusion AND clearing after active
acquisition. Use 0 to continue updating while people may enter/leave. Once
frozen, the same immutable snapshot is served to late/reconnecting clients.
There is no runtime control TXT file; change DeviceConfig and restart.

Building sessions always save a replaceable ASCII PCD on freeze and clean
shutdown, independently of RAW/processed logging; there is no save toggle.
Each building session automatically selects
../data/maps/RoomMap_YYYYMMDD_HHMMSS.pcd using local system time at startup,
the same timestamp as RAW/processed logs. RoomMapFile is ignored in building
mode and is used only as the input path when RoomMapLoadExisting is 1.
The first save exclusively claims a new file on Windows/Linux; an existing
filename (including another session starting in the same second) adds _1,
_2, etc. All later saves in that session update only its own file. The console
prints the actual filename when saving. Do not terminate power/debug abruptly if you need
the latest continuously building snapshot saved. No-data startup does not
create an empty PCD or overwrite an older map; a map whose confirmed geometry
was entirely cleared can save an empty update to its own session file.

### Load a read-only reference

RoomMapEnabled 1 with RoomMapLoadExisting 1 reads RoomMapFile and preserves
every saved point exactly, even if the current voxel setting differs. Loaded
mode does not subscribe to live geometry, integrate hits, cast rays, or save
over the original file. A missing/invalid/empty file reports LOAD ERROR; live
capture continues, and mapping does NOT silently fall back to reconstruction.
Set RoomMapFile to the exact existing PCD path; an empty value is LOAD ERROR,
not a request to select the newest map automatically.

RoomMapEnabled 0 publishes DISABLED status and empty map snapshots. The map
service remains alive to supply status/replays, but does not build or load.

Detection and tracking are not implemented by this change. Live sensor read,
decode, preprocessing, and ground processing remain available independently
of mapping. The reference is ready for future application consumers. Preserve
the sensor mounting/world frame when reusing it.

### Grafana master switch and dashboard

GrafanaEnabled controls the bridge and BOTH WebSocket enabled flags. True
starts Grafana Bridge, live WebSocket (8765), and map WebSocket (8766); false
starts none of those outputs without disabling capture/mapping/file storage.
WebSocket enabled keys no longer appear in the TXT. Legacy enabled keys are
accepted but cannot override GrafanaEnabled, regardless of TXT line order.
Address, port, point/client limits, and retry settings are still configurable.
The ports must differ whenever Grafana output is enabled, including map-disabled
mode. Room-map WebSocket opens even when mapping is disabled/has a load error,
and receives empty snapshots; the map-state stat explains why data is absent.

Import ../dashboard/VISTA_Live_Dashboard_V4_Free_Space.json and overwrite UID
vistalive. It keeps both 3D panels, numeric map states (EMPTY, BUILDING, FROZEN,
LOADED / READ ONLY, DISABLED, LOAD ERROR, STORAGE ERROR), and ray/cache diagnostics.
Both map network/panel budgets default to 100000 points per camera view.
Total disk/PCD geometry is not limited by that number.

Selected-LiDAR stats use stream/vista/pointcloud_current (with the configured
namespace in place of vista). Its tag-free fields keep startup OFFLINE and live
ONLINE samples in one series. The bridge now sends only fields consumed by the
current dashboard; the legacy tagged pointcloud formatter remains an API but its
channel is no longer published. Data Age includes bridge/queue/publication waiting;
Message Rate is messages/s, not necessarily hardware scan FPS. Pipeline History
owns the bridge's raw/decoded/processed drop counter, not mapping drops.

`RoomMapPublishIntervalMilliseconds` is the single map checkpoint/publication
cadence. Changed tile/LOD data is checkpointed before announcing a new revision;
8766 sends that revision without a second interval delay. Unchanged heartbeats
support late worker startup but do not resend 3D geometry. Reconnects and camera
changes still receive a view; repeated same-map camera queries are bounded by
that same configured interval. Freeze/shutdown can perform a final checkpoint.
Periodic checkpoints do not rewrite the entire session PCD: PCD export remains
on freeze/clean shutdown. Checkpoint means flushed tile/LOD files, not fsync-level
power-loss durability. Grafana metrics keep their independent publish interval.
Rebuild/restart VISTA and reimport this existing dashboard (UID vistalive). This
update does not change the plugin protocol or require a newer plugin than 1.1.0.

Update the installed vista-lidarpointcloud-panel to version 1.1.0 using this
repository's dashboard/plugins/lidarpointcloud/dist. This version accepts map
metadata, sends camera/frustum requests, shows stored vs rendered counts, and
keeps live LPC1/LDR1 streams compatible. Copy the dist contents into the existing local plugin
directory, restart Grafana, then refresh the browser. No new plugin ID is needed.
For a remote browser, replace 127.0.0.1 with the VISTA machine's address and set
the server bind address/firewall appropriately.

This is stationary-sensor mapping, not SLAM, semantic person removal, or a mesh.
A person who stays may become map geometry; occluded confirmed room points
stay behind them. Points disappear only when later measured rays provide free
evidence while building. No inference can reconstruct never-observed surfaces.
Keep the LiDAR fixed and prefer a stable calibrated mount over changing IMU
orientation when constructing a reusable reference.

Unit-test sources cover hit/free windows, occlusion, disappearance, occupied
precedence, negative directions/nonzero origins, ray work limits, strict load
failure, read-only references, master-switch behavior, storage and late replay.

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

# LiDAR mounting and ground calibration (hybrid, ransac, static)
GroundMode(lidar): hybrid
UseImuForGround(lidar): 0
MountX(lidar): 0.0
MountY(lidar): 0.0
MountZ(lidar): 0.75
MountRollDeg(lidar): 0.0
MountPitchDeg(lidar): 0.0
MountYawDeg(lidar): 0.0
FloorZ(lidar): 0.0
GroundDistanceThreshold(lidar): 0.10
GroundNormalToleranceDeg(lidar): 15.0
GroundCalibrationFrames(lidar): 50
GroundMinInlierRatio(lidar): 0.20

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

Ground processing first converts sensor coordinates to the common world frame
with `MountX/Y/Z` and `MountRoll/Pitch/YawDeg`. `FloorZ` is the floor height in
that world frame. `static` removes only the configured floor band; `ransac`
collects `GroundCalibrationFrames` before selecting a plane; `hybrid` uses the
configured floor immediately and replaces it with a validated RANSAC plane.
Calibration runs once per process start. `UseImuForGround` defaults to `0`; when
set to `1`, a stationary gravity sample supplies roll/pitch while configured yaw
is preserved. Missing or invalid IMU data automatically falls back to the
configured mounting angles.

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
processed point-cloud topics. It calculates counts, message rate, data age and
queue drops without scanning XYZ bounds, then batches compact Influx line-protocol
measurements per configured interval to:

```text
http://<GrafanaHost>:<GrafanaPort>/api/live/push/<GrafanaNamespace>
```

The bridge sends `pointcloud_current`, `imu`, `ground_state`, `ground_status`,
`room_map_status`, `pipeline_health`, `system_health`, `worker_health`, and
`grafana_bridge_health`. It no longer subscribes to storage/power topics; those
remain available internally. Ground plane parameters remain in the 3D header,
not duplicated in HTTP metrics. Future radar, detection, tracking, and analytics
measurements can be added to the same worker without changing `main.cpp`.
A Grafana outage does not stop LiDAR capture: the
worker reports the first failure, keeps draining its topic queues, and retries
using `GrafanaRetryIntervalSeconds`.

The full XYZ/intensity cloud is intentionally sent through the separate
`pointcloud-websocket` worker instead of the HTTP metrics bridge. It subscribes
to `pointcloud/cleaned_sensor` (sensor coordinates, floor retained), samples oversized frames to
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
