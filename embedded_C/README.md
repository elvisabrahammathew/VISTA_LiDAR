# VISTA Edge C++

This directory is the C++17 counterpart of the Rust project in `../embedded`.
The Rust source remains unchanged and can be kept as a behavioral reference.

For a clean checkout, Visual Studio 2019/2022 or VS Code setup, dependencies,
and what to commit, start with [Building VISTA C++](docs/BUILDING.md).

## Ready-to-run Release programs

The repository-level [`publish/`](../publish/README.md) contains separate
Windows x64, Linux x86-64 and Linux ARM64 packages. Windows Release links directly
to `publish/windows-x64/vista_edge.exe` with a static CRT and two TXT configs.
Linux Release builds copy their packages as before; Debug never replaces them. Existing
published DeviceConfig.txt and private GrafanaSecret.txt are preserved. See that
guide for startup, dependencies, config locations and publishing commands.

## Current map display (October 2026)

The two 3D panels now display **Room Map :8766** and **geometric Gaussian Map
:8767**. Live sensor 3D publishing on :8765 is retired, but internal sensor
processing and all LiDAR/IMU health metrics remain. Gaussian runs in an independent
CPU worker from full confirmed map checkpoints, with changed-tile/neighbor updates
and disk-backed output. No camera, clustering, detection or tracking is added.
See the [Gaussian mapping guide](4_Applications/mapping/gaussian_map/README.md).

Ground processing was retired from the runtime on 2026-10-21. Re-import the
updated existing V5 JSON to remove Ground panels; neither plugin needs an update
for this removal. Mounting, LIO, RoomMap and Gaussian algorithms remain unchanged.
See [Ground retirement and restoration notes](docs/GROUND_PROCESSING_RETIRED.md).

Update the existing `vista-lidarpointcloud-panel` from
`dashboard/plugins/lidarpointcloud/dist` to **1.3.0**, restart Grafana, hard-refresh
the browser and import **V5** (`dashboard/VISTA_Live_Dashboard_V5_Gaussian_Map.json`)
over the existing UID `vistalive`. V4 remains available as the previous file. `vistalive` and
health telemetry are unchanged. This update requirement supersedes older
schema-only upgrade notes below.

## Architecture

Worker Health uses the tag-free Grafana Live channel
`stream/vista/worker_health_rows`. Each worker name is a string column, with
one distinct millisecond row timestamp per publication batch. The dashboard
groups these rows by name and displays the latest values only, with STT starting
at 1. This avoids unnamed workers being collapsed into one null-name group.
After this schema update, rebuild/restart `vista_edge` and re-import the existing
`dashboard/VISTA_Live_Dashboard_V4_Free_Space.json` (UID `vistalive`). No panel
plugin reinstall, Grafana token change, or DeviceConfig change is needed.
That statement refers only to the Worker Health schema correction. The new
dashboard disconnect behavior additionally requires the companion
`dashboard/plugins/vistalive` data source: follow its `INSTALL.md`, then select
VISTA Live when importing the V4 JSON. Metric/status panels return No data
after 6 seconds without the existing bridge heartbeat, even with auto-refresh
off. Both custom 3D panels are unchanged. No new edge worker, publish field or
socket is required. Keep host calendar clocks synchronized and the Grafana
publish interval at 1–2 seconds. This tests producer-to-Grafana liveness, not
LiDAR liveness while the producer continues running.

The dashboard regression test uses the plugin's existing Grafana development
dependencies: run `node dashboard/unittest/worker_health_test.cjs` from the repo
root after installing those dependencies. It checks exact column order and the
latest values for 1, 9, 11, 12, 13 and 14 workers, including multiple input frames.

- `1_Platform`: shared MessageBus, bounded broadcast rings, 64-bit topic
  WaitSets, worker threads, and native priority mapping.
- `2_Transport`: TCP, UDP, serial, librealsense USB, HTTP, and local storage.
- `3_Devices`: common LiDAR interfaces plus Quanergy M8, RealSense L515, and
  Unitree L2 drivers.
- `4_Applications`: preprocessing, logging, and future application workers.
- `models`: sensor-neutral point clouds and LiDAR-specific topic envelopes.

`main.cpp` explicitly chooses and starts the independent Read, Decode,
Preprocessing, RAW Logger, PCD Logger, System Monitor, Grafana Bridge, and
Room Mapping, Room-map WebSocket, Gaussian Mapping and Gaussian-map WebSocket
workers, independent sensor-frame Live Preprocessing, plus optional LiDAR-inertial odometry for moving mapping. Each worker registers its own
publisher/subscriber endpoints on the shared MessageBus. RAW and PCD logger
workers write to filenames generated from the local system start time. Capture
runs continuously until Ctrl+C, SIGTERM, or a worker failure requests shutdown.

`main.cpp` includes only `lib.hpp`, but its `main()` body explicitly creates
each selected worker in the order chosen by the user. The
`1_Platform/workers` module contains lifecycle helpers only: validation,
completion results, priority reporting, shutdown, and join/error handling.

## Room mapping, free-space rays, and Grafana output

With `LioEnabled: 1`, a ROS-free Point-LIO adapter aligns IMU/scan timestamps,
initializes while stationary, updates its 30-DOF IMU-as-output state at IMU/point
measurement times and estimates a LiDAR-corrected
pose. Only accepted world-coordinate clouds enter preprocessing and room mapping;
missing IMU or bad localization never falls back to a fixed pose. Mount XYZ/RPY
is the initial pose, while LiDAR-to-IMU extrinsics are configured separately.
See [the LIO guide](4_Applications/mapping/lio/README.md) for the common host
monotonic clock, single IMU topic, limits, licensing and hardware validation requirements.
`LioEnabled: 0` retains the fixed-mount flow. No detection/tracking is added.

Optional `LioRobustImuEnabled` and `LioStationaryUpdateEnabled` default to 1.
They add bounded per-axis outlier downweighting and jointly qualified velocity
updates within Point-LIO, reusing its existing LiDAR correspondences. No new
worker or second ICP is introduced. Local odometry-map insertion can be gated
under this layer, but accepted full-resolution RoomMap output is not decimated.
See the LIO guide for vibration qualification, rollback/epoch safety and limits;
these layers do not guarantee elimination of stationary or moving drift.

Preprocessing publishes `pointcloud/cleaned` in world coordinates after finite/range
filtering, mounting (fixed mode only), ROI, and voxel downsampling. These are retained measured
points, not centroid-synthesized rays. Each cleaned frame carries the world
sensor origin; mounting is independent of the retired Ground branch. The
PCD logger and mapping both use cleaned world points, including the floor. Health telemetry
consumes `pointcloud/cleaned_sensor` from an independent worker, with
finite/range/voxel filtering but NO mount transform, ground removal or world ROI.
It continues when LIO is INITIALIZING, WAITING IMU, DEGRADED or LOST. Grafana's
LiDAR online/count/rate telemetry describes this live branch, not pose validity.
Runtime preprocessing has no IMU subscription; LIO and IMU health remain active.
On localization failure the existing Room Map is held; no invalid pose is integrated.
No detection/tracking worker is added in this change.

The IMU topic retains up to 4096 samples (~8 s at 500 Hz) and synchronization/
deskew retain 0.5 s of overlap history. Buffers remain bounded; this does not
guarantee real-time performance under sustained overload. An uncovered IMU
propagation interval after LIO starts enters LOST and retries initialization
after five host seconds, rather than silently reseeding the map origin. A sensor
timestamp rewind over 500 ms starts an explicit new clock epoch and immediately
resets decoder scan assembly and LIO temporal state together. Queued old-epoch
data is rejected; the accepted anchor/local map is retained, and map publication
requires fresh initialization and three qualifying local matches. Host-clock
faults remain latched. This neither restarts Unitree nor relaxes IMU quality gates.
A stale accepted result is DEGRADED,
not automatically classified as missing IMU. Original fault reasons are retained.
Only the Point-LIO numerical target uses optimized code in Visual Studio Debug
(debug symbols remain, variables may be optimized out). Other VISTA workers
retain normal Debug runtime checks. Test physical foreground and sensor calibration
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
decode and preprocessing remain available independently
of mapping. The reference is ready for future application consumers. Preserve
the sensor mounting/world frame when reusing it.

### Grafana master switch and dashboard

GrafanaEnabled controls the bridge and map WebSocket enabled flags. True
starts Grafana Bridge, Room Map (8766), and Gaussian Map (8767, when enabled); false
starts none of those outputs without disabling capture/mapping/file storage.
WebSocket enabled keys no longer appear in the TXT. Legacy enabled keys are
accepted but cannot override GrafanaEnabled, regardless of TXT line order.
Room Map address, port, point/client limits and retry settings remain configurable.
Gaussian Map has its own bind address, port and point budget; clients/retry/cadence
are inherited from Room Map. Legacy PointCloudWebSocket keys parse but never
enable the retired live-sensor socket.
The ports must differ whenever Grafana output is enabled, including map-disabled
mode. Room-map WebSocket opens even when mapping is disabled/has a load error,
and receives empty snapshots; the map-state stat explains why data is absent.

Import ../dashboard/VISTA_Live_Dashboard_V5_Gaussian_Map.json and overwrite UID
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
The supplied DeviceConfig now uses **5000 ms** for this cadence. Capture, LIO,
RoomMap integration and the configured Foreground sampling rate are independent
and unchanged. Gaussian construction adds its own processing time; this interval
is not a hard five-second end-to-end deadline. Gaussian fitting uses a bounded
per-tile padded lookup table (at most 72³ pointers, about 3 MiB on 64-bit systems)
instead of repeated neighborhood hash lookups. Fit order, covariance math, map
resolution, GSP1 wire format and the 41-byte disk record layout are unchanged;
record I/O and sequential selection avoid redundant small operations.
Foreground and Gaussian are compiled in a small dedicated OBJECT target, with
optimization enabled even in Debug, as already done for the numerical LIO core.
Debug symbols, the Debug C++ runtime and iterator ABI remain; local variables in
these two files may be optimized out. Other worker/controller code and tests keep
their normal Debug checks. No runtime worker/thread is added by this build target.
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

For Release, use the same configure preset and select the Release build/test
presets instead (run these commands from `embedded_C`):

```powershell
cmake --build --preset windows-release
ctest --preset windows-release
./out/build/windows-vs2019/Release/vista_edge.exe
```

In Visual Studio's CMake Presets interface, keep the Configure Preset set to
`windows-vs2019`, then choose `windows-debug` or `windows-release` (display name
`Windows x64 - Release`) in the Build Preset selector. Choose `vista_edge.exe`
as the startup target and press F5. If the new preset is not listed yet, reopen
the `embedded_C` folder so Visual Studio reloads `CMakePresets.json`.
The selectors are separate: the Configure Preset remains `windows-vs2019` for
both builds. Debug outputs go to `out/build/windows-vs2019/Debug`, Release outputs
to `out/build/windows-vs2019/Release`. Each folder has its own runtime
`DeviceConfig.txt` and `GrafanaSecret.txt`; builds never copy or overwrite them.

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

The presets require CMake 3.20 or newer. For Ubuntu 20.04 with CMake 3.16,
use the ADLINK script below instead. Run the ARM64 preset natively on a device
with preset support:

```bash
git submodule update --init --recursive
sudo apt install build-essential cmake pkg-config libusb-1.0-0-dev libudev-dev
cmake --preset linux-arm64-gcc
cmake --build --preset linux-arm64-debug
ctest --preset linux-arm64-debug
./out/build/linux-arm64-gcc/vista_edge
```

### ADLINK / Ubuntu 20.04 ARM64

The ADLINK deployment target is now Ubuntu 20.04 ARM64, with C++17 and
CMake 3.16 or newer. Ubuntu 18.04 / Bionic / GCC 7 is no longer an active
build or validation target. Use a vendor-supported OS image/BSP for the
device; this repository does not upgrade Ubuntu, JetPack or firmware.

Initialize submodules, then run from the repository root on ADLINK:

```bash
git submodule update --init --recursive
bash embedded_C/scripts/build_adlink.sh
cd embedded_C/out/build/linux-adlink-ubuntu20-arm64
# Missing DeviceConfig.txt is created on first launch; review it and run again.
./vista_edge
```

The script configures a Release build with tests ON, RealSense OFF, builds with
two jobs and runs all unit tests before reporting success. Override parallelism
with `VISTA_BUILD_JOBS=1` if RAM is tight. It does not install packages, connect
to hardware, start capture, or require presets/Python/CUDA. If build tools are
missing, install `build-essential cmake` through the device's configured
repositories. Existing compiler selection via `CC`/`CXX` is respected when
configuring a fresh build directory. Do not reuse a build cache or executable
from the old OS. See [the Ubuntu 20.04 ARM64 guide](docs/ADLINK_UBUNTU20_BUILD.md)
for prerequisites, configuration, rebuilds and validation limits.

RealSense OFF disables only L515; Unitree serial/UDP and Quanergy still build.
This camera-free path does not claim RealSense camera validation on ADLINK.
The application is named `vista_edge` on Linux (no `.exe` suffix), beside the
runtime `DeviceConfig.txt` and optional local `GrafanaSecret.txt`. Keep the secret
out of Git. On Windows and Linux, the application reads `DeviceConfig.txt` and
optional `GrafanaSecret.txt` beside the executable, regardless of the working
directory or a shortcut's "Start in" setting. Put these files in the same folder
as `vista_edge.exe` (Windows) or `vista_edge` (Linux); there is no fallback to a
different config in the working directory. Missing sidecars are created at startup:
`DeviceConfig.txt` uses `config/DeviceConfig.defaults.txt` embedded at build time;
`GrafanaSecret.txt`
contains `GrafanaToken:` with no token. Existing files, even empty or invalid ones,
are never overwritten. If the device config was created, the application exits
successfully without starting workers or issuing hardware commands: review the
COM/IP and other settings, fill the token if needed, then run it again. If only
the secret was created, normal startup continues (existing environment-token
fallback still applies). The executable directory must be writable when a file
is missing; errors identify the full path. No real token is embedded in the EXE.
Startup prints the full config path, and unreadable or invalid configuration
errors identify that path.
Other data/output path behavior is unchanged; the application does not change
its working directory. Builds never copy or overwrite either runtime file.
Edit only the `DeviceConfig.txt` beside the executable for your device's COM/IP
and other settings. `config/DeviceConfig.defaults.txt` is a build-time template,
not a runtime configuration: changing it affects only subsequently created files,
never an existing runtime file. The former `embedded_C/DeviceConfig.txt` is no
longer used. Local `embedded_C/GrafanaSecret.txt` is no longer copied by builds;
manage the token only in the secret file beside the executable.

For Unitree, set `SerialPort(unitree-l2)` to the actual Linux device, such as
`/dev/ttyACM0` or `/dev/ttyUSB0`, and grant the user serial access (normally the
`dialout` group). Do not assume Windows `COM4` exists on Linux. For a Grafana
server/browser on another PC, set the HTTP host to that PC and bind the two
WebSocket servers to an appropriate LAN interface; panel URLs must point to
the ADLINK address, not the browser PC's `127.0.0.1`. Restrict access with the
firewall: these WebSocket listeners are not authenticated public services.

The shared filesystem wrapper and its optional portable backend are retained;
removing an OS build target does not require changing storage or mapping code.
Normal modern builds select standard filesystem when its header/link probe
succeeds. An ARM64 binary built against newer Ubuntu libraries is not
automatically compatible with Ubuntu 20.04: build natively on the target, or
use a matching ARM64 compiler/sysroot. Test real serial I/O, LIO behavior and
sustained operation on the actual device.

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
InitializeOnConnect(unitree-l2): 1
WorkMode(unitree-l2): 9
UsbSerial(realsensel515): 123456789012
DepthWidth(realsensel515): 640
DepthHeight(realsensel515): 480
DepthFps(realsensel515): 30

# Initial LiDAR world pose for LIO; fixed mounting without LIO
MountX(lidar): 0.0
MountY(lidar): 0.0
MountZ(lidar): 0.75
MountRollDeg(lidar): 0.0
MountPitchDeg(lidar): 0.0
MountYawDeg(lidar): 0.0

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

# Map output; keep the RoomMap configuration block in DeviceConfig.txt too
RoomMapWebSocketBindAddress: 127.0.0.1
RoomMapWebSocketPort: 8766
RoomMapWebSocketMaxPoints: 100000
GaussianMapEnabled: 1
GaussianMapWebSocketBindAddress: 127.0.0.1
GaussianMapWebSocketPort: 8767
GaussianMapWebSocketMaxPoints: 100000
```

Use `Lidar: realsense-l515` for the L515. `UsbSerial` is optional: leave its
value blank to use the first matching L515, or enter the serial reported by
librealsense when selecting a specific USB device. The L515 does not use a
Windows COM port.

For the Unitree L2, select `Lidar: unitree-l2`, change the shared `SensorIP`
and `SensorPort` to the L2 UDP endpoint (normally `192.168.1.62:6101`), and
configure the PC endpoint with `LocalIP` and `LocalPort`. `ConnectionMode` can
be `serial`, `udp`, or `auto`. With `InitializeOnConnect: 1` (the default), this
selects the **initial control transport**, not the final data transport. Auto
tries serial first, then UDP. VISTA sends `WorkMode`, requires a matching
successful ACK (or exact type-107 mode readback), waits one second for persistence, sends restart once, waits two
seconds for boot, closes the control handle and receives 3D data over UART.
`SerialPort` and `BaudRate` are therefore required even for UDP initialization.
On Linux, change `SerialPort` from the Windows example `COM3` to a path such as
`/dev/ttyACM0`. UART must be physically connected; Ethernet cannot carry UART
data after the switch. The configured UDP local IP must belong to the host.

`WorkMode: 9` selects NEGA, 3D, IMU enabled, UART and self-start at power-on:

| Bit | 0 | 1 |
| --- | --- | --- |
| 0 | Normal / standard FOV | NEGA / expanded FOV |
| 1 | 3D | 2D |
| 2 | IMU enabled | IMU disabled |
| 3 | Ethernet | UART (required by this initialization flow) |
| 4 | Self-start | Standby |

Bits 5--31 are reserved and must remain zero. Gray/display options are not
encoded by these documented mode bits. The current acquisition flow rejects
2D and standby initialization (bits 1 and 4 must be zero), since its decoder
and startup probe require self-starting 3D data. The encoder follows the pinned SDK2
v2.0.10 wire output (`setLidarWorkMode`: type 2002; `resetLidar`: type 100,
command 1/value 1). Type 107 is handled as mode readback, not a write command.

Protocol references: [Unitree SDK2 mode bits](https://github.com/unitreerobotics/unilidar_sdk2#32-configuring-work-mode)
and the [Unitree Unilidar 2 user manual](https://oss-global-cdn.unitree.com/static/Unilidar%202%E7%94%A8%E6%88%B7%E6%89%8B%E5%86%8C.pdf).

Initialization state belongs to the connection configuration and survives
worker reconnects in the same application session. After a restart has been
attempted, retries open UART only and never repeatedly reset the LiDAR. If a
reset write fails, its outcome is uncertain: retries still avoid another reset;
power-cycle the sensor or restart VISTA after checking the connection if UART
does not recover. Starting a new application session initializes it again.

For **RAW replay through a virtual COM pair**, set `InitializeOnConnect: 0`.
Replay has no hardware ACKs and must never receive setup/reset commands. With
initialization disabled, the original passive serial/UDP/auto selection is
preserved: the first valid frame selects the transport, without any writes or
boot delay. Quanergy and RealSense do not execute this Unitree-only flow.

`MountX/Y/Z` and `MountRoll/Pitch/YawDeg` define the initial world pose for LIO,
or a fixed sensor-to-world transform when LIO is off. They do not estimate a floor.
Ground TXT keys are retired and rejected with a migration message; remove them
from old local deployment configurations. No Ground worker, calibration, filtering
or Ground metrics are enabled by the application. The direct reference API and
tests remain documented in `docs/GROUND_PROCESSING_RETIRED.md` for future reuse.

The Unitree wire protocol is decoded by VISTA-owned portable code rather than
linking the Linux-only `libunilidar_sdk2.a`. The pinned
`../third_party/unilidar_sdk2` submodule remains in the repository as the
official protocol and behavior reference. The same decoder is compiled on
Windows x64, Linux x86-64, and Linux ARM64.

Each frame/ACK wait and inactive read is bounded by the five-second timeout.
Hardware startup additionally includes the persistence and boot waits above;
auto mode can try both control transports. When the
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

The bridge sends `pointcloud_current`, `imu`,
`room_map_status`, `pipeline_health`, `system_health`, `worker_health`, and
`grafana_bridge_health`. It no longer subscribes to storage/power topics; those
remain available internally. Ground HTTP channels are no longer produced.
The optional legacy 3D header/codec remains compatible; no Ground values are
attached to the active map panels. Future radar, detection, tracking, and analytics
measurements can be added to the same worker without changing `main.cpp`.
A Grafana outage does not stop LiDAR capture: the
worker reports the first failure, keeps draining its topic queues, and retries
using `GrafanaRetryIntervalSeconds`.

Map geometry is sent through separate sockets, not the HTTP metrics bridge.
Room Map uses LPC1/octree LOD. Gaussian Map uses GSP1 covariance/opacity records
and bounded regional views. The production application no longer starts
`pointcloud-websocket` for live sensor frames. The retained LPC1 encoder/worker
API is only for compatibility and tests. Current panel addresses are:

```text
ws://<RoomMapWebSocketBindAddress>:<RoomMapWebSocketPort>
ws://<GaussianMapWebSocketBindAddress>:<GaussianMapWebSocketPort>
ws://<ForegroundWebSocketBindAddress>:<ForegroundWebSocketPort>
```

The optional independent `foreground-filter` branch samples the latest accepted world
scan every 250 ms and publishes current points outside the latest causally older,
completed confirmed-background checkpoint on
port 8768. It uses full-resolution map regions, not renderer LOD or just resident
mapper voxels. New stationary objects/unmapped surfaces also appear; this is not
a motion label. It does not restore Ground processing or add semantic classes
and persistent tracking IDs.
See [Room Map foreground](docs/ROOM_MAP_FOREGROUND.md)
for configuration, resource limits, safety pauses, and the Grafana plugin update.
Foreground's optional same-scan isolation filter uses a sensor-range-adaptive
radius (default 0.15 m through 1 m, growing linearly to a 0.30 m cap at 5 m), not distance
from the map origin. Its two-neighbor rule and bounded work remain; RoomMap's
confirmation and subtraction tolerance are unchanged. Set
`ForegroundOutlierMaxRadiusMeters` equal to `ForegroundOutlierRadiusMeters` for
fixed-radius A/B tests. No Grafana update is needed for this filtering change.
The October 22 pipeline-budget update also needs only a rebuilt C++ application:
Foreground tolerates localization heartbeat scheduling jitter for 2.5 seconds,
but still clears on explicit invalid poses or scans older than 500 ms. Bounded
idle-time prefetch warms current background pages without a new worker or an
extra whole-map copy. See the foreground guide for read/time limits and Unknown
handling. No plugin rebuild or dashboard reimport is required for this update.

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

Import `../dashboard/VISTA_Live_Dashboard_V5_Gaussian_Map.json` from Grafana's **Dashboards > New > Import**
screen. Its Namespace variable defaults to `vista` and must match
`GrafanaNamespace` in `DeviceConfig.txt`. The dashboard uses Grafana's built-in
VISTA Live companion data source and the existing lidarpointcloud panel plugin.
