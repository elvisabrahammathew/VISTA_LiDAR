# LiDAR-inertial moving mapping

This is a ROS-free **adaptation**, not a drop-in build of the upstream ROS node.
FAST-LIO is pinned at `7cc4175de6f8ba2edf34bab02a42195b141027e9` and its
ikd-Tree at `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4`.
The upstream IKFoM state/process model and iterated error-state filter are reused.
The VISTA adapter owns temporal synchronization, IMU initialization, interpolation,
deskew, point-to-plane residual construction, quality gates and local-map budgets.
Plane fitting uses a PCA plane so a floor through the world origin is supported.
LiDAR-to-IMU extrinsics remain fixed; online extrinsic estimation is disabled.

## Data flow

```text
LiDAR read (host monotonic receipt) -> raw topic -> decoder -> pointcloud/decoded
                                                        +--> live filters -> pointcloud/cleaned_sensor -> :8765
                                                        +--> lidar/imu
External IMU driver (alternative to integrated IMU) ---------> lidar/imu
                                   |
                      LIO synchronization + stationary initialization
                      IMU prediction + deskew + LiDAR correction
                                   |
                 localization/pointcloud_world (accepted poses only)
                                   |
                    preprocessing (no second mounting transform)
                                   |
                         pointcloud/cleaned (floor retained)
                            /                   \
             disk-backed Room Map           existing ground processing
                  |                               |
             ws://...:8766                   pointcloud/processed -> optional PCD logger
```

No detection/tracking is added. RAW and processed PCD logging keep their existing
configuration. The local LIO map is bounded and independent of the disk-backed
Room Map. Live rendering and LiDAR telemetry now use the independent sensor-frame
branch, including the floor. They never depend on a valid LIO pose. Mount/world
ROI and world-ground overlays are not applied to this branch. A LOST pose holds
the Room Map instead of supplying a fixed fallback. Future local detection can
consume this branch; world association/tracking will need a valid pose separately.
Each compensated point carries its acquisition-time world ray origin for
free-space clearing; filtering preserves that metadata. PCD, tiles and the
existing WebSocket point formats do not change.

## Running and calibration

1. Initialize recursively: `git submodule update --init --recursive`.
2. Build using the existing Windows/Linux presets. CMake builds `vista_fast_lio`
   for the selected compiler/CPU and links it into `vista_edge`; no ROS/PCL/Python
   runtime or prebuilt platform-specific LIO binary is needed.
3. `LioEnabled: 1` selects the moving pipeline. `0` retains the old fixed-mount
   pipeline. Keep the device stationary for at least 0.5 seconds and the configured
   number of valid IMU samples during initialization. Motion restarts this window.
4. Mount XYZ/RPY describes the **initial LiDAR pose in the world**, not its
   continuing position. LiDAR-to-IMU XYZ/RPY describes a separate fixed transform
   from LiDAR coordinates into IMU coordinates. RPY is degrees, XYZ is metres.
5. The example Unitree extrinsic translation `(0.007698, 0.014655, -0.00667)`
   and identity rotation come from the SDK2 ROS driver's published static TF.
   Check the exact hardware/calibration. For a different LiDAR/external IMU,
   start only the chosen IMU producer and replace extrinsics; do not reuse Unitree defaults.
6. IMU acceleration is measured specific force **including gravity**, in m/s²;
   gyro is rad/s. The IMU quaternion is not treated as a position measurement.
7. Per-point timestamps must be valid and scan duration must not exceed 500 ms.
   Ground-removal/voxel-downsampled clouds must NOT be fed into LIO first.

### Time alignment

When LIO is enabled, all measurements use the **Windows/ADLINK host monotonic
clock** (`std::chrono::steady_clock`). It is not calendar time and cannot jump
when Windows/NTP changes the date. Calendar timestamps are retained separately
for Grafana and log filenames. There are no topic/source/clock/offset TXT settings.

The read worker stamps each RAW receipt before publication. The decoder preserves
that stamp and uses `platform::MeasurementClock` to anchor the hardware clock to
the host. Sensor timestamps supply relative sample intervals and reset/order
checks, never a cross-device absolute epoch. Unitree LiDAR and its integrated
IMU use the **same** adapter, preserving their actual relative timing. Aggregated
scan start is reconstructed from the last RAW packet reference and each point's
timestamp; it is not approximated as receipt time minus scan duration.

The adapter slowly follows arrival-clock drift with a bounded correction
(100 ppm per source-time advance). USB batching cannot compress a 5 ms sensor
period into a 0.1 ms decode period. If an IMU has no timestamp, receipt time is
the fallback, with lower timing quality; it cannot recover hidden batch timing.
The point-cloud path still requires valid per-point timestamps for deskew.

Both integrated and future external drivers publish `models::ImuMessage` to the
fixed `lidar/imu` topic. `devices::LidarImuMessage` is an alias of that exact type,
so ground processing and Grafana use it unchanged. The legacy `lidar_id` envelope
field identifies the originating IMU for monitoring; it does not select a source.
Start only **one** IMU producer. A producer-ID change within an LIO session latches
LOST rather than mixing sensors. No external-IMU hardware driver is added here.

An external driver must capture `received_monotonic_ns` at transport receipt,
keep its own `platform::MeasurementClock`, and populate `measurement_timestamp_ns`
with `clock.align(sensor_timestamp_or_nullopt, received_monotonic_ns)`. Do this
before queues/decoding; do not substitute calendar time or worker execution time.
A failed alignment must remain null so LIO rejects it. The original device
timestamp stays in the payload for relative-time/reset checks.

Arrival anchoring is **not hardware synchronization**: independent devices can
still have different transport latency, and this cannot be inferred precisely
from sample deltas alone. Use synchronized or latency-calibrated acquisition in
the external driver when available. The common clock does not guarantee an
accurate moving map by itself.

Synchronization requires IMU measurements before/at scan start and after/at
scan end. Buffers are bounded (4096 IMUs, four pending scans); the synchronization
timeout reports missing/gapped IMU rather than extrapolating indefinitely.
Prediction across missing LiDAR scans requires continuous IMU coverage and is
limited to two seconds. An uncovered IMU gap cannot be recovered by guessing a
new world origin; inspect the status and restart after correcting the source.
Backward device clocks and out-of-order samples latch LOST: restart is required
so the world origin cannot silently reset inside an existing Room Map.

### Quality and memory

The filter estimates position, orientation, velocity, IMU biases and gravity.
Each corrected pose is checked for effective matches, residual, geometry,
covariance and excessive pose steps. Rejected poses do not update either map.
After sustained failures the pipeline latches LOST. IMU-only long-term mapping
and fixed-mount fallback are intentionally prohibited in moving mode.

`LioScanVoxelSizeMeters`/`LioScanMaxPoints` bound matching work.
`LioMapVoxelSizeMeters`, `LioLocalMapRadiusMeters` and `LioLocalMapMaxPoints` bound
the odometry map only; they do not cap the persistent room map or PCD export.
Remaining TXT limits control initialization, IMU gaps, synchronization timeout,
match ratio, residual and pose jumps. A pose passing these checks is not a proof
of absolute accuracy; recorded real-device movement tests are still required.

Use a **Release** build for real-time deployment. Windows Debug deliberately keeps
runtime checks and unoptimized Eigen/filter code; it can be much slower and cause
input drops even when the same configuration keeps up in Release. Observe LIO
Input Drops, pose quality and host load on the target hardware before increasing
scan/local-map budgets. A bounded queue prevents unlimited backlog, not overload.

This is local odometry, **without loop closure or automatic relocalization into
a loaded PCD map**. Mount/world origin must agree with a loaded reference map.
Odometry drift can accumulate during long trajectories. Never claim a globally
consistent large-area map solely from the IMU or this local estimator.

## Build isolation and license

`cmake/fast_lio.cmake` generates modified copies only under the build directory:
unused Boost.Bind include removed, Boost.Math epsilon replaced by the standard
equivalent, OpenMP wall time replaced by steady-clock time, private point layouts
replace PCL storage types, and ikd-Tree POSIX calls use private C++17 wrappers.
The hidden asynchronous tree rebuild is disabled; the owning LIO worker controls
updates. Original submodules remain unchanged.

Derived FAST-LIO algorithm code is subject to the upstream GPLv2 LICENSE, copied
as `core/COPYING`. IKFoM headers preserve their own upstream license notices;
Eigen and Boost.Preprocessor retain their respective license files in submodules.
Review distribution requirements before shipping binaries.

## Verification (2026-10-05)

The complete C++ test executable passes **152/152 tests** in each environment:

| Build | Test execution |
| --- | --- |
| Windows x64 / Visual Studio 2019 / Debug | Native Windows |
| Linux x86-64 / GCC / Debug | Ubuntu WSL |
| Linux ARM64 / GCC cross-build / Debug | QEMU Cortex-A57, ARM64 userspace |

All three builds include the static librealsense SDK. LIO tests cover stationary
and moving synthetic rooms (translation and rotation), deskew, floor preservation,
initial pose vs extrinsics, independent clock epochs, USB batching, IMU gaps,
one-IMU enforcement, shared decoder timing, calendar-clock independence,
degenerate geometry, clock-reset safety, per-ray moving origins and Grafana fields.
The dashboard JSON is checked for UID/channel/field agreement and layout overlap.

These checks do **not** constitute a real Unitree motion trial, an ADLINK hardware
benchmark, a 24/7 soak test, or visual verification inside a running Grafana browser.
Validate IMU units, per-point timing, clock offsets and extrinsics with recorded
hardware trajectories before relying on the map's metric accuracy.

Sources: [FAST-LIO](https://github.com/hku-mars/FAST_LIO),
[ikd-Tree](https://github.com/hku-mars/ikd-Tree),
[Unitree SDK2](https://github.com/unitreerobotics/unilidar_sdk2).
