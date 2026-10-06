# LiDAR-inertial moving mapping

This is a ROS-free **adaptation**, not a drop-in build of the upstream ROS node.
FAST-LIO is pinned at `7cc4175de6f8ba2edf34bab02a42195b141027e9` and its
ikd-Tree at `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4`.
The upstream IKFoM state/process model and iterated error-state filter are reused.
The VISTA adapter owns temporal synchronization, IMU initialization, interpolation,
deskew, point-to-plane residual construction, quality gates and local-map budgets.
Plane fitting uses a PCA plane so a floor through the world origin is supported.
Eight-neighbor surface fits tolerate ordinary sampling/noise variation, require
non-collinear support, and verify every neighbor against a 10 cm plane-distance
limit. This changes correspondence construction, not the configured acceptance
limits for pose residual, match ratio, covariance or pose jumps.
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
   number of valid IMU samples during initialization. A sliding bounded window
   checks mean acceleration/gyro and their RMS variation; ordinary isolated
   acceleration noise no longer discards all accumulated samples. Sustained
   motion still prevents initialization. The existing Grafana `Reason` field
   reports sample count, duration and the acceleration/gyro stability limits.
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

Stationary initialization also determines a fixed acceleration scale,
`9.81 / norm(mean_acceleration)`, matching upstream FAST-LIO. Only the LIO
prediction input and its acceleration process noise are scaled; the published
IMU, RAW recording and gyro measurements are unchanged. The scale is not
re-estimated while moving. This prevents a stationary reading such as
10.21 m/s² from becoming artificial acceleration against 9.81 m/s² gravity.
It is not a substitute for axis-specific IMU calibration or LiDAR correction.

When the device sits on the floor, `MountZ(lidar)` is the optical-center height
above the chosen world floor, not automatically zero. Verify the mounting
orientation as well. Placement alone does not justify changing the config.
LIO consumes the decoded cloud including the floor, but a mostly flat floor
without other visible geometry may still fail its observability checks.

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
fixed `lidar/imu` topic. This is an independent envelope from
`models/imu/imu_message.hpp`, not a specialization of `LidarMessage`.
The `source_id` field identifies the originating IMU; it does not select a source.
Ground processing and Grafana subscribe to this same independent type. Grafana
keeps the existing wire label `lidar_id` for dashboard compatibility.
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
The worker drains the actual IMU queue before each pending scan, including
measurements that arrived while the previous scan was being processed. A
batch lookup no longer discards IMU history. History is pruned only relative to
the engine's actual integrated horizon, retaining 0.5 seconds of overlap and its
left bracket. A rejected/unintegrated scan cannot advance that horizon. Under
overload, the worker chooses the latest queued scan and solves at most one scan
per ingestion iteration; skipped LiDAR scans remain counted as input drops.
Continuous IMU is still required across the skipped interval. A missing-IMU
deadline persists across latest-scan selection so fresh LiDAR cannot hide a
stopped IMU indefinitely. The engine distinguishes nonmonotonic scan ends,
missing integrated-horizon coverage and propagation backlogs above two seconds.
The timeout's existing Grafana `Reason` field distinguishes missing left/right
brackets from excessive sample gaps and reports the requested/buffered time
ranges, sample count, capacity evictions and topic/pending-scan drop counts.
These times are seconds relative to this session's host-clock reference, not
calendar timestamps. No extra telemetry channel or dashboard field is needed.
Prediction across missing LiDAR scans requires continuous IMU coverage and is
limited to two seconds. An uncovered IMU gap cannot be recovered by guessing a
new world origin. The worker enters LOST and retries initialization after five
continuous seconds of **host steady-clock time**. It discards stale synchronization
buffers and pending scans, then requires fresh valid IMU brackets. Source-ID
checks remain enforced; recovery must not mix two sensors. A decoder/driver
whose clock adapter has latched invalid alignment still needs its source repaired
or reconnected: automatic initialization cannot invent missing measurement times.

### Quality and memory

The filter estimates position, orientation, velocity, IMU biases and gravity.
Each corrected pose is checked for effective matches, residual, geometry,
covariance and excessive pose steps. Rejected poses do not update either map.
Quality rejection persists as DEGRADED for up to two acquisition seconds,
independent of scan frequency, then becomes LOST. Unsafe time coverage can enter
LOST immediately. After five continuous host seconds in LOST the worker starts
IMU initialization again, preserving the last accepted world pose and local map.
When there has never been an accepted pose, an untrusted startup seed is discarded
and rebuilt at the configured initial pose. An accepted local map is never
reseeded with guessed geometry. Following recovery, LiDAR must match that retained
map and pass all quality/jump checks before any world cloud reaches RoomMap.
This is a local retry, not global relocalization; if the device moved too far
during loss, recovery may continue to fail safely. RoomMap is held, not cleared.
IMU-only long-term mapping and fixed-mount fallback remain prohibited.

At startup, stable scans are accumulated into the local map before matching.
A finite-noise zero-velocity observation is applied only after at least 0.5 s
of persistent sensor-frame surface agreement and stable measured IMU, after a
LiDAR-corrected pose has been accepted. Non-repeating Unitree points are compared
to surfaces in a bounded, voxelized sensor-frame reference accumulated during a
short startup window, rather than requiring 80% of individual points to repeat
within 5 cm of a sparse single-scan reference. Full six-DOF surface observability,
at least 100 supported probes, 30% support, a median residual below 2.5 cm and a
small identity-pose correction are required. The mature reference is frozen;
it is not continuously updated to follow constant-velocity motion. Measured
acceleration stability is compared with that reference window, not the filter's
possibly drifting gravity/orientation prediction. The observation keeps the fixed extrinsics unchanged
and uses a Joseph covariance update. It does not bypass geometry/quality checks
or publish IMU-only poses. Ambiguous scenes/constant-speed motion can defeat any
stationary detector; validate on real trajectories, not only a desk test.
The reference is capped at 20000 points (10 cm voxels), with at most 1000 matching
probes per scan, and the IMU window at 100000 samples;
typical runtime IMU windows retain about 0.75 s, not the entire session.

`Reason` now distinguishes insufficient correspondences, degenerate geometry,
match ratio, residual, non-finite state, covariance, translation and rotation
limits. It includes actual matches, residual, covariance, correction and minimum
geometry eigenvalues. Non-tracking telemetry shows the last trusted pose (or
initial configured pose), with `pose_valid=false`; drifting internal IMU
prediction is not presented as a usable world pose. Live :8765 remains independent.
Tracking/degraded `Reason` also reports rest surface support, median surface
residual and identity-pose translation correction; -1 means unavailable, not
zero error. These use the existing Reason field, with no new dashboard channels.

`LioScanVoxelSizeMeters`/`LioScanMaxPoints` bound matching work.
`LioMapVoxelSizeMeters`, `LioLocalMapRadiusMeters` and `LioLocalMapMaxPoints` bound
the odometry map only; they do not cap the persistent room map or PCD export.
Remaining TXT limits control initialization, IMU gaps, synchronization timeout,
match ratio, residual and pose jumps. A pose passing these checks is not a proof
of absolute accuracy; recorded real-device movement tests are still required.

Use a **Release** build for real-time deployment. The isolated numerical LIO
target is optimized in Windows Debug (/O2 /Ob2, with /Zi symbols, without /RTC1);
the rest of the application keeps its existing Debug configuration. Debug can still cause
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

## Verification (2026-10-06)

The complete C++ suite now contains **191 tests**, including stationary
non-repeating surface samples, quiet-IMU constant-velocity motion, unintegrated
IMU-history preservation, separate temporal-failure diagnostics and a stopped
IMU with continuous LiDAR input. Default execution still runs every test; an optional
substring argument (e.g. `vista_edge_tests lio_`) selects a focused group and
returns an error if no test matches.

| Build | Test execution |
| --- | --- |
| Windows x64 / Visual Studio 2019 / Debug | Native Windows |
| Linux x86-64 / GCC 13 / Debug | Ubuntu 24.04 WSL |
| Linux x86-64 / GCC 7.5 / CMake 3.10 | Ubuntu 18.04 userspace in WSL |
| Linux ARM64 / GCC 7.5 cross-build / CMake 3.10 | QEMU, Ubuntu 18.04 ARM64 userspace |

Windows and Ubuntu 24.04 builds include the static librealsense SDK; the legacy
Ubuntu 18.04 x86/ARM configurations disable the optional L515 backend. LIO tests cover stationary
and moving synthetic rooms (translation and rotation), deskew, floor preservation,
initial pose vs extrinsics, independent clock epochs, USB batching, IMU gaps,
one-IMU enforcement, shared decoder timing, calendar-clock independence,
degenerate geometry, clock-reset safety, per-ray moving origins and Grafana fields.
New regression tests isolate IMU prediction without LiDAR correction for several
stationary acceleration scales and initial orientations, and verify that the
normalization preserves genuine translation and deskew. Synchronizer tests cover
the detailed gap/bracket diagnostics and capacity eviction accounting.
The model refactor tests additionally verify independent IMU/ground/map topic
types, host timing metadata, unchanged LPC1 bytes and reading older tile/LOD
records whose obsolete sensor metadata slots contain non-zero values.
The dashboard JSON is checked for UID/channel/field agreement and layout overlap.
The new tests cover isolated IMU noise, 200-sample initialization at a low IMU
rate, motion preventing initialization, finite-noise stationary constraints,
exact five-second host-time recovery, worker-level recovery, retained-map/pose
continuity, refusal to insert unmatched geometry, and time-based quality loss.
Existing moving/rotating/overlapping deskew tests remain part of the full suite.

### Latest stationary RAW reproduction

The newest closed recording, `data/raw/20261006_192944.bin`, contains 264715 IMU
samples over 530.196 sensor-relative seconds, with a maximum gap of 14 ms, no
nonmonotonic IMU timestamps and no gaps above 100 ms. Its initial stationary mean
acceleration norm is 10.231 m/s²; initialization normalizes this to 9.81 m/s².
The older single-scan stationary predicate never passed in the first 614 audited
scans: median nearest-point separation was about 24 cm, despite a stationary
sensor. Before this repair, the first-minute estimator audit reached LOST at
17.329 seconds, with a maximum trusted position displacement of about 8.76 m.

With scan voxels 0.05 m, map voxels 0.20 m, match ratio 0.15 and the existing
quality limits unchanged, the repaired offline estimator produced:

| Recording / input scheduling | Examined scans | Accepted scans | LOST | Maximum pose displacement |
| --- | ---: | ---: | ---: | ---: |
| Latest recording, all 530.196 s | 12578 | 12563 | 0 | 0.0235 m |
| Previous `20261006_192542.bin`, entire recording | 1221 | 1206 | 0 | 0.0405 m |
| Latest recording, first 60 s, only every fifth scan | 284 | 279 | 0 | 0.0576 m |
| Latest recording, first 60 s, only every tenth scan | 142 | 138 | 0 | 0.0369 m |

Nonaccepted scans in these runs were initialization/seed accumulation, not
tracking failures. The latest full recording applied 2512 finite-noise stationary
velocity constraints. Maximum displacement is measured relative to the offline
zero-position initial anchor, not an external ground-truth measurement or proof
of absolute map accuracy. The preceding recording ends near
(-0.0049, -0.0004, -0.0082) m and the latest near (-0.0140, -0.0011, -0.0085) m.
The scheduling probes deliberately drop 80%/90% of scans while retaining
continuous IMU, not IMU extrapolation or missing-sample repair.

RAW does not retain host receipt timing, so the estimator replay cannot validate
live USB scheduling, actual topic drops or debugger pauses. Worker timeout,
stopped-IMU and five-second recovery behavior are tested separately. An actual
COM4 run with the extended `Reason` diagnostics is still needed; no physical
LiDAR was connected for these replay checks. No DeviceConfig quality limit,
Grafana JSON or plugin change is required by this repair.

These checks do **not** constitute a real Unitree motion trial, an ADLINK hardware
benchmark, a 24/7 soak test, or visual verification inside a running Grafana browser.
Validate IMU units, per-point timing, clock offsets and extrinsics with recorded
hardware trajectories before relying on the map's metric accuracy.

Sources: [FAST-LIO](https://github.com/hku-mars/FAST_LIO),
[ikd-Tree](https://github.com/hku-mars/ikd-Tree),
[Unitree SDK2](https://github.com/unitreerobotics/unilidar_sdk2).
