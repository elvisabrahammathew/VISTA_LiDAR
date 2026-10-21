# Point-LIO moving mapping (ROS-free)

The active backend is **Point-LIO, Unitree IMU-as-output**, from
`third_party/point_lio_unilidar` at commit
`18ed5976d8fab2bd8a5148c26a40692bd3c0dc91`.
FAST-LIO2 is no longer built, included or linked. Its old checkout is not deleted.

This is a portable adaptation, **not** the unmodified ROS node. CMake extracts the
upstream `state_output`, `get_f_output`, `df_dx_output` and builds the upstream
Point-LIO two-measurement EKF. It does not rename the former 23-DOF FAST-LIO model.
The state has 30 tangent dimensions: position, attitude, fixed extrinsic R/T,
velocity, angular velocity, specific acceleration, gravity, gyro bias and
accelerometer bias. IMU is an **output measurement**, rather than a direct motion
input. LiDAR observations update the filter at retained point acquisition times.

## Pipeline and compatibility

```text
Read -> RAW -> decoder -> pointcloud/decoded -> independent live health filters
                   |             |
                   +-> lidar/imu +-> Point-LIO synchronization
External IMU (alternative) -> lidar/imu       |
                         stationary initialization
                    time-ordered IMU / point updates
                  valid pose + per-point world ray origins
                                      |
                     localization/pointcloud_world
                                      |
                       world preprocessing (keep floor)
                            /                   \
                  disk-backed RoomMap       existing ground processing
                         :8766              optional processed PCD logger
                           |
                    Gaussian surface map :8767
```

The public topics, localization state codes, Grafana field names, LPC1 WebSocket
format, PCD and room-map tile formats are unchanged. No Grafana plugin or JSON
reimport is required solely for clock recovery. No detection/tracking is
added. Live sensor health remains independent of localization validity; live
sensor-point WebSocket publishing is retired (RoomMap/Gaussian remain enabled).
`LioEnabled: 0` still selects fixed-mount mapping.

Point-time updates are internal. The existing message bus still publishes
accepted world clouds and localization status through its scan/worker interface,
not a new 4–8 kHz robot-control topic. The paper's odometry rates are not a
guarantee of this adapter's public output rate or Adlink latency.

## Numerical port and intentional differences

- The upstream model/filter and tree are copied/extracted **only into the build
  directory**. ROS callbacks, PCL storage, publisher globals, Python plotting,
  OpenMP and vendor Linux binaries are not linked. The adapter owns per-instance
  filter/map state; multiple engines never share upstream ROS global variables.
- State propagation visits actual IMU and point times, covariance propagation
  occurs at IMU times (`prop_at_freq_of_imu` behavior). Already-consumed left IMU
  brackets are not applied twice. Per-axis saturated IMU measurements are masked
  as in `h_model_IMU_output`; an unrelated unsaturated axis remains usable.
- Spatial matching uses retained measured points, not PCL-generated centroid
  timestamps. The scan budget is spread across the full temporally ordered scan.
  Each retained point is updated separately; full-resolution valid output points
  use their acquisition-time pose. There is no scan-end FAST-LIO deskew/update.
- Five-neighbor planes use PCA with non-collinear support, including planes
  through the world origin. The distance check and range-dependent match rule
  follow the upstream correspondence design. Extrinsics remain fixed.
- Mount XYZ/RPY is kept as the initial **LiDAR world pose**, not overwritten by
  automatic gravity alignment. Gravity is initialized in that frame. Initial gyro
  bias is estimated while stationary. Measurement acceleration normalization uses
  explicit `acc_norm`, **not** the previous FAST-LIO startup-mean scale.
- The former stationary-velocity/ZUPT shortcut is removed. Stationary and moving
  behavior must be supported by Point-LIO measurements, not a forced fixed pose.
- Local map retention is bounded by existing radius/point budgets. Tree rebuilds
  are synchronous in the owning worker; no hidden pthread startup messages or
  million-entry asynchronous queues are enabled.

These adaptations and safety gates mean stock ROS-node benchmark results must
not be claimed as performance/accuracy results of VISTA.

## Configuration migration

Keep `LioEnabled: 1`, the single `lidar/imu` topic, mounting/extrinsics and existing
timing/map-budget keys. The backend is always Point-LIO; there is no FAST-LIO
fallback or algorithm-selection switch.

| TXT setting | Point-LIO role | Default |
|---|---|---:|
| LioScanVoxelSizeMeters | Internal matching voxel, L2 launch preset | 0.4 m |
| LioMapVoxelSizeMeters | Internal odometry-map voxel, L2 launch preset | 0.4 m |
| LioImuAccelerationCovariance | IMU acceleration measurement R | 0.1 |
| LioImuGyroscopeCovariance | IMU angular-rate measurement R | 0.1 |
| LioImuAccelerationBiasCovariance | Accelerometer bias process Q | 0.0001 |
| LioImuGyroscopeBiasCovariance | Gyro bias process Q | 0.0001 |
| LioVelocityProcessCovariance | Velocity process Q (`vel_cov`) | 20 |
| LioAccelerationProcessCovariance | Acceleration-state Q (`acc_cov_output`) | 500 |
| LioAngularVelocityProcessCovariance | Angular-rate-state Q (`gyr_cov_output`) | 1000 |
| LioLidarMeasurementCovariance | Point-to-plane measurement variance | 0.01 |
| LioImuAccelerationNorm | Acceleration unit norm (`acc_norm`) | 9.81 |
| LioImuAccelerationSaturation | Saturation per raw acceleration axis | 30 |
| LioImuGyroscopeSaturation | Saturation per raw gyro axis | 35 |
| LioMinRangeMeters | Minimum range for matching only | 0.5 |
| LioPlaneThresholdMeters | Maximum neighbor-to-fitted-plane error | 0.1 |

All covariance values are diagonals, **not standard deviations**. Do not square
them. The two pre-existing IMU covariance keys now control measurement R, not the
former FAST-LIO process Q. New defaults follow Unitree's L2 output-model settings
and `parameters.cpp`'s velocity default. Older config files still parse and receive
these defaults, but old tuned noise values should be reviewed for their new role.

For acceleration in m/s², use norm 9.81; for acceleration in g, use norm 1 and
adjust saturation to the corresponding raw units. Gyro is rad/s. The firmware
quaternion is not used as position/attitude ground truth or to auto-rescale gyro.

`LioScanVoxelSizeMeters`/`LioScanMaxPoints` bound matching work only. Internal
odometry map settings do not limit the separate persistent RoomMap. Valid near
points below the matching blind range can still be emitted to RoomMap.

## Timing, loss and recovery

### Lightweight IMU robustness and qualified rest updates

`LioRobustImuEnabled: 1` and `LioStationaryUpdateEnabled: 1` enable two bounded
layers inside the existing Point-LIO engine. Each can be set to 0 for A/B tests;
there is no additional worker, estimator, ICP pass, RAW rewrite, ROS dependency
or extra Grafana series. RAW and public IMU samples/timestamps are unchanged.

- One causal 15-sample, six-channel median/MAD detector inflates the existing
  per-axis measurement R for isolated outliers. It does not replace samples or
  delete timestamps. A coherent sustained step is released after two samples;
  alternating glitches are not mistaken for a sustained motion step. Existing
  saturation masks still apply. Observed IMU-window variance can increase R but
  cannot make it smaller than the configured measurement covariance.
- Initialization retains the original quiet-window path. A separate vibration
  qualification requires at least 0.70 s, five populated 150-ms blocks with
  consistent means, bounded raw RMS, and low mean angular rate. It is not merely
  a larger RMS threshold or smoothing data until it passes. Reason distinguishes
  `startup_mode=quiet`, `vibration-qualified`, and `waiting`. Stationary startup
  is still an assumption: inertial statistics alone cannot prove no motion.
- Rest evidence reuses the plane correspondences, Jacobians and 6-DOF information
  eigendecomposition ALREADY calculated by Point-LIO. A bounded 1.2-s window
  checks LiDAR displacement slope/scatter together with IMU consistency and
  estimated velocity. This guards against constant-velocity false detections.
- Qualified rest adds a velocity measurement through a rank-three Schmidt/Joseph
  EKF update. Pose/extrinsic gain rows are zero: it does not overwrite position,
  reset to Mount, rotate old map geometry or invalidate already-generated rays.
  This deliberately differs from hard-locking the device to make drift look zero.
- After local-map warmup, the stationary layer also gates odometry-cache
  insertion on at least 5 cm translation or 0.5 degree rotation. This avoids
  repeatedly feeding sub-voxel registration jitter into its reference. The
  separate persistent **RoomMap still receives every accepted full-resolution
  world cloud**. The gate is disabled with `LioStationaryUpdateEnabled: 0`.

The rolling IMU statistics are shared by initialization, R adjustment and rest
qualification. Once initialized, a large initialization sample count does not
retain a long stale statistics window. Rest history holds at most 128 entries;
the outlier detector has fixed storage. No second registration/search is added.

Temporal-epoch initialization clears both new histories. A tentative rejected
scan restores the outlier-detector state along with the EKF before any IMU-only
replay, so old samples do not acquire a different weight after rollback.

Diagnostics reuse the existing Reason field: `imu_downweighted_axes`,
`zupt_updates`, `rest_confirmed`, `rest_gate`, `rest_lidar_m`, `rest_imu_error`.
No dashboard/plugin update is needed. These layers do not guarantee zero drift,
globally relocalize a lost sensor, or replace extrinsic/time/IMU calibration.

Initialization requires the configured number of unique IMU samples plus at least
0.5 seconds of a stable bounded IMU window. Motion prevents initialization; the
Reason column reports sample count and IMU statistics. Mount is an initial pose;
LiDAR-to-IMU translation/rotation is a separate calibration.

Host `steady_clock` remains the common monotonic domain. RAW reception time is
preserved. Unitree's integrated LiDAR/IMU share a clock adapter; external IMU
drivers use the same fixed topic, with their own source clock anchored to host
time. The adapter preserves sample deltas to avoid USB-batch compression and
cannot correct an unknown gross sensor clock-rate error. RAW BIN does not retain
original per-packet host arrival times. A RAW-only replay is not validation of
live serial timing or a way to resolve the recorded gyro/quaternion inconsistency.

Only finite, geometrically supported, quality-gated poses reach RoomMap.
Rejected tentative LiDAR corrections are rolled back before continuing IMU-only
propagation. Neither trusted map is updated by a rejected scan. After a failure
of an established pose, **three consecutive qualifying scans against the held
local map** are required before map publication resumes. This is local recovery,
not global relocalization. Large movement during loss may remain unrecoverable.

Persistent quality rejection becomes LOST after two acquisition seconds; unsafe
timing can enter LOST immediately. Five continuous host seconds in LOST trigger
IMU initialization while keeping the accepted world anchor/map. Confirmed recent
bit-identical duplicate IMUs are discarded; conflicting samples, unknown
reordering or a second active producer are not silently hidden.
RoomMap is held and live sensor health continues while localization is invalid.

### Sensor-clock epoch recovery

A backward device timestamp step greater than 500 ms starts an explicit new
measurement-clock epoch. This is a reset of the sensor-to-host time adapter,
**not** a reset of Windows/Linux clocks or a hardware Restart command. Small
interleaving stays in the same epoch. Invalid/backward host monotonic time is
still a latched fault and cannot be repaired by a sensor-time reset.

The decoder checks every RAW packet and discards partial Unitree scan assembly
at the boundary. LiDAR and IMU envelopes carry independent timing metadata
(`measurement_clock_epoch`, `measurement_epoch_started_ns`) without depending
on each other's message definitions. Integrated streams share one boundary;
external producers must publish their own adapter's epoch and host boundary.

Either stream's new boundary immediately resets LIO temporal state, including
its synchronizer, pending scans, early IMUs, duplicate history and relative host
reference. The same integrated boundary is handled once, even if the two topic
queues deliver it at different times. Pre-boundary receipts and older stream
epochs are discarded. Independent external IMU clocks need not use the same
epoch counter as the LiDAR.

The accepted world anchor and local map remain held. Fresh stationary IMU
initialization and three qualifying local-map matches are required before world
cloud publication resumes. If initialization quality is insufficient or the
device moved beyond local matching range, mapping stays paused: recovery is not
a guarantee of global relocalization or a license to seed a different room.
Clock recovery itself does not relax quality gates. The optional vibration
qualification above is a separate initialization path; geometry gates remain.

The existing Reason field adds `clock_epoch_recoveries` and
`stale_epoch_messages` alongside `reinitializations`. No dashboard/plugin or
DeviceConfig change is required. RAW-only replay reconstructs host receipt
timing; it does not reproduce the original USB delays.

## Build and validation

### Processing-cost optimizations (21 October 2026)

Unitree frame CRC32 uses the same reflected IEEE polynomial and wire coverage,
with a constexpr lookup table. The reader certifies immutable owned RAW bytes;
the decoder reuses that private certification. Constructing a new RAW packet
(including replay) does not certify it: the first decoder access validates it.
Standalone byte-vector protocol entry points still validate each input. Semantic
layout, point count, IMU finiteness and timestamp checks remain independent.
The transport scanner advances an offset and compacts only before another read.
Point rows append directly to scan storage, without a temporary row vector.

The LIO worker borrows its immutable bus cloud and synchronized IMU batch for a
synchronous engine call. The separate point-time origin does not relabel the
source cloud. Per-engine matching scratch is reused; already ordered points skip
sorting, while unordered input retains stable sorting and equal-time ordering.
World output is built directly rather than copying then clearing the input.
Latest-time pose lookups reuse the last history knot; overlapping historical
points retain the binary-search path. There is no change to EKF update order,
point budgets, voxel sizes, quality/recovery gates, rest/noise layers or map
resolution. Rollback still restores complete filter/history/robustifier state.
These are execution-cost changes, not a remedy or guarantee for position drift.

Use existing Windows/Linux CMake presets (CMake 3.20+) or the
[Ubuntu 20.04 ARM64 build script](../../../docs/ADLINK_UBUNTU20_BUILD.md)
(CMake 3.16+).
`cmake/point_lio.cmake` creates `vista_point_lio`; no initialized FAST-LIO checkout
is needed. Eigen and Boost.Preprocessor submodules remain required. The portable
target uses standard C++17 threading wrappers and the shared filesystem wrapper;
no Ubuntu 18.04-specific build is required. Only numerical code is optimized in Windows Debug; use
Release for deployment and observe input drops/CPU load.

The suite includes point-time update counts, unique IMU consumption, 30-DOF
Q/P ordering, saturation masks, acceleration units, independent engines, held-map
recovery, stationary/moving/rotating synthetic rooms, overlapping scans and ray
origins. Worker, RoomMap, serialization and Grafana tests remain enabled. Actual
pass counts/platforms and RAW replay outcomes are documented in the
[migration validation report](../../../docs/POINT_LIO_MIGRATION.md), not the
former FAST-LIO test report.

There is no loop closure, global pose-graph optimization or automatic localization
against a loaded PCD. Passing gates or avoiding LOST is not proof of map accuracy.
The stationary RAW has apparent sensor motion and an IMU consistency anomaly;
switching algorithms does not certify those inputs or repair firmware calibration.

## License

Point-LIO/Unitree derived code is GPL-2.0; `core/COPYING` matches the upstream
LICENSE. Generated IKFoM headers retain their own notices. Eigen and
Boost.Preprocessor keep their submodule licenses. Review distribution obligations
before shipping binaries.

Sources: [Unitree Point-LIO](https://github.com/unitreerobotics/point_lio_unilidar),
[Point-LIO paper](https://doi.org/10.1002/aisy.202200459),
[Unitree SDK2](https://github.com/unitreerobotics/unilidar_sdk2).
