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
Read -> RAW -> decoder -> pointcloud/decoded -> independent live filters -> :8765
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
```

The public topics, localization state codes, Grafana field names, LPC1 WebSocket
format, PCD and room-map tile formats are unchanged. No Grafana plugin or JSON
reimport is required solely for the backend migration. No detection/tracking is
added. Live sensor-frame rendering remains independent of localization validity.
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
reordering, clock resets or a second active producer are not silently hidden.
RoomMap is held and live :8765 continues while localization is invalid.

## Build and validation

Use existing Windows/Linux CMake presets or the legacy Adlink build script.
`cmake/point_lio.cmake` creates `vista_point_lio`; no initialized FAST-LIO checkout
is needed. Eigen and Boost.Preprocessor submodules remain required. The portable
target uses standard C++17 threading wrappers and the existing GCC 7 filesystem
compatibility layer. Only numerical code is optimized in Windows Debug; use
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
