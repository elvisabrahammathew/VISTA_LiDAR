# Point-LIO migration — 18 October 2026

## What changed

The active C++ localization backend is now Unitree Point-LIO, pinned to
`18ed5976d8fab2bd8a5148c26a40692bd3c0dc91` in
`third_party/point_lio_unilidar`. No upstream submodule source was modified.
`cmake/point_lio.cmake` builds `vista_point_lio`; the FAST-LIO build adapter was
removed. The old FAST-LIO checkout is retained but not required or linked.
No changes were committed or staged by the migration.
The removed `cmake/fast_lio.cmake` remains recoverable from Git; no RAW/map data
or third-party checkout was deleted.

This is a ROS-free adaptation of the real 30-DOF IMU-as-output model/filter,
not the former FAST-LIO2 engine renamed. The generated model declarations,
output dynamics and Jacobian are extracted from upstream Estimator sources.
The adapter visits actual point and IMU measurement times, masks saturated IMU
axes and preserves point acquisition-time world ray origins. It does not use the
firmware quaternion as an attitude or position ground truth.

The live sensor-frame point cloud remains independent of localization. RoomMap
receives accepted world clouds only and retains the floor. Topic types, Grafana
fields, WebSocket ports/formats and PCD format remain compatible. No dashboard
JSON or plugin reinstall is required for this migration.

## Configuration

Keep `LioEnabled: 1`. The console identifies
`Point-LIO (Unitree IMU-as-output)` at startup. Rebuild normally in Visual Studio
2019; CMake updates the numerical target and copies DeviceConfig beside the EXE.

Internal matching/map voxels now follow Unitree's L2 launch preset:

```text
LioScanVoxelSizeMeters: 0.40
LioMapVoxelSizeMeters: 0.40
```

They remain separate settings and affect odometry only. The persistent RoomMap
still uses `RoomMapVoxelSizeMeters: 0.05`; full-resolution valid world output is
not replaced by the coarse internal matching cloud.

The two existing IMU covariance keys now mean IMU **measurement R**, not the
former FAST-LIO process Q. Separate process covariance, LiDAR measurement,
acceleration-unit norm, per-axis saturation and matching-plane settings are
documented in [the LIO guide](../4_Applications/mapping/lio/README.md) and commented
in DeviceConfig. Values follow the Unitree output-model defaults; do not square
covariances or silently rescale gyro/timestamps to force a fit.

After an established pose fails a quality check, three consecutive qualifying
scans against the unchanged trusted local map are required before RoomMap output
resumes. This is local recovery, not global relocalization or loop closure.

## Software validation

These are historical migration results, not the current suite size or supported
OS matrix. Ubuntu 18.04 builds were retired on 2026-10-21; current ADLINK
deployment uses the [Ubuntu 20.04 ARM64 guide](ADLINK_UBUNTU20_BUILD.md).
At migration time the full suite had 207 tests. Final 0.4/0.4 m default results:

| Platform | Compiler / build | Optional RealSense | Result |
|---|---|---|---|
| Windows x64 | Visual Studio 2019, Debug | On, static | 207/207; 57.53 s |
| Linux x86-64 / Ubuntu WSL | GCC 13.3, Debug | Off | 207/207; 23.20 s |
| Ubuntu 18.04 x86-64 userspace | GCC 7.5 / CMake 3.10.2, Release | Off | 207/207; 33.67 s |
| Ubuntu 18.04 ARM64 userspace | GCC 7.5 cross / CMake 3.10.2, Release; QEMU Cortex-A57 | Off | 207/207; 187.24 s |

Times include test execution under desktop load, not an Adlink performance
benchmark. An additional modern GCC 13 ARM64/QEMU run passed all 207 tests before
the final default-voxel adjustment. Tests cover:

- The 30-DOF Q/P ordering and R-vs-Q distinction.
- Multiple LiDAR-time updates per frame, unique left-bracket consumption,
  per-axis saturation, acceleration in g/m/s² and independent engine instances.
- Stationary, biased, translating, rotating and overlapping synthetic scans;
  per-point world ray origins and retained-map recovery.
- Existing worker lifecycle, RoomMap persistence/cache, live WebSockets,
  Grafana serialization, device decoding and filesystem compatibility.

One Windows full run encountered the previously observed temporary-directory
cleanup failure in `room_map_session_file_claims_suffixes_and_updates_only_its_own_file`.
The isolated test and subsequent full run passed. It was not suppressed and no
unrelated filesystem production code was changed to hide it.

ARM execution is through QEMU Cortex-A57, **not physical Adlink hardware**.
No COM port was opened, no firmware was flashed and no running Grafana service
was reconfigured. Use Release and validate live scheduling on the actual device.

## RAW replay results and important limitation

These are production-engine offline replays with actual recorded points and IMU,
not synthetic motion and not independent ground truth. The complete replay
preserves sensor-relative timing; RAW BIN does not retain original host arrival
timestamps. Displacement/angle below are relative to the first accepted pose.

| Recording | Examined scans | Tracking scans | Final sensor time | Maximum displacement | Maximum attitude change |
|---|---:|---:|---:|---:|---:|
| 20261018_203122.bin | 2,985 | 2,972 | 130.060 s | 0.261 m | 55.25° |
| 20261016_133012.bin | 13,853 | 13,840 | 583.977 s | 0.326 m | 57.14° |

Both selected-config runs contain 13 initializing/seed scans and no subsequent
DEGRADED or LOST. One initial scan lacks a left IMU bracket and is skipped before
engine examination. Largest successive accepted translation steps are about
4.69 cm and 4.11 cm, respectively. The original run performs 1,918,214 LiDAR and
291,361 IMU measurement updates, confirming point-time rather than scan-end-only
processing.

The 0.05 m scan / 0.20 m map configuration inherited from the old engine was also
tested on the latest replay capture. It produced approximately **25.90 m** maximum
displacement despite remaining TRACKING. It is not the shipped configuration.
This is direct evidence that passing local gates or avoiding LOST alone does not
validate spatial accuracy, and that old fine-resolution tuning cannot be carried
over without testing.

**The selected configuration still fails the operator's stationary 1–2 cm
requirement. The backend migration must not be represented as a resolved drift
or calibration issue.** The recorded IMU/geometry inconsistency identified in the
stationarity audit remains unresolved. A fresh direct acquisition with host
reception timing, an official SDK comparison and controlled known-angle motion
are needed before calibrating or trusting real robot localization.

The offline harness loads the whole RAW file into RAM. Its peak RSS and desktop
replay speed are not estimates of deployed Adlink memory use or control latency.
The separate final 1x worker-pipeline replay checks decoder → Point-LIO →
preprocessing → RoomMap and the independent live branch:

- RAW packets sent: 117,327; decoded clouds: 2,986.
- Accepted world clouds published/observed: 2,972 / 2,972.
- Independent live clouds observed: 2,986.
- Reported LIO drops: 1; RoomMap input drops: 0.
- RoomMap received: 2,972; final exported PCD: 61,235 points, `x y z intensity`.
- Maximum world-origin displacement: 0.2623 m; largest successive step: 0.04692 m.
- No decoder/LIO/preprocessing/map/live worker errors.

Creating a PCD does not certify that its geometry is correct. No real browser
rendering or physical LiDAR/Adlink motion trial was performed in this migration.

## References

- [Unitree Point-LIO](https://github.com/unitreerobotics/point_lio_unilidar).
- [Point-LIO paper](https://doi.org/10.1002/aisy.202200459).
- [Unitree SDK2](https://github.com/unitreerobotics/unilidar_sdk2).

Derived code remains subject to GPL-2.0; `core/COPYING` matches the Point-LIO
submodule license. Generated IKFoM headers retain their upstream notices.
