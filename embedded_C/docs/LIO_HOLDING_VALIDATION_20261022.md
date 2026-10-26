# LIO HOLDING for insufficient geometry

Date: 2026-10-22. This change is in the ROS-free Point-LIO adapter, not in the
vendored library. No sensor command, physical serial connection, new worker,
camera/clustering algorithm or scan-voxel/quality-threshold change is introduced.

## Contract

Existing localization wire codes 0–4 remain unchanged. Code **5 = HOLDING** means
the coordinates/orientation shown are the **last accepted pose**, not a verified
current pose. `pose_valid` is false and no world cloud reaches the local map,
RoomMap or Foreground from a held/recovery-confirmation scan. RoomMap/Gaussian
retain existing geometry. Live acquisition and health telemetry remain independent.

After a valid pose has been established, fewer than 100 selected scan voxels,
insufficient correspondences, or degenerate constraints enter HOLDING rather
than the timed DEGRADED → LOST → IMU-initialization cycle. Before the first
accepted pose the existing startup behavior remains unchanged.

IMU samples are still validated and internally integrated once at their actual
times. The synchronization horizon advances on sparse scans, so a long hold
does not manufacture an IMU-history gap or a propagation-backlog fault. IMU
predictions and their growing uncertainty are not certified as world poses.
The cached maps and last accepted pose do not follow this unobserved drift.
Overlapping scan recovery never moves the integrated IMU horizon backwards.
Freshly qualified IMU calibration after a real epoch restart also survives
subsequent HOLDING; recovery does not restore pre-reset bias values.

When sufficient geometry returns, the old accepted pose/calibration seeds a
local registration hypothesis. It uses zero candidate velocity and an extra
0.25 on each pose-covariance diagonal; this is a hypothesis, not a declaration
of stationarity or an accepted uncertainty reset. No new world origin/map is
seeded. The existing correspondence, information-eigenvalue, ratio, residual,
covariance and pose-jump limits remain enforced. Three consecutive qualifying
scans against the retained local map must also agree within 0.25 m / 5 degrees
(or the configured step bounds if smaller). Only the third may resume mapping.

Non-finite states, bad IMU, broken temporal coverage and clock/epoch faults are
not converted into valid/HOLDING poses. Existing genuine-fault loss/recovery
remains. Residual/ratio/covariance/pose-jump failures still use their existing
failure policy. Large real movement or scene ambiguity may prevent local
reacquisition; this is not global relocalization. No moving-map accuracy promise
follows from the absence of LOST.

## Grafana

Reimport the existing `dashboard/VISTA_Live_Dashboard_V5_Gaussian_Map.json`,
overwriting UID `vistalive`, to show yellow HOLDING for state 5. No plugin build
or reinstall is required. An old dashboard may show the numeric value 5 instead.
Other state codes, metrics, socket formats and the 5-second map cadence are
unchanged. Foreground does not subtract using an unverified held pose, even if
an upstream producer accidentally sets `pose_valid=true` while state is HOLDING.

## Validation

Five new engine/timer/serialization regressions test:

- unchanged wire codes and no five-second LOST timer while HOLDING;
- a 12-second sparse interval preserving pose/map, advancing IMU exactly once,
  and requiring three matches to resume;
- actual 0.20 m sensor translation during missing geometry: recovery must find
  the changed pose rather than freeze the last displayed coordinates;
- corrupt IMU, clock rewind and explicit epoch initialization remain distinct;
- an established room becoming a single plane holds rather than publishing it.

The existing Foreground freshness test also rejects HOLDING, including a malformed
`pose_valid=true` message. All older moving/rotating, duplicate/reset, saturation,
map/ray-origin and IMU-coverage tests remain enabled.

| Platform | Verification |
| --- | --- |
| Windows VS2019 Debug, RealSense enabled | 332/332 full-suite tests passed |
| Windows VS2019 Release, RealSense disabled | 332/332 full-suite tests passed |
| Linux x86_64 Debug | 332/332 full-suite tests passed |
| Linux ARM64 Release, QEMU Cortex-A57 | 332/332 full-suite tests passed; after final overlap/calibration guards, all 5 HOLDING-specific regressions passed again |

The extended HOLDING tests also exercise a 50 ms overlap without duplicate IMU
updates and a real epoch restart with a newly qualified gyro bias, followed by
sparse geometry and successful local recovery. The V5 JSON parses correctly,
retains codes 0–4, and maps 5 to yellow HOLDING.

The unchanged replay input
is `data/raw/20261022_163417.bin` (95,804,404 bytes), SHA-256
`83675CB4C66B5D65ED85B9F3BB41D10EFF6C1E6EFE178C14363525B031336DDD`.
Before this update, Debug and Release both reinitialized four times near the
end, with `insufficient scan geometry`; RAW had no CRC failure or clock rewind.
Sensor-time-paced replay cannot reproduce original USB/Visual Studio scheduling
because the BIN does not contain original host receipt times. EOF freshness
clears are not counted as a new in-capture localization failure.

Initial successful before/after replays through actual workers and loopback
WebSockets observed:

| Observation | Before Debug | HOLDING Debug | Before Release | HOLDING Release |
| --- | ---: | ---: | ---: | ---: |
| LOST status publications | 20 | 0 | 20 | 0 |
| Automatic reinitializations | 4 | 0 | 4 | 0 |
| LIO input drops | 894 | 854 | 4 | 0 |
| Accepted world scans | 2906 | 2939 | 3775 | 3781 |

These scheduling-dependent counters are not a CPU or accuracy benchmark. Debug
still drops scans. At approximately sensor second 154, insufficient geometry
now enters HOLDING. One interval lasted about 18.3 seconds before qualified
matches returned; no held-coordinate change or world-cloud output was observed
inside stable HOLDING intervals (excluding 200 ms publication-order boundaries).
Unit tests independently assert every provisional scan produces no world cloud
and cannot change the local map.

The final capture can still end in HOLDING: keeping an invalid pose is not proof
that the hidden scene became observable. In its final 20 seconds, 380/482 scans
had fewer than 100 selected 0.40 m voxels. Thresholds were not lowered to force
TRACKING. Small accepted-pose registration noise/drift outside HOLDING may remain.

An extra replay performed while other verification work was active experienced
one diagnostic-client socket read timeout and observer counter loss. That attempt
still had zero LIO LOST/reinitializations, but it is not counted as a clean socket
validation; its logs remain in `release-final`. Further quiet-run results are
recorded separately. No production socket timeout was changed to hide the result.

The final fresh-linked Release replay, run after all build/test jobs finished,
passed cleanly: 4,678 decoded clouds, 97,157 IMU samples, 3,781 accepted world
scans, zero LIO drops, LOST publications, reinitializations, CRC/mixed-epoch
errors, query/prefetch failures or WebSocket decoding errors. Held coordinates
were unchanged within each HOLDING interval; no world output appeared inside
stable HOLDING. The capture ended in invalid HOLDING as expected, rather than
being forcibly marked TRACKING.

The previous stable capture `20261022_151923.bin` also passed a final Release
control: 1,900 decoded clouds, 39,536 IMUs, all 1,882 initialized world scans,
zero LIO drops, LOST/reinitializations, query failures or socket decoding errors.
No HOLDING was needed during its input window. Expected EOF freshness clears
are outside the input-window interpretation. This is a regression check, not a
claim that every future real sensor scene will remain observable.

`DeviceConfig.txt` is byte-identical to its pre-change snapshot. The update does
not lower 0.40 m scan voxels, the 100-feature gate, 0.5 m minimum range or any
matching/IMU acceptance threshold. Only the state/recovery policy and V5 label
change. No plugin artifacts or ZIP were generated.
