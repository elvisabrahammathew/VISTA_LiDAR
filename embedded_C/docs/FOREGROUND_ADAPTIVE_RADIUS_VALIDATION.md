# Foreground sensor-range-adaptive isolation filter

Date: 22 October 2026. Source repository updated directly; no delivery ZIP.

## Final behavior

Only the optional Foreground same-scan neighbor filter is changed. It still
requires two other distinct XYZ positions in the selected scan, excluding self
and duplicate coordinates. There is no Foreground multi-frame persistence gate,
new worker, second denoising algorithm, library dependency or motion classifier.

```text
ForegroundOutlierFilterEnabled: 1
ForegroundOutlierRadiusMeters: 0.15
ForegroundOutlierStartDistanceMeters: 1.0
ForegroundOutlierEndDistanceMeters: 5.0
ForegroundOutlierMaxRadiusMeters: 0.30
ForegroundOutlierMinNeighbors: 2
```

`t = clamp((range - startDistance) / (endDistance - startDistance), 0, 1)`;
`radius = baseRadius + (effectiveMaxRadius - baseRadius) * t`.
The effective maximum is max(baseRadius, configuredMaxRadius), preserving older
configs whose base is larger than the default cap. A max <= base selects fixed
radius. End must exceed start; distances and radii must be finite and within the
validated limits. Defaults and the user's DeviceConfig both use 1-5 m / 0.30 m.

Range is 3D Euclidean distance to the scan's localized sensor origin, not the map
origin. At <=1 / 2 / 3 / 4 / >=5 m the support radius is respectively
0.15 / 0.1875 / 0.225 / 0.2625 / 0.30 m. Missing/nonfinite origin falls back to
fixed base radius; it never guesses that the sensor is at the world origin.

The single spatial hash uses cell size max(baseRadius, effectiveMaxRadius/2)
and at most 125 neighboring cells per query. Existing 512-per-point and
1,000,000-per-sample candidate-comparison limits remain. Invalid or exhausted
queries remain unresolved, not certified as noise. Original input order,
intensity and shared scan data are preserved.

RoomMap subtraction code was compared byte-for-byte with the pre-change snapshot
and is unchanged. LIO/IMU, mapping confirmation/free-space logic, pose/freshness
gates, binary/JSON packet schemas and Grafana plugin/dashboard are not modified.
All previous DeviceConfig values remain, including the user's 200 ms cadence,
12,000-point sample limit and 0.15 m background tolerance. New diagnostic Reason
text includes adaptive-radius and missing-origin point counts. No COM port was
opened and no hardware initialization/restart was sent during these tests.

## Latest RAW and reproducibility limits

- File: `data/raw/20261022_142814.bin`, 98,888,132 bytes; unchanged.
- SHA-256: `B4F7D68CB34538129A6F5C28C9A85CBEFAA3EB17DF2EF7D37F9BBE3C0FC1B391`.
- Complete offline decode: 187,354 valid wire frames, 4,835 clouds, 200.798 s
  sensor-time span, zero CRC failures and zero detected timestamp resets.
- BIN contains sensor packets but not original host receipt timestamps. Filename
  and modification time cannot establish exact correspondence to the reported
  14:32-14:35 host-clock intervals. Replay reconstructs pacing from sensor time;
  original USB/Debug scheduling, packet losses before logging and real-time
  Grafana visibility are not reproduced exactly.

## A/B results at 2-4 m

The full-capture sensor-frame audit applies fixed and adaptive filters to the
same 967 selected scan samples at the unchanged 200 ms cadence. It does not
claim semantic or moving-person detection.

| Sensor-frame point decisions, range 2-4 m | Fixed 0.15 m | Adaptive 1-5 m |
| --- | ---: | ---: |
| Input | 587,809 | 587,809 |
| Retained | 533,389 | 563,906 |

Adaptive retains 30,517 more decisions (+5.72% relative to fixed retained
points). Windows and Linux produce identical counts in all three range bands.

An additional paired diagnostic observes the actual decoder -> Point-LIO ->
world cleaning -> RoomMap pipeline. Each pair uses the identical cleaned scan,
sensor origin and immutable, causally older confirmed-background snapshot.
Both alternatives run the unchanged RoomMap subtraction engine with equal
cache/read/comparison budgets. This observer adds test-only CPU/I/O work.

Excluding the first 30 s to reduce initial-map bootstrap dominance (not claiming
that the map is then complete):

| Paired world scans, range 2-4 m | Fixed | Adaptive |
| --- | ---: | ---: |
| Retained before map subtraction (806 pairs) | 311,153 | 341,651 |
| Foreground points after map subtraction | 3,090 | 3,433 |
| Nonempty Foreground scans | 222 | 268 |
| Fully resolved pairs only: Foreground points (643 pairs) | 2,568 | 2,818 |
| Fully resolved pairs only: nonempty scans | 173 | 216 |

Across the 806 pairs, 50 formerly empty scans gain Foreground; four lose it under
the unchanged conservative map-query budgets. Wider neighborhoods retain more
points and may warm/query additional map regions; unknown counts are therefore
not identical. Restricting to the 643 pairs with zero unresolved map points on
both sides removes that ambiguity: 43 formerly empty scans gain Foreground,
with no fixed-nonempty scan lost. Query-unknown totals in analysis.json apply
to all scan ranges, not just 2-4 m.

These are point/frame counts, not counts of unique people or detected objects.
Visual snapshots and range curves were inspected. Foreground at 2-4 m is present,
but the exact reported human crossings cannot be certified without host-time
alignment and reference labels/video. Unmapped static surfaces and retained
noise can also appear; adaptive radius does not solve RoomMap absorption,
insufficient illumination/occlusion or localization faults.

## Full-worker replay and performance

Windows Release replay uses 200.5 s of LiDAR plus a short IMU-only closing tail
from the capture, avoiding a manufactured missing-right-IMU-bracket at cutoff.
Real decoder, LIO, preprocessing, RoomMap, Gaussian, Foreground and a loopback
WebSocket client run; physical drivers and Grafana HTTP are disabled. Outputs
are separate diagnostic files, never the user's original map.

- 4,828 decoded clouds, 100,296 IMU messages, no decode/LIO input drops, missing
  measurement times, mixed-epoch clouds, LOST heartbeats or reinitializations.
- 938 production Foreground queries; 795 nonempty frames delivered over socket;
  1,031 binary deliveries including empty/heartbeat frames; zero malformed
  packets, server failures or Foreground query failures.
- No scan-filter unresolved decisions in this replay. 56,622 production
  map-query points remain conservatively unresolved; wider radius is not a
  cache-budget fix. Final map: 38,313 points. Gaussian: 55 completed revisions,
  zero failed revisions. Final map exported normally.
- The final `no fresh localized scan` heartbeat occurs after artificial EOF
  while workers drain; it is not a recorded LOST event.

Paired Release filter-only median wall times: fixed 0.895 ms, adaptive 1.254 ms;
p95 1.987 / 2.999 ms. Complete production Foreground queries including map I/O:
6.08 s total / 938 queries (about 6.48 ms mean), 54.03 ms maximum. These are
shared-host Windows timings, not ADLINK CPU utilization or latency guarantees.
The wider search adds work but does not add an unbounded search or another pass
in the production pipeline.

## Regression verification

Eight new tests cover linear/configurable range limits, sensor-origin translation
and rotation, near/far support, radius cap, missing/nonfinite origin, config
validation, fixed-radius compatibility, duplicate/work caps, actual-worker
origin propagation and immutable shared input.

- Windows x64 VS2019 Debug: 319/319 passed (187.44 s).
- Windows x64 VS2019 Release: 319/319 passed (70.55 s).
- Linux x86_64 Debug: 319/319 passed (61.01 s).
- Linux ARM64 Release / QEMU cortex-a57: 319/319 passed (585.93 s).
- Address/UndefinedBehavior sanitizer + exhaustive oracle: 3,000 randomized scans,
  300,047 decisions, zero mismatches or sanitizer errors; all neighbor thresholds
  1..16, duplicate coordinates, varying origins, start/end/cap values and fixed mode.
- Staged/deployed equality and unchanged subtraction/config checks: passed.
- Git diff whitespace check: passed. No plugin/dashboard rebuild is needed.

Detailed logs, CSVs, analysis.json, range-comparison.png and scan-snapshots.png
are retained in this task's `diagnostics/foreground_adaptive_20261022` directory.
No ZIP or installed Grafana update was created for this change.
