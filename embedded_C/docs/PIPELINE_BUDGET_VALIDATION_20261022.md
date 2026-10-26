# Pipeline budget, heartbeat and background-cache update

Date: 2026-10-22. Scope: the existing C++ application; no new runtime worker,
Grafana protocol, camera pipeline, clustering or tracking.

## What changed

1. The supplied DeviceConfig uses `RoomMapPublishIntervalMilliseconds: 5000`.
   This is checkpoint/view publication cadence, not a slower decoder or LIO
   cadence. RoomMap keeps its 100 ms observation interval and Foreground keeps
   its configured 200 ms sampling interval. Freeze/clean shutdown can still
   produce a final checkpoint/PCD. Periodic checkpoints are not full PCD exports.
2. Gaussian fitting replaces repeated neighborhood hash lookups with one bounded
   padded lookup table per active tile (at most 72 cubed pointers, approximately
   3 MiB on 64-bit systems). The neighborhood order and fitting math are unchanged.
   Disk records remain 41 bytes; batched record I/O and sequential reads remove
   redundant small operations. GSP1 encoding, point budgets and covariance math
   are unchanged. Changed tiles/halos and retained immutable views still work.
3. Foreground's 1-second localization heartbeat now has a 2.5-second receipt
   timeout. The LIO worker publishes actual state/pose-validity transitions as
   soon as it observes them. Explicit LOST/invalid/degraded status still clears
   Foreground. The 500 ms scan freshness and 0.01 square-metre variance gates
   remain unchanged. This is not a relaxation of LIO matching or IMU coverage.
4. Foreground opportunistically prefetches immutable background pages in its own
   idle time: at most 64 rotating probes, two reads per pass, four additional
   reads per sampling interval, within the existing point/tile cache limits.
   A soft 3 ms pass budget leaves 5 ms before the next query. A synchronous page
   read cannot be preempted, so these are not hard real-time guarantees.
   Query reads remain capped at eight. Unknown stays Unknown, never foreground.
   Deleted/replaced pages cannot mask new geometry. Snapshot/read errors clear
   only this optional branch, which can recover when a valid checkpoint arrives.
5. Foreground and Gaussian compile in a dedicated OBJECT target with optimized
   Debug code, following the existing numerical LIO-core precedent. Debug symbols,
   the Debug CRT and iterator ABI remain; local variables in these two files may
   be optimized out. Other worker/controller sources retain normal Debug checks.
   An OBJECT build target is not an additional runtime thread.

No map confirmation/free-space rule, scan isolation radius, neighbor threshold,
LIO numerical model, world-frame convention or detection decision was changed.
No hardware command was issued and no physical COM port was opened during tests.
Existing dirty repository changes were retained. No ZIP/plugin/dashboard upgrade
is needed: rebuild and restart the C++ application.

## Verification

| Check | Result |
| --- | --- |
| Windows VS2019 Debug, RealSense integration enabled | 327/327 full tests passed |
| Windows VS2019 Release, RealSense disabled | 327/327 full tests passed |
| Linux x86_64 Debug, RealSense disabled | 327/327 full tests passed |
| Linux ARM64 Release, QEMU Cortex-A57 | 326/326 full tests passed before the final snapshot-error guard; final rebuild passed all 45 Foreground and 11 Gaussian tests, including the new recovery case |
| ASan + UBSan on modified Foreground/Gaussian code | 54/54 tests passed, with leak checks enabled; real-PCD fixture also passed |

Eight added regressions cover bounded prefetch, cache page replacement/deletion,
deadlines/corruption/epoch reset, unchanged-page reuse, heartbeat jitter versus
real stale scans/invalid poses, immediate LIO fault publication, Gaussian bulk
records/sparse selection, and recovery from snapshot acquisition failure.

One Release run failed while removing an already-empty temporary test directory
in the existing session-file test. No stored PCD assertion failed. The directory
was empty on inspection; three focused reruns and the subsequent full suite all
passed without changing that test or storage code. The first failure log is kept.

## Gaussian geometry control

The same 35,375-point RoomMap PCD was processed by preserved pre-change Gaussian
source and the updated source, in Debug and Release. All complete GSP1 packets
have the same SHA-256:

`C7C25A94744C8996BB4DCC64AE2157388C4B597689F661E1BFEE0D97EF7A1ED5`

This checks centers, unsigned intensity, covariance, opacity and ordering, not
just point count. A separate ASan/UBSan run loaded, fitted and selected all
35,375 real-PCD points successfully. Fixed-map timings and final replay counters
are recorded in the accompanying diagnostics, not treated as target-device CPU
or hard latency guarantees. A final paired fixed-PCD measurement gave:

| Mode | Old fit | New fit | Old view selection | New view selection |
| --- | ---: | ---: | ---: | ---: |
| Windows Release | 665 ms | 470 ms | 36.3 ms | 16.4 ms |
| Windows Debug | 5258 ms | 1303 ms | 93.6 ms | 16.4 ms |

These are single paired host measurements, not repeated-trial confidence bounds.

## Replay data and interpretation

Input: `data/raw/20261022_151923.bin`, 39,005,484 bytes, SHA-256:

`EEAD60C831AF1A1D3628D03CED7AC6A6BBAC39160D264ED0D94BEB3DDC5C4C9F`

Sensor-time span is approximately 79.175 seconds. The replay sends original
validated wire frames through the actual decoder, Point-LIO, preprocessing,
RoomMap, Foreground and Gaussian workers, with loopback clients decoding LPC1
and GSP1. It runs once, paced by sensor-time deltas. LiDAR input stops at 78.9 s,
with a short real IMU-only closing tail to avoid an artificial final missing-right-
bracket failure. EOF freshness clears are expected and excluded from the input
window statistics. Original host receipt/USB scheduling was not logged in BIN;
this is not an exact reproduction of the original Visual Studio session.

After the build/test jobs finished, final Release and Debug replays ran
sequentially. Both decoded 1,900 clouds and 39,536 IMU samples with zero CRC
failures, source clock resets, mixed epochs, LOST heartbeats, query/prefetch
failures or socket decoding errors. Actual EOF freshness clears are expected.

| Final observation | Earlier Release | Final Release | Earlier Debug | Final Debug |
| --- | ---: | ---: | ---: | ---: |
| Accepted world scans | 1882 | 1882 | 1376 | 1616 |
| LIO input drops | 0 | 0 | 506 | 266 |
| Foreground query + prefetch wall time, full replay | 2.58 s | 1.69 s | 46.72 s | 21.55 s |
| Unknown / queried points, final 30 seconds | 9.25% | 3.36% | 14.49% | 3.90% |
| Heartbeat-timeout pauses, final 30 seconds | 3 | 0 | 9 | 0 |

Final Release delivered all 390 active query frames through the socket (including
valid empty geometry frames); final Debug delivered all 382. Release had no
Foreground pause in the last 30 seconds. Debug had three causal-checkpoint waits,
not heartbeat expiries. Gaussian output gaps had medians of 5.23 s / 4.96 s and
maxima of 5.67 s / 6.74 s, Release / Debug respectively. Lag from a matching
completed map status to Gaussian publication had medians of 0.64 s / 2.50 s.

Debug still drops 266 LIO scans: do not describe this patch as eliminating Debug
overload. Earlier post-change experiments recorded 845 and 1,125 drops while
other background work was present; those logs remain and are not used as the
final throughput comparison. These are scheduling-dependent observed runs,
not repeated-trial normalized CPU benchmarks. Query plus prefetch wall time is
not whole-process CPU usage, and prefetch is additional work, not free I/O.
More complete mapping and different accepted timing mean final PCD geometry
need not be byte-identical; the fixed-PCD Gaussian equality control above is the
geometry-preservation test for the optimized fitter.

Five seconds is an update-cadence target, not maximum end-to-end latency:
checkpoint completion, fitting and transmission add time, especially on Debug
or slower storage. Foreground remains a single selected scan per interval,
not a multi-scan accumulation or a motion/person detector. Better cache coverage
and more complete background can legitimately reduce foreground point counts.
These unlabelled captures cannot establish person-detection precision/recall.

Use Release or RelWithDebInfo for live throughput evaluation. Physical ADLINK
ARM64 performance, real USB timing and browser/GPU rendering still require an
on-device run; QEMU and loopback socket checks do not establish those properties.

Detailed harnesses, packets, config snapshots, raw counters, test logs and the
before/stage snapshots are retained in this task's
`diagnostics/pipeline_budget_20261022` directory.
