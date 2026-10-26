# Live foreground from confirmed RoomMap

The panel displays current LiDAR points farther than a configurable radius from
confirmed map geometry. It is **not** a motion classifier, semantic detector,
cluster tracker, or camera pipeline. New stationary objects/unmapped walls can
appear. The former moving-point algorithm is no longer run.

## Flow and lifecycle

```text
LiDAR + IMU -> Point-LIO -> accepted world scan -> preprocessing
                                                      |
                          +---------------------------+-------------+
                          |                                         |
                    latest-scan slot                          RoomMap owner
                          |                                  /      |      \
                          |                     map/PCD :8766  checkpoint  Gaussian :8767
                          |                                         |
                          +--> foreground-filter <------------------+
                                every 250 ms
                                     |
                         full confirmed-map distance query
                                     |
                         immediate WebSocket :8768
```

- An independent worker retains only the latest accepted world scan, samples it
  every `ForegroundPublishIntervalMilliseconds` (250 ms by default, up to 4 Hz),
  and reads the newest completed immutable RoomMap checkpoint. Decoding, LIO and
  RoomMap integration continue at their existing rates. There is no per-scan
  foreground work, mapper wait, forced flush, or catch-up burst.
- Each checkpoint carries its host monotonic completion time, unchanged by
  heartbeats. The sampled scan's original host receipt must be strictly newer;
  otherwise the worker waits for a newer scan. This prevents subtraction against
  a background that may already contain that scan or future observations.
- The worker never processes an identical source/sequence/epoch/receipt twice.
  New results go to the socket immediately, without a second cadence delay.
  Pauses/staleness clear the display immediately; unchanged active geometry is
  not repeatedly transmitted. A slow or invalid upstream cannot guarantee 4 Hz.
- Background changes follow `RoomMapPublishIntervalMilliseconds`. Newly confirmed
  points can remain foreground until the next checkpoint, matching map publication
  cadence; foreground updates can be faster.
- Tentative voxels do not mask new points. A stationary object disappears from
  foreground once confirmed and checkpointed.
- Moving beyond the tolerance reveals its new returns. Parts overlapping old
  geometry can remain masked. Old map points disappear only after repeated
  measured free space, not mere absence or occlusion. Stopping can make an object
  background again.
- Foreground never changes map insertion/deletion, LIO, or acquisition decisions.
- Before subtraction, a same-scan radius filter removes spatially isolated
  points from the worker-local bounded sample. By default each retained point
  needs two distinct neighboring positions within 0.15 m, excluding itself;
  duplicate coordinates cannot manufacture support. Checks use the original
  sample, not sequentially eroded output. No new point is required to recur in
  earlier/later frames. RoomMap still receives the unchanged full scan and keeps
  its existing repeated-observation confirmation/free-space rules.
- Loaded PCDs are full confirmed read-only references from the first observation,
  never overwritten. Frozen maps still support subtraction but do not adapt to
  new stationary objects until mapping is resumed/rebuilt.

## Full background, bounded resources

Session tile/LOD/surface and Gaussian files are temporary working storage. On clean shutdown,
after the final PCD export succeeds, only that session's owned cache directory
is deleted. Deletion waits for all RoomMap, foreground and Gaussian readers to
release their disk/surface references. FROZEN still needs its cache while the
application runs. Loaded-reference caches are also temporary; the input PCD is
never overwritten or deleted. Failed exports/unhandled failures retain cache
evidence; forced termination/power loss can leave folders behind. Older session
folders are not automatically swept. No-data sessions create no empty PCD.
Use the application's normal stop (Ctrl+C) and allow shutdown to finish. Visual
Studio Stop Debugging/task-manager termination/power loss can kill the process
before final export or cleanup; the application cannot guarantee either then.

The worker reuses full-resolution immutable map-region pages, also used by
Gaussian. It does not compare against renderer LOD or treat mapper RAM misses as
new objects. Foreground requests these pages even when Gaussian is disabled.

A separate LRU spatial index shares RoomMap's cache point/tile limits. Only tiles
needed by current points and neighboring tiles within the distance radius are
queried. At most eight cold region reads and one cache-budget worth of source
points are read per observation. The input/output sample and queues are bounded.
Distance comparisons are capped at 512 per point and 1000000 per observation;
exhausting those budgets also yields unresolved rather than false foreground.
The separate scan-isolation grid is rebuilt for each selected sample, with at
most `ForegroundMaxPoints` entries, up to 125 neighboring cells per point, and its own
512-per-point / 1000000-per-sample comparison cap. Adaptive filtering uses one
grid with cell size max(baseRadius, effectiveMaxRadius / 2), not stacked filters.
A budget-exhausted/invalid
query is omitted and reported as unresolved, not falsely certified as noise.
There is no cross-frame point history, second worker, or PCL dependency.
Unresolved/unread/oversized regions are omitted, NOT mislabeled foreground. This
can temporarily hide objects until cache warming or a larger budget; the complete
map stays on disk. Deleted/replaced versions invalidate stale index entries.

During otherwise idle time, the same worker can prefetch current immutable pages
using at most 64 rotating probes from the latest scan. Each pass loads at most
two pages, with four additional reads allowed per sampling interval. Prefetch
uses the existing point/tile cache limits, a soft 3 ms pass budget, and leaves
5 ms before the next query. A synchronous page read cannot be interrupted, so
this is not a hard real-time deadline. Prefetch does not classify or publish
points, advance the committed scan clock, force a checkpoint, or change LIO.
It retains only unchanged page versions; replacement/deletion invalidates old
geometry. It is opportunistic, not a guarantee that every page will be warm.

The optional worker never evicts/dirties/writes mapper RAM. It adds CPU and read
I/O and may require immutable checkpoint pages when Gaussian is off; it is not
zero-cost. No new library, whole-map RAM copy, or second motion algorithm is used.

## Safety and limitations

With LIO enabled, recent valid TRACKING status and position variance below
0.01 m² are required. Invalid/degraded/LOST pose or scans older than 500 ms clear
the output. Freshness uses host steady-clock receipts, not sensor calendar time.
HOLDING is also invalid for foreground subtraction: the displayed pose is the
last accepted pose, not proof that a fresh scan is aligned. Existing RoomMap and
Gaussian geometry remain available, but held/predicted points are never inserted
into either map or published as current world foreground. The existing V5 JSON
adds HOLDING (code 5); reimport it for the label. No plugin rebuild is required.
The 1-second localization heartbeat has a 2.5-second receipt timeout to tolerate
scheduler jitter. Real localization state/validity transitions are published
immediately when the LIO worker observes them; the grace period does not turn an
explicit invalid pose into a valid one. The scan freshness limit remains 500 ms.
Source/epoch/backward-time/gap changes clear the query cache. Corrupt-region/query
failures clear this branch without stopping LIO or RoomMap.

The supplied DeviceConfig uses a 5000 ms RoomMap checkpoint/publication interval.
Acquisition, LIO and RoomMap observation/integration cadence are unchanged;
Foreground also keeps its configured cadence (currently 200 ms in DeviceConfig,
versus the API default of 250 ms). Newly confirmed geometry reaches subtraction
and the Gaussian worker at the next completed checkpoint. Gaussian fitting and
disk work add time: 5 seconds is a target update cadence, not a maximum latency
guarantee. No future/newer-than-scan background is used to subtract that scan.

Without LIO, the sensor must stay fixed. Bad ego-motion can reveal the room as
foreground. Larger distance tolerance suppresses more noise but masks nearby
objects/small movement. Foreground alone cannot distinguish unmapped background
from actual motion. No real moving-object precision/recall claim is made.

## Configuration

```text
ForegroundEnabled: 1
ForegroundBackgroundDistanceMeters: 0.15
ForegroundOutlierFilterEnabled: 1
ForegroundOutlierRadiusMeters: 0.15
ForegroundOutlierStartDistanceMeters: 1.0
ForegroundOutlierEndDistanceMeters: 5.0
ForegroundOutlierMaxRadiusMeters: 0.30
ForegroundOutlierMinNeighbors: 2
ForegroundMaxPoints: 12000
ForegroundPublishIntervalMilliseconds: 250
ForegroundWebSocketBindAddress: 127.0.0.1
ForegroundWebSocketPort: 8768
```

Distance is a geometric radius in meters, not a velocity threshold. Maximum points
limits sampling/display, not stored map size. `ForegroundEnabled: 0` disables
sampling and both workers. The interval controls sampling, filtering and result
publication together, not LiDAR decode or RoomMap update cadence. Cache budgets derive from `RoomMapCacheMaxVoxels`
and `RoomMapCacheMaxTiles`; client/retry settings derive from RoomMap sockets.
`GrafanaEnabled` controls its socket. LiDAR/IMU health telemetry is unchanged.
The outlier radius is scan-to-scan-point support within one sample, whereas the
background distance is scan-to-confirmed-map tolerance: they serve different
purposes. Disable only the spatial filter with `ForegroundOutlierFilterEnabled: 0`
for A/B checks; RoomMap subtraction remains active. Sparse distant people, thin
objects and heavily subsampled clouds may lose genuine points; tune against
recorded data rather than assuming all removed points are sensor errors.
The same filter now adapts its support radius to sensor-relative 3D range:
`fraction = clamp((range - startDistance) / (endDistance - startDistance), 0, 1)`;
`radius = base + (max(base, configuredMax) - base) * fraction`.
With the settings above it uses 0.15 m through 1 m, 0.1875 m at 2 m,
0.225 m at 3 m, 0.2625 m at 4 m and 0.30 m at/after 5 m. Start/end/max are all configurable;
end must be greater than start. It still needs two distinct neighboring positions in this scan;
the larger radius is not evidence of motion. The scan's localized sensor origin
is used, never the world/map coordinate origin. A missing/nonfinite sensor origin
falls back to the fixed base radius. Setting max <= base selects fixed-radius
behavior for A/B replay without changing RoomMap subtraction. Existing configs
with a larger base remain valid; the effective maximum is never smaller than base.
`Reason` includes `adaptive_radius_points` and `missing_origin_points` alongside
isolated/unresolved counts. Packet schemas and Grafana plugins are unchanged.
No second filter, temporal persistence gate, additional worker or PCL dependency
is introduced. CPU comparison limits remain unchanged, though larger neighborhoods
can increase candidate checks or unresolved counts in dense/duplicate data.
See [adaptive-radius validation](FOREGROUND_ADAPTIVE_RADIUS_VALIDATION.md) for
the latest recorded-data A/B measurements and their limitations.
An object already in the confirmed map remains background under the unchanged
subtraction rule. New unmapped walls can still appear as foreground.
Old `Motion*` settings are retired: use the updated DeviceConfig, not old
displacement/confirmation/history/cluster settings.

## Single Grafana update

The 250 ms scheduling change needs only the rebuilt C++ application. Existing
plugin 1.2.2 and V5 panels remain compatible; no plugin reinstall is needed for
this change. V5's panel description is updated, without query/schema changes.
The following installation steps apply if the diagnostic-capable plugin has not
yet been installed.

The foreground socket also sends a small `foreground-status` JSON text message
before each unchanged LPC1 binary frame. The panel shows the precise pause reason
(including the LIO reason), processed/background/unresolved sample counts, and
distinguishes matched-background zero output from missing input. Older clients
ignore the extra text and can still decode LPC1. This does not relax any pose,
freshness or distance gate, and does not enable motion classification.
`inputPoints` describes the post-isolation sample queried against the map. The
existing Reason text adds `isolated_removed` and `scan_filter_unresolved`; final
console summaries include aggregate removed/unresolved points and neighbor checks.
No panel schema, plugin rebuild or dashboard reimport is needed for these fields.
`Connected` only confirms the socket. A default `frame 0` can mean there is no
fresh accepted scan, not that all samples matched background. If upstream
processing/queueing exceeds the 500 ms freshness budget, the branch intentionally
stays empty. Test/profile an optimized Release build rather than treating stale
observations as current data by increasing the timeout.

1. Stop Grafana and replace the CONTENTS of
   `data/plugins/vista-lidarpointcloud-panel` with the contents of
   `dashboard/plugins/lidarpointcloud/dist` (version **1.2.2**). `plugin.json` must
   be directly in that folder, not under another `dist` directory.
2. Keep only one folder declaring this plugin ID. Keep its unsigned-plugin
   allowance and the separate `vistalive` plugin unchanged.
3. Restart Grafana, hard-refresh, and import the updated existing
   `dashboard/VISTA_Live_Dashboard_V5_Gaussian_Map.json` with UID `vistalive`.
4. Panel **Foreground - Points Outside RoomMap** uses `ws://127.0.0.1:8768`.
   It clears on disconnect or 1000 ms without frames. RoomMap and Gaussian keep
   their persistent geometry. No intermediate 1.2.1 install is required.
5. For ADLINK 192.168.10.2, bind its socket appropriately and set the panel URL
   to `ws://192.168.10.2:8768`. The browser connects directly. Restrict these
   unauthenticated, non-TLS sockets to your trusted LAN.

No V6 dashboard, full live-sensor 3D stream, Ground worker or camera pipeline was
added. Future clustering, motion classification and persistent tracking remain
separate stages after foreground.
