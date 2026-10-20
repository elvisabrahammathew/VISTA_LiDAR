# Incremental geometric Gaussian mapping

This optional ROS-free CPU branch consumes `mapping/room_map_view`, never raw
LiDAR scans or the bounded `mapping/room_map` display preview. Room Map remains
the source of truth, including the floor. Camera, clustering, detection and
tracking are not implemented here. This is a geometric surfel representation,
not the external batch Python optimizer or camera-trained RGB 3DGS.

## Data flow

Confirmed Room Map checkpoint -> immutable full region pages -> Gaussian worker
-> changed tiles plus neighboring support -> completed immutable Gaussian view
-> bounded GSP1 WebSocket :8767 -> covariance ellipses in the existing Grafana panel.

Room Map :8766 and its PCD export remain unchanged. Live sensor 3D :8765 is no
longer started by the application. Internal cleaned sensor processing and all
LiDAR/IMU/worker/Grafana health metrics remain active independently of LIO.

## Updates and consistency

The mapper hashes every confirmed XYZ/intensity record while checkpointing its
existing tile page. Evidence-only changes reuse the previous region version.
Geometry changes retain an immutable page using a filesystem hard link, with a
copy fallback. A snapshot owns references to those pages, so later page replacement
or deletion cannot change a Gaussian job already reading an older revision.
Old immutable files are removed when the last snapshot releases them.

The Gaussian worker coalesces pending checkpoints, reads full geometry one tile
and a 3-voxel halo at a time, deduplicates by source voxel, estimates a PCA normal
from nearby confirmed points and fits a thin anisotropic surface Gaussian. Sparse
or non-planar neighborhoods use a small isotropic fallback; they are not extended
into a synthetic plane. New/deleted regions invalidate neighboring fits as well.
The entire Gaussian view switches only after a job completes. Fit/output-cache
errors clear the optional view and retry on a later checkpoint; acquisition is
not stopped. If the mapper cannot retain an immutable source page, it disables
only the optional snapshot catalog and reports a Gaussian error requiring restart;
the successful ordinary Room Map checkpoint is preserved.

Point working memory is limited by tile dimensions (at most a 72³-cell working
set), not total map points. Region/page catalogs do grow with mapped area. Output
attributes live in per-tile `.gsurf` cache pages; older pages are reference-counted.
These are temporary derived caches, not an additional PCD/PLY export format.
Normal shutdown removes referenced derived files; empty cache folders may remain.
An interrupted process can leave cache pages behind, as with existing map caches.

Gaussian camera views frustum-cull tiles and stratify fixed-width disk records
within the point budget. This is a bounded regional surface view, not a new full
Potree converter. Room Map keeps its existing octree LOD. No total storage cap is
introduced; disk space, per-tile metadata and browser GPU/CPU still set practical
limits. Coalescing may skip intermediate revisions; it never mixes incomplete
revisions or gates Room Map fusion on Gaussian completion.

Gaussian adds CPU work and 44 bytes per displayed splat (ordinary map points use
16 bytes). Removing the former 100 ms live stream reduces one workload, but does
not make surface fitting free. Initial large reference conversion can take much
longer than one checkpoint period, especially in Debug. Use Release on ADLINK;
lower the Gaussian view budget if browser/network load is high.

## Configuration

- `GaussianMapEnabled: 1`: enable the derived worker; `0` avoids snapshot/fit work.
- `GaussianMapWebSocketBindAddress: 127.0.0.1`: use the ADLINK LAN address or 0.0.0.0
  for a browser on another machine (trusted LAN only, no authentication/TLS).
- `GaussianMapWebSocketPort: 8767`: must differ from Room Map's port.
- `GaussianMapWebSocketMaxPoints: 100000`: one rendered view budget, not stored map.

`RoomMapPublishIntervalMilliseconds` schedules source checkpoints. No extra
Gaussian timer is added. Socket MaxClients/RetryIntervalSeconds are inherited
from Room Map. `GrafanaEnabled` gates both map sockets/telemetry, not mapping.
Load-existing and frozen maps are also valid Gaussian sources. A missing input
PCD remains LOAD ERROR and does not start automatic reconstruction.

Install the rebuilt `dashboard/plugins/lidarpointcloud/dist` (same plugin ID),
restart Grafana, hard-refresh the browser and overwrite the existing dashboard
with `dashboard/VISTA_Live_Dashboard_V5_Gaussian_Map.json`. No vistalive update or
new plugin installation is required. Existing camera/clustering workflows are
untouched.
