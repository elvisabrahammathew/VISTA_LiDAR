# Ground processing: retired runtime, retained reference

Retired on 2026-10-21 at the owner's request. This change removes Ground work;
it does not implement motion classification, clustering, detection or tracking.
The owner explicitly selected **keep the RoomMap algorithm unchanged**.

## Active behavior

- No `ground-processing` worker is declared, validated, started or joined.
- World and sensor-frame cleaning workers have no IMU subscription, Ground
  publisher, plane calibration or floor removal. IMU still feeds LIO and health.
- `MountingConfig` in `4_Applications/processing/pointcloud/mounting.hpp` contains
  only the six initial/fixed world-pose fields. LIO receives the same initial pose.
  With LIO off, mounting is fixed; there is no former IMU-for-Ground tilt override.
- Optional point-cloud PCD logging consumes `pointcloud/cleaned` directly,
  including floor points. It no longer waits for `pointcloud/processed`.
  RAW logging and RoomMap PCD format/save policy are unchanged.
- Grafana Bridge no longer subscribes to, wakes for or publishes Ground data.
  Re-import the existing updated V5 dashboard: Ground State and Ground Validation
  are removed, Room Map State occupies their first row. No plugin update is needed.
- LiDAR/IMU health, valid-pose gating, RoomMap confirmation/free-space clearing,
  tile formats, Gaussian mapping, WebSocket codecs and host clock behavior remain.

Remove these obsolete keys from local DeviceConfig copies: `GroundMode`,
`UseImuForGround`, `FloorZ`, `GroundDistanceThreshold`, `GroundNormalToleranceDeg`,
`GroundCalibrationFrames`, `GroundMinInlierRatio`. Parsing rejects them rather
than silently suggesting Ground is still active. Keep all six `Mount...` keys.

## What existed, and what is retained

Ground was an independent consumer of cleaned world points. It output a floor-
removed cloud and Ground status, while RoomMap independently retained the floor.
Its modes were fixed height, startup RANSAC and hybrid fixed-height/RANSAC.
Defaults included a 0.10 m plane-distance band, 15-degree normal tolerance and
50 calibration frames. It fit one plane once, not multi-surface adaptive terrain.
Hybrid constrained the plane near the configured floor and fell back to a
horizontal plane when calibration failed. IMU optionally provided stationary
gravity alignment, not floor height or the slope of an arbitrary surface.

For possible reuse, `GroundRemovalConfig`, the direct `PointCloudPreprocessor`
Ground methods, the independent Ground message, pure legacy Grafana formatters,
and optional LPC1 Ground header support remain as tested reference APIs.
They are **not enabled by AppConfig**. Both cleaning worker entry points reset
the reference Ground option before constructing their processor, never call
Ground methods and never consume IMU. Retaining source does not run RANSAC.

The complete pre-removal runtime is recorded in Git commit:

```text
37c6269b9ea3773feb0ed368a9aacc31ce69c2df
```

Read the original files without overwriting the current tree, for example:

```bash
git show 37c6269b9ea3773feb0ed368a9aacc31ce69c2df:embedded_C/4_Applications/processing/pointcloud/pointcloud_processing.cpp
git show 37c6269b9ea3773feb0ed368a9aacc31ce69c2df:embedded_C/main.cpp
```

## If restoring it later

Restore it as an explicitly optional perception branch, not a prerequisite for
RoomMap, Gaussian, LIO or health telemetry. Keep mounting independent and avoid
applying the world transform twice. Revisit the single-plane/startup-only limits
before using it on slopes, stairs or a moving robot. Reintroduce the worker,
configuration, topic wiring and dashboard together, with migration tests.
Do not simply reset the entire project to the historical commit.

Two internal legacy topic names are retained for codec/reference tests only;
production has no publisher for `pointcloud/processed` or `processing/ground_status`.
New point-cloud consumers must use the cleaned topic or a new, explicit perception
topic rather than depending on the retired Ground stage.
