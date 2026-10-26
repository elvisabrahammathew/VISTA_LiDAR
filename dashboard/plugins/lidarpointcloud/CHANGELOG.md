# Changelog

## 1.3.0 - 2026-10-22

- Optional Gaussian + Foreground overlay uses two sockets and one camera/canvas.
- Separate layer toggles, point budgets, foreground color/size and expiry.
- Foreground pause/empty/disconnect/stale events clear only Foreground; Gaussian
  transport failures retain its last completed map with a disconnected status.
- Reject wrong-layer protocols and obsolete socket callbacks; cancel retries,
  timers and both render resources on unmount. Gaussian VIEW requests follow the
  shared camera; Foreground never auto-fits/resets it.
- V5 merges panels 51/90 into panel 51 and retains RoomMap LOD and all health
  panels. Existing single-stream/DataFrame options remain backward compatible.
- C++ workers, two socket protocols and mapping/foreground algorithms unchanged.

## 1.2.2 - 2026-10-21

- Foreground diagnostic metadata distinguishes paused localization/stale input
  from matching background, with processed/background/unresolved counters.
- V5 replaces the unverified moving-object view with live RoomMap foreground.
- Retains the opt-in stale/disconnect clearing introduced in 1.2.1.
- RoomMap/Gaussian persistent geometry and vistalive are unchanged.

## 1.2.1 - 2026-10-21

- Optional stale/disconnect clearing for transient moving-point panels.
- Room Map and Gaussian Map retain their existing geometry by default (timeout 0).
- Transient empty frames do not repeatedly reset the camera; obsolete callbacks are ignored.

## 1.2.0 - 2026-10-20

- Current dashboard import is V5_Gaussian_Map; V4 is kept as a previous file.

- GSP1 world covariance/opacity decoding and anisotropic Gaussian ellipse rendering.
- Back-to-front blending and bounded map views; empty replacements remove ghosts.
- V4 replaces Live Point Cloud 3D with Gaussian Map; LiDAR/IMU health is unchanged.
- Fit the geometry instead of the decorative grid, allowing useful room-scale zoom.

## 1.1.0 - 2026-10-05

- Camera-dependent out-of-core room-map LOD over the existing WebSocket.
- Separate stored-point count from currently rendered points; bounded GPU buffers.
- Map state and global bounds metadata, automatic initial camera fit, independent panel views.
- Live LPC1/LDR1 and Grafana DataFrame modes remain supported.

## 1.0.3 - 2026-10-05

- Clear old geometry when a replacement point-cloud snapshot contains zero points.
- Distinguish connected-with-no-data from a disconnected WebSocket.

## 1.0.2 - 2026-09-23

- Decode the optional LPC1 ground-diagnostics extension.
- Draw the selected ground plane and its normal in the 3D scene.
- Show calibration state, tilt, height error, inlier ratio, removed ratio, and orientation source.

## 1.0.1 - 2026-09-23

- Expand the world-coordinate ground grid from 20 m x 20 m to 60 m x 60 m.
- Keep fit/reset camera views wide enough to show the full ground grid.

## 1.0.0 - 2026-09-22

- Initial `lidarpointcloud` panel.
- Three.js point-cloud rendering with orbit, zoom, grid, axes, reset view, and browser-side sampling.
- LPC1 and legacy LDR1 WebSocket support.
- Grafana DataFrame support for x/y/z and optional intensity fields.
