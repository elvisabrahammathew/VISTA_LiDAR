# Changelog

## 1.10.1

- Align settings controls in each row when labels or covariance values wrap. Align paired Settings dashboard panels and reduce unused height.

## 1.10.0

- Add the active minimum LiDAR correspondence count to VISTA Edge Settings under Point-LIO Matching and Timing.

## 1.9.1

- Keep the last accepted LiDAR marker visible through temporary pose gaps and while a new map arrives.

## 1.9.0

- Show LiDAR position and heading on Room Map and Gaussian Map. Use the last
  accepted pose during localization loss and Mount pose in fixed mode.

## 1.8.0

- Show confirmed corner markers and the existing edge splats within the selected
  radius. Set a per-panel radius in Corners mode or follow the edge default.

## 1.7.0

- Add the 13-panel VISTA Edge Settings dashboard with a dedicated authenticated
  settings WebSocket. Advanced changes apply for the running edge session.
- Use one shared connection and show device-discovered serial ports, capability
  flags, current settings, and paired covariance controls.

## 1.6.0

- Add the Unitree Room Map toolbar for six Mount values, LIO Enabled, Apply,
  and Reset view. The app requires a control token and confirms each reset.
- Map pose settings last for the running VISTA Edge session. Changing the pose
  starts a new Room Map while preserving the previous confirmed PCD.

## 1.5.0

- Add a Corners display mode for supported 3D vertices. It requires the matching
  `vista_edge` update, uses the existing Gaussian WebSocket, and retains Edges as
  the default. Checkboxes and shared camera work across all three modes.

## 1.4.2 - 2026-11-18

- Give the Gaussian display selector and Reset view button explicit 32 px heights.
- Reset inherited native-control styling and set readable select/option colors.
- Align layer checkboxes, labels and controls; wrap complete control groups on
  narrow panels. Preserve Edges as default and independent Foreground toggling.

## 1.4.1 - 2026-11-18

- Default Gaussian display to Edges for new panels and panels without a saved
  surface mode. Explicit Full Surface selections remain available and respected.
- Set the supplied V5/V6 Gaussian overlay panels to Edges. Foreground is unchanged.

## 1.4.0 - 2026-11-18

- Add a saved **Full Surface / Edges** dropdown to the Gaussian overlay toolbar
  and panel options. Foreground retains its independent checkbox and connection.
- Request cached geometric edge splats from the edge application before GSP1
  transmission. Mode changes work on frozen maps without resetting the camera.
- Detect older servers and disable Edges when their metadata lacks support.
- Preserve Full Surface, ordinary RoomMap LOD and the full-panel sizing update.
- Requires the matching C++ edge-cache/protocol update; no new port or worker.

## 1.3.1 - 2026-11-18

- Fill the full panel with both Room Map and Gaussian/Foreground canvases,
  independently of Windows display scaling, browser zoom and pixel ratio.
- Canvas CSS follows the panel while the drawing buffer retains pixel-ratio sizing.
- V6 places the two explanatory notes at the bottom in taller, equal-width columns.

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
