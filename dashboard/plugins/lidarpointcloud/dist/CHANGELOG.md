# Changelog

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
