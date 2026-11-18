# VISTA Digital Twin panel 1.0.0

One body-relative inferred mesh, independent of the LiDAR map panels. The Windows
browser renders Three.js locally. ADLINK owns the camera; the selected `lab` worker
runs YOLO + InstantHMR on Windows and posts results back to the gateway. A read-only
Windows `scripts/grafana_bridge.py` forwards status/topology/HM01 to the panel.
No token is stored in dashboard options. No external CDN/runtime model downloads.

Build: `pnpm install --frozen-lockfile`, `pnpm typecheck`, `pnpm test`, `pnpm build`.
Copy `dist` as `<Grafana plugins directory>/vista-digitaltwin-panel`, add
`vista-digitaltwin-panel` to the existing `[plugins] allow_loading_unsigned_plugins`
list, restart Grafana and import V6. See `../../DEPLOY_DIGITAL_TWIN_V6_EN.md`.

Panel options: bridge URL (default `http://127.0.0.1:8101`), polling interval, wireframe.
Serial polling, bounded response decoding, exact topology SHA-256, gateway session
checks, monotonic-age expiry and immediate clear on errors prevent stale bodies.
One request cycle has a 2 s deadline. A 204 response does not refresh a live mesh.
Gateway demo/replay are explicitly labelled. GPU render FPS is not inference FPS.

Use the dashboard in a browser on the Windows bridge computer, from an allowed
Grafana HTTP origin (default localhost/127.0.0.1:3000). An HTTPS/remote deployment
needs a same-origin HTTPS reverse proxy; do not expose the loopback bridge to LAN.
No camera/LiDAR calibration or room placement is supplied by this panel. Hidden
surfaces are model inference. Multiple-person Virtual Ward is a different protocol.
