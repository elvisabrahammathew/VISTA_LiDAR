# Local validation — 18 November 2026

Windows development host; no ADLINK connection or real weights available.

- New Python suite: 10 passed (7 real loopback HTTP bridge tests and 3 gateway bundle tests).
- Existing person_app/person_app_http regressions: 8 passed.
- TypeScript strict typecheck: passed.
- Protocol tests: passed; HM01, coordinate rotation, invalid lengths/vertices/indices,
  stale/absent/stopped/demo visibility.
- Production webpack build: passed. Three performance warnings for bundled Three.js
  (~508 KiB); no external CDN is required.
- Actual React panel + Three.js in headless Edge via Playwright: passed synthetic
  mesh render, reset, disconnect and reconnect; no page errors. This harness is not
  a running Grafana server acceptance test.
- V6 JSON: every V5 panel/query/option retained with vertical shift only, UID
  vistalive retained, unique IDs and no panel overlap.
- Python 3.8 syntax: parsed locally; execution on NEON still pending.

Not verified: Windows CUDA/model execution, trained YOLO live inference, ADLINK
camera/network, installed Grafana plugin, real motion accuracy or throughput.
Missing assets: complete verified InstantHMR bundle and registered PIC YOLO ONNX.
Synthetic preview is display QA only, not an inference or accuracy result.
