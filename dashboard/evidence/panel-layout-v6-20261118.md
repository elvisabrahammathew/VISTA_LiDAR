# V6 panel layout verification — 18 November 2026

- Fixed both canvas CSS dimensions independently of DPR-scaled drawing buffers.
- Actual React/Three.js components tested in headless Edge at DPR 0.75, 1, 1.5, 2.
- Both canvases filled 600x440, 700x520 and 460x300 after dynamic resizing.
- Drawing buffer dimensions matched CSS size multiplied by DPR; no page errors.
- Strict TypeScript check and 24 existing protocol/layer tests passed.
- Production plugin build passed (1.3.1; bundle-size warnings only).
- V6 notes are at the bottom, x=0/12, width=12 each, height=16; no overlaps.
- All existing endpoints, queries, note contents and other panel settings retained.
- Prepared files only; installed Grafana instance/service not changed or verified.
