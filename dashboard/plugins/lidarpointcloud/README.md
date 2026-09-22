# lidarpointcloud

`lidarpointcloud` is an unsigned Grafana panel plugin for displaying live LiDAR point clouds with Three.js/WebGL.

## Sources

- **WebSocket**: receives binary `LPC1` packets and also accepts the earlier `LDR1` test packets.
- **Grafana DataFrame**: reads numeric fields named `x`, `y`, `z`, and optional `intensity`/`reflectivity`/`i` from the panel query.

The WebSocket is opened by the user's browser. Therefore, `ws://127.0.0.1:8765` refers to the computer running the browser, not necessarily the Grafana server.

## LPC1 packet format

All multi-byte values are little-endian.

| Offset | Type | Meaning |
|---:|---|---|
| 0 | char[4] | `LPC1` |
| 4 | uint8 | Version (`1`) |
| 5 | uint8 | Flags; bit 0 means intensity exists |
| 6 | uint16 | Header size (`32`) |
| 8 | uint64 | Frame ID |
| 16 | uint64 | Timestamp in nanoseconds |
| 24 | uint32 | Point count |
| 28 | uint32 | Point stride: `12` for XYZ or `16` for XYZI |
| 32 | float32[] | Interleaved little-endian XYZ[I] points |

## Build

```text
pnpm install
pnpm run typecheck
pnpm run build
```

The generated installable plugin is in `dist`.
