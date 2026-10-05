# lidarpointcloud

`lidarpointcloud` is an unsigned Grafana panel plugin for displaying live LiDAR point clouds with Three.js/WebGL.

The ground grid covers 60 m x 60 m, centered at the world origin (`-30` to
`+30` m on X and Y), with one square per metre.

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
| 5 | uint8 | Flags; bit 0 means intensity exists, bit 1 means ground diagnostics exist |
| 6 | uint16 | Header size (`32`, or `96` with ground diagnostics) |
| 8 | uint64 | Frame ID |
| 16 | uint64 | Timestamp in nanoseconds |
| 24 | uint32 | Point count |
| 28 | uint32 | Point stride: `12` for XYZ or `16` for XYZI |
| 32 | 64 bytes | Optional ground state, mode, plane, quality metrics and removal counts |
| header size | float32[] | Interleaved little-endian XYZ[I] points |

When the ground extension is present, the panel draws a translucent plane and
normal arrow. Green means valid, yellow means calibrating, and red indicates a
fallback/invalid model. The overlay text also reports tilt, height error,
inlier ratio, removal ratio, and whether IMU or configured mounting pose was used.

## Build

```text
pnpm install
pnpm run typecheck
pnpm run build
```

The generated installable plugin is in `dist`.

# Room-map snapshots (1.0.3)

An empty replacement snapshot clears the previous 3D geometry. A connected
WebSocket with zero points is shown as connected/no-data, not disconnected.
Use the V4 Free Space dashboard's Room Map State panel to distinguish DISABLED,
LOAD ERROR, EMPTY, BUILDING, FROZEN, and LOADED / READ ONLY.
The C++ GrafanaEnabled setting controls both WebSocket outputs.

# Room-map LOD (1.1.0)

Port 8766 can now send room-map metadata followed by bounded LPC1 view frames.
The panel sends camera/frustum requests automatically while zooming/panning.
Maximum displayed points is a per-view rendering budget, not a total-map limit.
The C++ mapper stores full geometry in disk tiles and exports every confirmed
point to PCD. This is a Potree-inspired hierarchy, not a native Potree file reader.
Install this updated plugin alongside the updated C++ application; a 1.0.x
plugin cannot request camera-dependent detail. Port 8765 live streams are unchanged.
