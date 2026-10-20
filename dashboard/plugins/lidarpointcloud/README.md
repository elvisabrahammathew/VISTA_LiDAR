# lidarpointcloud

`lidarpointcloud` is an unsigned Grafana panel for Room Map and geometric Gaussian surfaces with Three.js/WebGL. Plugin ID is unchanged: `vista-lidarpointcloud-panel`.

The ground grid covers 60 m x 60 m, centered at the world origin (`-30` to
`+30` m on X and Y), with one square per metre.

## Sources

- **WebSocket**: receives Room Map `LPC1` on :8766 and Gaussian Map `GSP1` on :8767. Earlier `LDR1` test packets remain compatible. Production no longer publishes live sensor 3D frames.
- **Grafana DataFrame**: reads numeric fields named `x`, `y`, `z`, and optional `intensity`/`reflectivity`/`i` from the panel query.

The browser opens the WebSocket. `127.0.0.1` means the browser's computer, not necessarily Grafana or VISTA. For ADLINK use its reachable LAN address in both panel URLs.

## Gaussian surface rendering (1.2.0)

GSP1 is little-endian: magic[4], header size u32 (32), point count u32,
stride u32 (44), completed revision u64, host timestamp ns u64; followed by
11 float32 values per record: XYZ, normalized intensity, covariance
xx/xy/xz/yy/yz/zz (m²), opacity. Packet length and positive-definite covariance
are checked before accepting a replacement frame.

Instanced quads project the full world covariance into screen ellipses and
blend Gaussian alpha back-to-front. Camera motion updates projection/sorting;
VIEW requests obtain bounded disk-backed regional samples. Empty replacements
clear old geometry. Disconnects retain the last scene. Point size applies only
to ordinary point clouds; Gaussian size comes from covariance in metres.

These are CPU-fitted geometric/intensity surface Gaussians, NOT a photometrically
trained RGB 3DGS model. No camera, clustering, detection or tracking is added.
The external Python splatting repository is not launched on map updates.

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
