# VISTA LiDAR Rust

This Rust crate mirrors the four-layer layout of the C++ implementation in
`../embedded_C`:

- `1_Platform`: message bus, Pub/Sub, metrics, threading, and OS adapters.
- `2_Transport`: Ethernet, librealsense USB, HTTP messaging, and storage.
- `3_Devices`: common LiDAR interfaces and device-specific drivers.
- `4_Applications`: logging, preprocessing, monitoring, Grafana output, and
  reserved object-processing applications.
- `models`: LiDAR envelopes, point clouds, shared topic names, and telemetry.

Configuration is loaded from `DeviceConfig.txt`. Grafana authentication reads
`GrafanaSecret.txt` first and `VISTA_GRAFANA_TOKEN` second. Copy
`GrafanaSecret.example.txt` to the ignored local file and keep the real token
out of Git:

```text
GrafanaToken: glsa_your_generated_token
```

LiDAR file logging is disabled by default and controlled independently:

```text
RawLoggingEnabled(Lidar): 0
PointCloudLoggingEnabled(Lidar): 0
```

Set a value to `1` to start its logger and create the corresponding `.bin` or
`.pcd` file. Reading, decoding, and preprocessing continue when both are `0`.

Run the checks from this directory:

```text
cargo fmt --check
cargo test
cargo run
```

## Static librealsense build

Rust and C++ share the repository-level librealsense `v2.50.0` submodule:

```text
../third_party/librealsense
```

On the first native `cargo build`, `build.rs` builds the static SDK inside
Cargo's ignored `target` directory:

```text
Windows x86-64   realsense2.lib
Linux x86-64     librealsense2.a
Linux ARM64      librealsense2.a
```

Only the library matching the native Cargo target is built. Linux ARM64 enables
the RSUSB backend; Windows and Linux x86-64 use their native backends by
default. No `realsense2.dll` or `librealsense2.so` is required at runtime.

Initialize the shared source once after cloning:

```text
git submodule update --init --recursive
```

Each static library must be built natively on its matching platform. A Windows
build cannot create Linux x86-64 or ARM64 libraries without a dedicated
compiler, sysroot, and CMake toolchain. Cargo reuses its generated build cache
until the build script or librealsense configuration changes.

Native OS calls are permitted at warning level and remain isolated in
`1_Platform/os`. Windows publishes CPU usage, process memory, total memory
usage, and free disk space through Win32 APIs. Linux reads CPU/memory and
thermal data from procfs/sysfs and uses `statvfs` for free disk space.
