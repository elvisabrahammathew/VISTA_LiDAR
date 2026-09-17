# Third-party dependencies

This directory contains dependencies owned by neither VISTA nor the C++
application. Keep each dependency self-contained and preserve its license and
notice files.

## librealsense 2.50.0

Expected layout:

```text
realsense/
`-- librealsense-2.50.0/     Complete Intel librealsense source release
```

The application builds this source as a static `realsense2` library for the
active compiler and CPU. Therefore no prebuilt `.dll`, `.lib`, or `.so` is kept
in `third_party`, and Windows x64, Linux x86-64, and Linux ARM64 all use the same
source tree. Each target still needs its own build directory and native or
cross-compilation toolchain.

The SDK examples, tools, bindings, tests, CUDA processing, update checks, and
firmware downloads are disabled by the application's root `CMakeLists.txt`.

Two small CMake integration patches are kept in the vendored source:

- `CMake/embedd_udev_rules.cmake` generates `udev-rules.h` with one write
  operation, avoiding an intermittent Windows file-lock error.
- `CMake/connectivity_check.cmake` skips the network probe when the SDK download
  features are disabled, allowing deterministic offline configuration.

Neither patch changes device or frame-processing behavior.

The `others` directory is reserved for future external dependencies.
