# Third-party dependencies

This repository-level directory contains dependencies owned by neither VISTA
nor its C++/Rust applications. Both implementations consume the same pinned
source, so third-party code is not duplicated below either language folder.

## librealsense 2.50.0

Expected layout:

```text
librealsense/     Git submodule pinned to librealsense v2.50.0
```

Initialize it after cloning the parent repository:

```text
git submodule update --init --recursive
```

Each application builds the checked-out source as a static `realsense2` library
for the active compiler and CPU. Therefore no prebuilt `.dll`, `.lib`, `.a`, or
`.so` is kept in `third_party`, and Windows x64, Linux x86-64, and Linux ARM64
all use the same source commit. Generated libraries stay in the CMake or Cargo
build directory.

The SDK examples, tools, bindings, tests, CUDA processing, update checks, and
firmware downloads are disabled by the application's root `CMakeLists.txt`.
The submodule is kept unmodified so its pinned commit remains reproducible.

## Moving LiDAR mapping (C++)

- `fast_lio`: FAST-LIO source, including its recursively initialized `ikd-Tree`.
- `eigen`: Eigen 3.4.0 headers used by IKFoM and the VISTA LIO adapter.
- `boost_preprocessor`: Boost.Preprocessor 1.85.0 headers used by IKFoM's state macros.

These are pinned Git submodules, not downloaded platform binaries. The C++ build
generates portable private copies under its build directory and links a static
`vista_fast_lio` into `vista_edge` on Windows x64, Linux x86-64 and Linux ARM64.
Upstream checkout files remain unmodified. ROS, PCL and Python are not required
by the adapter; see `embedded_C/4_Applications/mapping/lio/README.md` for licensing
and the distinction between local odometry and globally corrected SLAM.

Unitree protocol references remain in `unitreelidar` and `unilidar_sdk2`; the
existing device decoder does not link their Linux-only vendor binaries.

The `others` directory is reserved for future external dependencies.
