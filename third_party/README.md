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

The `others` directory is reserved for future external dependencies.
