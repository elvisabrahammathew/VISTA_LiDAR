# Bundled C++ build libraries

These exact upstream source snapshots are required to build `embedded_C` from
this directory alone. They are ordinary versioned files rather than Git
submodules. No network download or sibling `third_party/` folder is needed.
The corresponding code is not modified here; VISTA's adaptations are made in
its own CMake files and generated build directory.

| Directory | Upstream | Pinned commit | Use |
| --- | --- | --- | --- |
| `point_lio_unilidar` | https://github.com/unitreerobotics/point_lio_unilidar | `18ed5976d8fab2bd8a5148c26a40692bd3c0dc91` | IMU/LiDAR localization math, IKFoM, ikd-Tree |
| `eigen` | https://gitlab.com/libeigen/eigen | `3147391d946bb4b6c68edd901f2add6ac1f31f8c` | Linear algebra headers |
| `boost_preprocessor` | https://github.com/boostorg/preprocessor | `c4ea7e40d365ba28faecef8917d5c3f1e0121bf9` | Point-LIO state macros |
| `filesystem` | https://github.com/gulrak/filesystem | `c306390e0d5670e3d832e7798d27e99e41668a9b` | Portable filesystem fallback |
| `librealsense` | https://github.com/realsenseai/librealsense | `c94410a420b74e5fb6a414bd12215c05ddd82b69` | Optional RealSense L515, enabled by default in shared presets |

Each directory preserves its upstream license and notices. The source is
compiled locally for the host platform. `third_party/unilidar_sdk2` remains a
protocol reference and is not part of the current C++ build.
