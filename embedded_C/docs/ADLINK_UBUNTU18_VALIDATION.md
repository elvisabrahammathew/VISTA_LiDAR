# ADLINK Ubuntu 18.04 compatibility validation

Validated on 2026-10-06. This is software/toolchain validation, not a physical
ADLINK acceptance test or a long-running performance certification.

## Target and changes

Target reported by the user: NEON-2000-JNX, aarch64, Ubuntu 18.04.5,
Jetson Linux R32.5, GCC 7.5.0 and CMake 3.10.2. The current deployment does
not use a camera. No changes to Ubuntu, JetPack, CUDA, LiDAR configuration,
Grafana dashboard/plugin, or the localization/mapping algorithm are required.

- CMake minimum is 3.10. OBJECT sources are compiled once and attached explicitly
  to the application/tests. A separate INTERFACE target carries dependencies.
- `1_Platform/compat/filesystem.hpp` provides `vista::fs` and file-stream aliases.
  CMake probes standard filesystem header/link support, including `stdc++fs`
  for GCC 8. Older toolchains select the portable implementation automatically.
- `third_party/filesystem` is a Git submodule pinned to ghc/filesystem v1.5.16,
  commit `c306390e0d5670e3d832e7798d27e99e41668a9b`, MIT licensed. It is not
  patched. Its implementation is compiled once; public headers use `fs_fwd.hpp`
  so Windows system macros cannot leak into VISTA numeric/socket headers.
- `scripts/build_adlink.sh` uses traditional CMake directory-based commands,
  Release, tests ON, RealSense OFF and two build jobs. It neither upgrades nor
  installs system packages. C++17 is retained; Python/CUDA/ROS are not required.
- Four additional tests cover path normalization, Unicode/space-containing
  filenames, stream I/O, recursive iteration, rename, disk space and errors.

## Results

CTest registers one suite executable; that suite contains 162 individual cases.

| Environment | Build / filesystem | RealSense | Individual tests | Final suite time |
| --- | --- | --- | --- | --- |
| Windows x64, VS 2019 MSVC 19.29 | Debug / standard | ON | 162/162 passed | 46.24 s |
| Windows x64, VS 2019 MSVC 19.29 | Debug / forced portable | OFF | 162/162 passed | 57.87 s |
| Ubuntu 24.04 x86_64 WSL, GCC 13.3, CMake 3.28.3 | Debug / standard | ON | 162/162 passed | 5.27 s |
| Isolated Ubuntu 18.04 x86_64, GCC 7.5.0, CMake 3.10.2 | Release / portable | OFF | 162/162 passed twice | 16.85 s / 15.90 s |
| Ubuntu 18.04 ARM64 cross compiler GCC 7.5.0, CMake 3.10.2; QEMU cortex-a57 | Release / portable | OFF | 162/162 passed | 29.74 s |

The ARM64 binary is an aarch64 ELF using `/lib/ld-linux-aarch64.so.1`, linked
against Bionic's libraries. Its versioned requirements top out at GLIBC 2.17
and GLIBCXX 3.4.22; it does not depend on the modern WSL host C++ runtime.
FAST-LIO code, Unitree decoding/IMU routing, independent live processing,
room-map/PCD storage, message bus and WebSocket tests are included in the suite.

## Failures investigated during validation

The initial forced-portable Windows build exposed `windows.h` min/max and
near/far macro leakage. Separating the implementation from the public header
resolved this; the final forced-portable build and full suite passed.

Bionic tests initially failed the worker's final PCD export because the test
chroot did not have `/proc` mounted. Syscall tracing showed a successful
directory read/close followed by glibc's allocator probing the missing
`/proc/sys/vm/overcommit_memory`, contaminating errno during directory cleanup.
Mounting `/proc` fixed the test environment; no export logic or test assertion
was relaxed. Native Bionic passed twice and emulated ARM64 passed afterward.

An initial Windows standard-backend run reported a non-empty temporary directory
during test cleanup. This was not reproduced in the final standard/portable
runs. Existing upstream ikd-Tree template warnings and compact-code indentation
warnings remain; successful build does not mean a warning-free source tree.

## Deployment and limitations

Follow the ADLINK section in `../README.md`: initialize all submodules, run
`bash embedded_C/scripts/build_adlink.sh`, then edit/run from the generated
build directory. Select the actual Linux serial path and device permissions.
Use the correct Grafana/LAN addresses; localhost on a browser PC is not ADLINK.
Do not deploy an arbitrary ARM64 executable built against newer Ubuntu libraries.

The old-toolchain tests run on the WSL host kernel, with an isolated Bionic
userspace and Bionic ARM64 libraries under QEMU. They do not reproduce the
Jetson R32.5 kernel, USB adapter, power/thermal behavior or physical IMU timing.
RealSense on Bionic, serial acquisition at 4 Mbaud, physical motion/localization,
actual Grafana rendering, throughput and 24/7 stability still require tests on
ADLINK. No detection/tracking feature is added by this compatibility change.

## References

- [ghc/filesystem implementation and separated-header usage](https://github.com/gulrak/filesystem)
- [GCC 8 standard library changes](https://gcc.gnu.org/gcc-8/changes.html)
- [CMake 3.10 library targets](https://cmake.org/cmake/help/v3.10/command/add_library.html)
