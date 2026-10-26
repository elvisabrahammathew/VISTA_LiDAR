# Building VISTA C++ from a clean checkout

This guide covers the C++ application in `embedded_C`. The Rust reference,
WPF utilities, Grafana plugin build, and external Python splatting experiment
are separate projects. They are not prerequisites for building `vista_edge`.
The integrated Gaussian worker is C++; it does not require Python, uv, or npm.

## 1. Install the Windows build tools

Install Git and Visual Studio 2019 (16.11) or Visual Studio 2022 with:

- Desktop development with C++ (MSVC x64/x86 tools and Windows SDK).
- C++ CMake tools for Windows.
- CMake 3.20 or newer for the 2019 presets; 3.21 or newer for the 2022 generator.

The project uses C++17. VS Code is an editor, not a replacement for MSVC/SDK.
If using VS Code, install Microsoft's C/C++ and CMake Tools extensions too.
Do not copy `CMakeCache.txt`, `.vs`, generated `.sln`/`.vcxproj`, or an `out`
directory from another machine. Their paths/toolchain state are machine-local.

## 2. Clone and initialize the pinned C++ dependencies

Replace `<repository-url>` with the team's actual repository URL:

```powershell
git clone <repository-url> VISTA
cd VISTA
git submodule update --init --recursive -- third_party/point_lio_unilidar third_party/eigen third_party/boost_preprocessor third_party/filesystem third_party/librealsense
```

Run the same submodule command after pulling updates. Use the commits recorded
by the parent repository; do not use `git submodule update --remote` to build a
release. The official upstream submodule URLs must be reachable (network/proxy).
`unilidar_sdk2` is a protocol reference, not a linked vendor binary. Initialize it
only if needed for development. The external Python splatting checkout is not
needed for the integrated C++ pipeline and can remain uninitialized.

ROS, PCL, Python, CUDA, OpenCV, Grafana, and Node.js are not required to compile
this application. Grafana is needed separately only for dashboard viewing.
RealSense support is ON in the shared presets. For a Unitree-only build, pass
`-DVISTA_ENABLE_REALSENSE=OFF` during configuration to omit that SDK build.

## 3. Visual Studio

1. Use **File > Open > Folder** and choose `VISTA/embedded_C`.
2. Select `windows-vs2019` or `windows-vs2022`, matching your installation.
3. Configure and build, then choose `vista_edge` as the startup target.
4. Select Debug for debugging. For a shared release, select Release.

If an older VS 2019 installation does not offer presets, update its CMake tools
or use the terminal commands below. You can open the generated solution in
`out/build/windows-vs2019` after configuring; never commit that solution.

## 4. VS Code

1. Open the repository's `VISTA-CPP.code-workspace` file.
2. Install the recommended C/C++ and CMake Tools extensions.
3. Run **CMake: Select Configure Preset**, then select your VS version.
4. Run **CMake: Configure**, select the matching Debug/Release build preset,
   and run **CMake: Build**.
5. Set `vista_edge` as the launch target and use **CMake: Debug**. Run tests
   through CMake Tools/CTest. No absolute compiler path is stored in the repo.

## 5. Terminal equivalents

From `embedded_C`, for VS 2019:

```powershell
cmake --preset windows-vs2019
cmake --build --preset windows-debug --parallel 4
ctest --preset windows-debug
cmake --build --preset windows-release --parallel 4
ctest --preset windows-release
```

For VS 2022, use `windows-vs2022` to configure and
`windows-vs2022-debug` / `windows-vs2022-release` to build/test. Reduce parallelism
if memory is limited. The current cleanup validation machine has VS 2019 only;
the 2022 preset is supplied but has not been compiled on that machine.

Debug output: `embedded_C/out/build/<configure-preset>/Debug/vista_edge.exe`.
Windows Release output: `publish/windows-x64/vista_edge.exe`, linked directly
there with static MSVC runtime. Release packaging preserves the two existing
TXT configs. Do not build two Release trees into that same package concurrently.
Use `-DVISTA_AUTO_PUBLISH=OFF` for an isolated diagnostic build; its executable
stays in the selected build directory instead.

At first application startup, missing `DeviceConfig.txt` and `GrafanaSecret.txt`
are created beside the executable. Review DeviceConfig and restart if prompted.
The secret template is blank; set your own Grafana token locally. Runtime
configuration is not read from the source root. Defaults live in the versioned
`embedded_C/config/DeviceConfig.defaults.txt`. Never commit a real token.

## 6. Native Linux builds

Use Ubuntu 20.04 or newer on x86-64 or ARM64 with a C++17 compiler. See
[the ADLINK guide](ADLINK_UBUNTU20_BUILD.md) for hardware/backend details.

```bash
sudo apt-get update
sudo apt-get install build-essential cmake git pkg-config libusb-1.0-0-dev libudev-dev
cd embedded_C
cmake --preset linux-x86_64-release
cmake --build --preset linux-x86_64-release --parallel 2
ctest --preset linux-x86_64-release
```

Substitute `linux-arm64-release` on ARM64; it is a native ARM preset, not an
x86-to-ARM cross compiler. Ubuntu 20.04's stock CMake 3.16 supports the source
project but not presets. With that CMake use explicit configuration instead:

```bash
cmake -S . -B out/build/native-release -DCMAKE_BUILD_TYPE=Release -DVISTA_BUILD_TESTS=ON
cmake --build out/build/native-release --parallel 2
cd out/build/native-release
ctest --output-on-failure
```

The ARM USB backend defaults
to RSUSB. Without a camera, add `-DVISTA_ENABLE_REALSENSE=OFF` to save build time.
Release packages go to `publish/linux-x86_64` or `publish/linux-arm64`. Debug
does not update release packages. Linux packages may include runtime libraries;
the Windows single-EXE rule does not apply to Linux.

## 7. What the maintainer must commit before teammates pull

Commit a consistent set, not only the executable:

- All changed/new C++ sources, headers, models, tests, `CMakeLists.txt`,
  `CMakePresets.json`, `cmake/`, `config/DeviceConfig.defaults.txt`, and docs.
- `.gitmodules`, required submodule gitlinks, `.gitignore`, `.gitattributes`,
  this guide, and the portable VS Code files/workspace.
- `publish/` binaries, Linux runtime libraries, default/sample configs,
  build information, and dependency licenses if clone-and-run is promised.
- Updated dashboard JSON, plugin source, lockfiles, tests, and `dist` assets
  when delivering the matching visualization changes.

Check `git status --short --untracked-files=all` before committing: a new
header or CMake helper left untracked can make your build pass while a teammate's
fails. Review the diff and secret exclusions; then stage only the relevant paths.
Do not commit RAW recordings, generated maps, `node_modules`, `target`, `bin`,
`obj`, `.vs`, `out`, personal `CMakeUserPresets.json`, or `GrafanaSecret.txt`.
`.gitignore` does not untrack files already committed.

Changes inside an external submodule are a separate Git repository: commit and
push them to a team-accessible remote/fork first, then commit the parent gitlink
and its URL if changed. A parent commit alone cannot publish untracked files or
local-only commits inside `external/lidar_surface_splatting`.

Before publishing, test in a second checkout with empty build directories and
the pinned dependencies. `cmake --build` must succeed without files from the
maintainer's original build cache. Unit tests do not prove live hardware timing,
serial driver availability, IMU calibration, or final LIO trajectory accuracy.

## Cleanup policy

Generated build checks, Rust `target`, IDE caches, and plugin `node_modules`
can be regenerated. Keep sources, lockfiles, test data you intend to share,
deployment `dist`/`publish`, and all captures/maps you still need. Removing an
unused legacy submodule does not remove its earlier Git history. The Rust
reference and uncommitted external-viewer changes are intentionally preserved.
