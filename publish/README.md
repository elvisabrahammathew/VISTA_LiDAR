# VISTA Edge - runnable C++ Release builds

Clone the repository, select the folder matching your machine, review
`DeviceConfig.txt`, then run the program. No compiler or initialized source
submodules are needed to run a checked-in executable. Sensor drivers/permissions
and a separately configured Grafana server are still required where applicable.

| Folder | Target | Start |
| --- | --- | --- |
| `windows-x64` | Windows 10/11, x64 | Double-click `vista_edge.exe` |
| `linux-x86_64` | Linux x86-64 | `./publish/linux-x86_64/vista_edge` |
| `linux-arm64` | Linux AArch64 / ADLINK | `./publish/linux-arm64/vista_edge` |

The initially supplied Linux builds target Ubuntu 20.04 (glibc 2.31). See each
folder's `BUILD_INFO.txt` for its actual build OS, compiler, checksum and enabled
RealSense support. Rebuilding on a newer Linux distribution may increase the
minimum system requirement. ARM64 is not 32-bit ARM. Native ADLINK hardware
validation is separate from cross-build and emulated tests.

Windows has exactly three runtime files: `vista_edge.exe`, `DeviceConfig.txt`
and `GrafanaSecret.txt`. Its build metadata, import report and default template
are outside that app folder, under `build-info/windows-x64/`. Its runtime is
statically linked, so no Microsoft runtime DLLs need to travel beside the EXE.

## Settings and data

- Edit `DeviceConfig.txt` **in the folder of the executable you run**, not in
  `embedded_C/config/`. Serial ports and network addresses must match your setup.
  Linux defaults to `/dev/ttyACM0`; Windows uses the source template's COM port.
- Builds create `DeviceConfig.txt` only if missing. Existing settings are never
  overwritten. `DeviceConfig.defaults.txt` is refreshed with the current source
  defaults on every publish, so you can compare or reset settings deliberately.
  For Windows this template is in `build-info/windows-x64/`; Linux keeps it
  beside the executable.
- The real `GrafanaSecret.txt` is private and ignored by Git. Windows publishing
  creates it with a blank token only if missing; first startup also restores it
  after cloning. Linux retains its `GrafanaSecret.example.txt` template.
  Never force-add a real token. No build copies the developer's secret file.
- RAW, PCD and map output from these three folders uses **this clone's `data/`**,
  independent of the builder's absolute path and the shell working directory.
  Keep the `publish/<platform>` layout if moving the repository.
- A virtual serial replay must use `InitializeOnConnect(unitree-l2): 0`.
  Hardware startup defaults may configure/restart a real Unitree; review settings
  before starting. This packaging operation itself never opens a sensor port.
- Grafana/plugins/dashboard are not embedded in the executable. They remain
  available in `dashboard/`; configure/install them separately on Windows or
  your selected Grafana host. Published programs are not a Grafana installer.

## Automatic publishing after a build

Successful **Release** builds refresh only the matching platform folder. Debug
and RelWithDebInfo never replace the checked-in Release program. **Windows Release
links directly into `publish/windows-x64/`**, including when you build only the
`vista_edge` target or select Release and press F5 in Visual Studio. Debug keeps
its original `embedded_C/out/build/windows-vs2019/Debug/` executable/config path.
Old executables remaining under `out/build/.../Release/` from earlier builds
are no longer updated; use the executable in `publish/windows-x64/`.

From `embedded_C/` on Windows:

```powershell
cmake --preset windows-vs2019
cmake --build --preset windows-release --parallel 2
```

Building only `vista_edge` also publishes after a relink. To restore a deleted
publish executable without changing C++ source, build the `vista_publish` target:

```powershell
cmake --build out/build/windows-vs2019 --config Release --target vista_publish
```

On Ubuntu 20.04 ARM64, `bash embedded_C/scripts/build_adlink.sh` builds, publishes
and runs the existing tests. Generic native Linux commands (from `embedded_C/`):

```bash
cmake -S . -B out/build/publish-release -DCMAKE_BUILD_TYPE=Release \
  -DVISTA_BUILD_TESTS=ON -DVISTA_ENABLE_REALSENSE=OFF
cmake --build out/build/publish-release --parallel 2
(cd out/build/publish-release && ctest --output-on-failure)
```

The native processor selects x86-64 or ARM64 automatically. The example disables
RealSense for a Unitree/Quanergy-only package; use `VISTA_ENABLE_REALSENSE=ON` when
L515 support is required and its build dependencies are installed. Existing
presets retain their original SDK defaults. A Windows build does not create or
update Linux binaries: build each platform natively or with a suitable toolchain.

Use `-DVISTA_AUTO_PUBLISH=OFF` for temporary/diagnostic build directories that
must not change versioned packages. Stop the published program before rebuilding
on Windows, because Windows locks an executable while it runs.

## Version control and dependencies

Linux shared libraries are included where needed. Linux glibc and the dynamic
loader come from the target OS, not this folder. Windows Release uses `/MT`
consistently across VISTA, Point-LIO, RealSense and tests; Debug retains `/MDd`.
Its EXE still uses ordinary Windows system DLLs supplied by Windows itself.
Linux published executables have unneeded/debug symbols stripped to reduce Git
storage; the original build executables and test binaries are not stripped.
On Linux, the executable searches beside itself using an origin-relative RPATH.
If executable permissions are lost after copying from Windows, run
`chmod +x publish/linux-*/vista_edge`.

After a verified build, review `git diff` and commit the matching `publish/`
files **together with the source changes they were built from**. Auto-publish
does not automatically commit or push. Package metadata says whether RealSense
is compiled in; do not rename one architecture's binary into another folder.
Do not commit local settings containing sensitive data. When loading maps, use
paths valid on the deployment machine, not private absolute paths from another PC.

Source and license references remain in the same repository: root `LICENSE`,
`third_party/point_lio_unilidar`, `third_party/eigen`,
`third_party/boost_preprocessor`, and `third_party/librealsense` if enabled.
Bundled GCC runtime libraries use the GCC Runtime Library Exception; Microsoft
runtime components remain under Microsoft's redistribution terms. Retain the repository
and the applicable license notices when redistributing these packages.

Existing dependency license/copyright notices are retained under `licenses/`.

Automatic packaging is not automatic test approval. Review test results and
`BUILD_INFO.txt` (under `build-info/windows-x64/` for Windows) before committing binaries. A successful compile does not prove
LiDAR calibration, localization accuracy or actual ARM sensor access.
