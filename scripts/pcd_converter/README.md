# PCD Converter and Gaussian Splatting — Visual Studio 2019

A small English-language WPF application for Windows. It converts VISTA ASCII
PCD maps to TXT and controls the existing Gaussian splatting repository.
VISTA's C++ application, DeviceConfig.txt and Grafana are not changed.

## Build requirements

- **Visual Studio 2019**, preferably the installed 16.11 release.
- **.NET desktop development** workload.
- **.NET Framework 4.7.2 targeting pack**, available in Visual Studio Installer's
  Individual components. This targeting pack was already installed on the
  development machine, so this version does not require another SDK installation.
- The projects use the classic .NET Framework project format and C# 8 syntax.
  There are no NuGet dependencies and no SDK-style project requirement.

**No .NET 8 SDK, Visual Studio 2022, or global.json is required.**
The previously installed .NET 8 SDK can remain installed; there is no need to
uninstall it. The converter itself runs on .NET Framework 4.7.2 or newer, including
machines with .NET Framework 4.8/4.8.1 installed.

Gaussian processing still needs **Node.js 22.12 or newer, npm, and uv**. Setup uses
uv to provision Python 3.12 and the repository's locked dependencies. These tools
are not required for plain PCD-to-TXT conversion. Initial setup needs internet.

## Open and debug in Visual Studio 2019

1. Close the old unloaded PcdConverter solution, then reopen it. If Visual Studio
   offers to reload the changed projects, accept that reload.
2. Choose **File > Open > Project/Solution**, not Open Folder, and open:
   `D:\MASTER\VISTA\LiDar\Repo\scripts\pcd_converter\PcdConverter.sln`.
3. Both **PcdConverter** and **PcdConverter.Tests** should load. If they still show
   `(unloaded)`, right-click each one and select **Reload Project**.
4. Right-click **PcdConverter** and select **Set as Startup Project**.
5. Select **Debug / Any CPU** and use **Build > Rebuild Solution**.
6. Press **F5**. The main window opens immediately; no configuration file is needed.
7. Use **Select Repository...** to choose the Gaussian Splatting repository, or
   use plain **Convert** without selecting a repository.

The debug EXE is `bin/Debug/PcdConverter.exe`. The release EXE is
`bin/Release/PcdConverter.exe`. The old `net8.0-windows` output subfolder is no longer
used. The solution is independent of the C++ solution under `embedded_C`.

If a framework targeting pack is missing (MSB3644), use Visual Studio Installer >
Modify > Individual components to install the **.NET Framework 4.7.2 targeting
pack**. A newer runtime does not replace a particular version's targeting pack.

## Build, test and publish from PowerShell

```powershell
Set-Location "D:\MASTER\VISTA\LiDar\Repo\scripts\pcd_converter"
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\build.ps1
```

The script finds VS2019 MSBuild, rebuilds both projects, runs the offline tests,
and copies the application into `publish`. No repository configuration is generated.

Optional explicit MSBuild path:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\build.ps1 -MSBuildPath "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\MSBuild\Current\Bin\MSBuild.exe"
```

For a Debug build, add `-Configuration Debug`. Do not use `dotnet run` or the old
`-DotNetPath` argument for this version. No .NET SDK is needed.

Distribute these **two files together**:

- `publish/PcdConverter.exe`
- `publish/PcdConverter.exe.config`

The second file is the .NET Framework runtime configuration. Keep it beside the
EXE. It is not SplattingConfig.txt: that old repository configuration is no longer
read or created. Repository selection is performed directly in the UI.

To clean later, close the converter, then use **Build > Clean Solution**, followed
by **Build > Rebuild Solution**. There is no reason to delete map files or Gaussian
dependencies.

## Select the Gaussian Splatting repository

1. Launch PcdConverter.exe from a writable folder, not Program Files.
2. The window opens immediately. It restores the last valid repository you selected
   under this Windows user account. On first use, or if the saved folder is no
   longer valid, plain conversion remains available while Gaussian controls stay
   disabled until you select a valid repository.
3. Click **Select Repository...** and choose its root folder, for example:
   `D:\MASTER\VISTA\LiDar\Repo\external\lidar_surface_splatting` or
   `D:\MASTER\VISTA\LiDar\Test\lidar_surface_splatting-main`.
4. The app checks package.json, package-lock.json, pyproject.toml, uv.lock and the
   Python CLI before accepting the folder. Selection alone does not run commands.
5. If you choose another valid repository, this app stops the previous viewer
   before switching paths. An invalid selection leaves the old repo/viewer intact.

The last successfully selected valid path is remembered in **Windows user preferences**:
`HKEY_CURRENT_USER\Software\VISTA\PcdConverter`, value `LastRepositoryPath`.
Debug and published builds share this preference for the same Windows account.
No SplattingConfig.txt, replacement settings file, or preference inside the repo is
written. Existing old TXT configurations are not used.

On reopening, the path is restored and Select Repository... starts at that folder.
Restoring a path never automatically runs setup, generation or the viewer. If the
folder was moved/deleted, the app shows a warning and asks you to select again;
the folder picker starts at the nearest existing parent if possible. Invalid or
cancelled selections do not overwrite the previous saved path. If Windows denies
access to preferences, the app logs a warning but can still use a repo for that session.

## Choose the viewer port

**Viewer Port** defaults to **5173**. Enter a whole number from **1024 to 65535**,
for example **5180**. A valid value is remembered when you leave the field, start
the viewer, or close the app. It is restored at next launch, without starting any
tools. It uses the same Windows user preference key as the repo, value `ViewerPort`;
there is no TXT configuration. An invalid saved preference falls back to 5173.

Start Viewer and Convert and Generate use this port; readiness checks, the displayed
address and Open Viewer all use the port of the running viewer. The port field is
locked while a task or viewer is running. Use Stop Viewer before changing it.
An invalid port disables Start Viewer and Convert and Generate, not plain Convert.
If the port is occupied, the app reports it instead of choosing another port or
terminating its owner. Choose another port and try again. There is no need to edit
vite.config.ts or rebuild this app. These settings affect serving only, not the
generated PLY/TXT files. The viewer still listens only on 127.0.0.1.

## Use the application

1. **Check Environment** verifies Node.js, npm and uv without installing packages.
2. After choosing a repository, **Setup / Repair Environment** stops this app's viewer, then runs:

   ```text
   uv sync --locked --python 3.12
   npm ci
   ```

   Run once per repo or when dependencies change. It requests confirmation before
   downloading packages/replacing node_modules. Only configure a trusted repo.
   Missing Node.js/uv must be installed separately; restart the application afterward.
3. **Select PCD...** chooses a completed room-map export.
4. **Output Format** currently contains only `.txt`. Output is next to the input;
   its base filename stays the same: `RoomMap.pcd` becomes `RoomMap.txt`.
5. **Convert** performs only the format selected in Output Format (currently TXT).
   It works without a Gaussian repository. Existing TXT requires an
   overwrite confirmation. The original PCD is never modified.
6. **Convert and Generate** asks for confirmation, converts the PCD, then runs:

   ```text
   uv run --locked --python 3.12 lidar-splat convert <input.txt> <repo>/out/<name>-optimized.ply --intensity-encoding uint8 --stage
   ```

   PLY variants, diagnostics and JSON origin metadata are saved in the selected
   repository's `out` folder. Viewer PLYs are staged in `public`. Existing generated
   files/viewer assets may be replaced after confirmation. The viewer is stopped
   during generation to avoid serving partially written assets.
7. The viewer starts after generation, or use **Start Viewer** for staged assets:

   ```text
   npm run dev -- --host 127.0.0.1 --port <ViewerPort> --strictPort
   ```

8. **Open Viewer** opens `http://127.0.0.1:<ViewerPort>/` in your default browser once
   ready, for example `http://127.0.0.1:5180/` if you entered 5180.
9. **Stop Viewer** stops this app's npm/Node/Vite tree. Closing the WPF window also
   cancels its active task and stops its viewer. Browser tabs remain open and a
   previously loaded scene may stay visible until refreshed.

Commands run in the selected repository. npm is invoked through its actual
JavaScript CLI with node.exe, as npm.cmd does, without constructing shell commands.
Output appears in Execution Log. File paths, including spaces, quotes and shell
metacharacters, are passed as literal Windows arguments.

## Data and safety limits

- Supports **DATA ascii** PCD with scalar x, y, z and intensity (finite, 0-255).
  Extra fields and reordered fields are supported. TXT is headerless `x y z intensity`.
- Original numeric text and point order are preserved. Missing intensity is
  rejected, not invented. Binary/compressed PCD is explicitly unsupported for now.
- Conversion streams the input. Point counts and non-finite values are checked.
  A same-folder temporary file is committed only after success. A failed/cancelled
  conversion does not replace an existing TXT. Use a completed PCD, not a file
  still being written by another program.
- TXT conversion has no point cap. Gaussian generation retains upstream defaults,
  including its one-million-input-point sampling cap. Large reconstructions can
  consume substantial CPU/RAM.
- Cancel Task stops the current task. Upstream setup/generation is not transactional;
  partial PLYs, public assets or downloads may remain. Run setup/generation again.
- Windows Job Objects supervise owned process trees. Stopping the app's viewer does
  not terminate unrelated Node processes or browser windows. A busy port is reported
  instead of killing its owner or silently choosing another port.
- Do not run multiple app instances against the same repo: they share .venv,
  node_modules and public assets.
- All application-authored text is English. External output is shown unchanged.
- The upstream lockfile reported a high-severity `source-map-js` npm advisory,
  GHSA-68fv-2mgg-jv7q. No automatic dependency fix is applied. Review that repository
  separately. The viewer is bound to 127.0.0.1 only.

## Tests

`build.ps1` runs the offline test executable automatically. After building:

```powershell
.\Tests\bin\Release\PcdConverter.Tests.exe
.\Tests\bin\Release\PcdConverter.Tests.exe --real-pcd "D:\path\to\RoomMap.pcd"
```

The test runner prints its isolated temporary artifact directory and renders a WPF
preview there. It covers port validation/persistence, custom-port HTTP startup,
port edit locks, busy-port refusal, saved-folder restoration, moved/missing folders, folder
selection, repo validation, conversion without a
repo, switching repos while a viewer is active, overwrite/cancellation,
Windows argument quoting, stdout/stderr draining, parent/child/grandchild cleanup,
occupied ports, and closing idle/active WPF windows. No NuGet test package is needed.
The optional integration mode additionally installs dependencies and overwrites
generated assets; use it only with a disposable copy of the Gaussian repository.

Preference tests use unique temporary keys under the application's Tests subkey,
then remove those keys. They do not modify the real LastRepositoryPath or ViewerPort preferences.
