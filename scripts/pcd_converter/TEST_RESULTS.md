# Verification — selectable viewer port, 2026-10-16

## Build and project loading

- Classic .NET Framework WPF/C# projects, target **.NET Framework 4.7.2**, C# 8.0.
- Built with the installed **Visual Studio 2019 Community 16.11.60** toolchain
  and its MSBuild 16.x, not with a .NET SDK.
- Release solution rebuild: passed with no warnings/errors.
- Debug solution rebuild: passed with no warnings/errors.
- Visual Studio 2019's `devenv.com /Build Release` loaded both projects and
  completed successfully. This verifies IDE compatibility, not just compilation.
- Automatic `build.ps1` discovery of VS2019 MSBuild, tests and deployment: passed.
- No global.json, SDK-style project, .NET 8 runtime, or NuGet dependency is required.
- No machine-wide framework/SDK installation was needed. The 4.7.2 targeting pack
  was already installed. The app also runs on the machine's newer 4.x runtime.

## Tests

- Default offline tests: **45 passed, 0 failed** in Debug.
- Release tests plus a real PCD and full integration: **47 passed, 0 failed**.
- WPF layout rendered and visually checked after the migration.

Coverage includes folder selection/validation, path handling, locale-neutral
conversion, field ordering, exact preservation of XYZ/intensity text, overwrite
protection, malformed data, cancellation and temporary-file cleanup.

The migration additionally verifies Windows argument quoting for empty values,
embedded quotes, trailing backslashes, spaces and metacharacters; complete stdout
and stderr draining; owned descendant cleanup after a parent exits; idle-window
closure; active-command cancellation on close; viewer child/grandchild cleanup;
and an unrelated-process control. New UI tests verify configuration-free startup,
plain conversion without a repo, controls enabling after selection, invalid-folder
selection preserving the current viewer, and valid repo switching stopping it.
Windows Job Objects, not a broad node.exe kill,
remain responsible for terminating owned tool processes.

Preference tests verify that reading an absent value creates no key; saved paths
survive a new history/window instance; Unicode and literal percent signs survive;
invalid choices cannot overwrite the last valid choice; malformed values are
ignored; moved folders fall back safely; the folder picker reopens at the saved
location or nearest existing parent; and a restored path does not auto-run tools.
Tests use isolated, uniquely named registry subkeys and remove them afterward,
without reading or changing the user's production LastRepositoryPath value.

New port coverage checks 1024/65535 boundaries, invalid text/ranges, saved-value
fallback, no interference with the saved repository, save on focus loss/close,
restoration without running tools, invalid values not blocking plain conversion,
custom-port HTTP startup/readiness/address, locking during tasks/viewer operation,
unlocking and restarting on another port, and busy-port refusal without killing
its owner. The offline HTTP helper is an owned process, not a browser or real Vite.
Registry tests use isolated keys, never the production ViewerPort value.

## Real Gaussian workflow

- Input: VISTA `RoomMap_20261006_170139.pcd`, **1,393 ASCII XYZI points**.
- Every point's numeric text/order was preserved in TXT. The original PCD was not edited.
- Real `uv sync --locked --python 3.12` and `npm ci` completed.
- Real `lidar-splat convert ... --intensity-encoding uint8 --stage` completed.
- Generated optimized/uniform/surface/points PLYs, diagnostics and origin metadata.
  Optimized output contained **1,338 Gaussian records** for this input.
- Real npm/Vite server started on a temporary loopback port. HTTP checks retrieved
  its HTML and staged PLY. Stopping it cleared its listening port.
- Integration ran in a workspace-local disposable copy, not the original Gaussian
  submodule/download. It is a command/serving test, not a reconstruction-accuracy
  or new browser-rendering assessment.

Tools: Node.js 24.19.0, npm CLI 11.17.0, workspace-local uv 0.13.1, Python 3.12.14,
locked NumPy 2.5.3 and SciPy 1.18.1.

## Deployment and limits

Ship `PcdConverter.exe` and `PcdConverter.exe.config` together. SplattingConfig.txt
is no longer read or created. The English UI offers Select Repository... and a
generic Convert button. The selected repo is remembered per Windows user at
HKCU\Software\VISTA\PcdConverter (LastRepositoryPath), not in a TXT file. Debug
output lives directly under bin/Debug. Restoring the repo does not start the viewer.
Viewer Port is editable (1024–65535), defaults to 5173, and is remembered as the
ViewerPort DWORD in that same per-user key. HTTP readiness, the displayed address,
and Open Viewer follow the running viewer's selected port. Invalid input disables
viewer/generation only; running tasks/viewer lock port editing. Stop Viewer unlocks
it. The app retains strict-port behavior and never kills another port's owner.

Existing functional limits remain: ASCII XYZI PCD, intensity 0-255; no invented
intensity or binary/compressed PCD support; upstream Gaussian sampling defaults
remain unchanged. The upstream lock reports one high-severity `source-map-js`
advisory, GHSA-68fv-2mgg-jv7q. No automatic upstream dependency changes were made.
The viewer remains loopback-only. C++ sources and Grafana were not changed.
