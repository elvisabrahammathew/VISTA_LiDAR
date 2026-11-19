# Install lidarpointcloud on local Grafana for Windows

## 1.10.0 — Minimum LiDAR correspondences

Update VISTA Edge and replace the installed panel plugin with the rebuilt
`dist` contents. Restart Grafana and refresh the browser. The existing **VISTA
Edge Settings** dashboard reads **Minimum LiDAR correspondences** in **Point-LIO
· Matching and Timing** when the connected edge reports the setting. Apply
updates the running session and restarts the LIO and mapping workers. Set
`LioMinCorrespondences` in the edge's `DeviceConfig.txt` to use it at startup.

## 1.4.2 — Readable, aligned map controls (18 November 2026)

The map toolbar explicitly sizes native controls and resets inherited form
styling. Full Surface / Edges stays readable; checkboxes and Reset view align
vertically and wrap as complete groups on narrow panels. Replace the existing
plugin's contents with the rebuilt `dist`, restart Grafana, then use Ctrl+F5.
Verify installed version 1.4.2. No ADLINK rebuild or dashboard import is required.

## 1.4.1 — Edges by default (18 November 2026)

New panels and panels without a saved surface mode now default to **Edges**.
The supplied V5/V6 dashboard JSONs also select Edges. A previously saved
**Full Surface** choice is preserved: change the toolbar dropdown to Edges and
save the dashboard, or reimport the updated JSON. Replace the plugin with the
rebuilt `dist` contents, restart Grafana and hard-refresh the browser.
The C++ 1.4.0 edge-cache/protocol implementation needs no further rebuild for
this default-only update. Foreground remains independent.

## 1.4.0 — Full Surface / Edges (18 November 2026)

The Gaussian/Foreground panel has a **Display** dropdown with exactly two modes:
**Full Surface** and **Edges**. The Foreground checkbox remains independent.
The selected mode is a panel option; save the dashboard to retain your choice.

This update requires **both** the rebuilt panel and the updated C++ `vista_edge`
on ADLINK. Build/restart the edge application, replace the contents of the existing
Grafana `vista-lidarpointcloud-panel` folder with this repository's `dist` contents,
restart Grafana and hard-refresh the browser. No new plugin ID, port, worker,
DeviceConfig setting or VISTA Live data-source update is required. Existing V5/V6
dashboards acquire the dropdown with this plugin; see 1.4.1 above for defaults.

Edges are supported geometric creases, curved/linear structures and one-sided
surface boundaries, not semantic object detections. Sparse/noisy surfaces may
have incomplete or false edges. RoomMap, PCD export, localization and Foreground
subtraction continue using complete geometry. The extra edge selection is fitted
once per changed tile with neighboring support and cached on disk, then selected
before GSP1 transmission. Switching modes does not reconnect Foreground. New
clients identify edge support in server metadata; older applications continue
to serve Full Surface and show an update message if Edges is requested.

For your current network, use ADLINK `192.168.137.154` on ports 8766/8767/8768;
Grafana remains on Windows `192.168.137.1`. Earlier version notes below include
historical example addresses and installation instructions.

## Updating an existing installation to 1.3.0

1. Use the newly built `dist` in this repository (no rebuild needed if supplied).
2. Stop the Grafana service, then copy all contents of `dist` over the existing
   `data/plugins/vista-lidarpointcloud-panel` folder. Do not create a nested `dist` folder.
3. Start Grafana and refresh the browser with Ctrl+F5. Plugin details must show 1.3.0.
4. Import `dashboard/VISTA_Live_Dashboard_V5_Gaussian_Map.json` (V4 is kept as a previous file),
   overwrite dashboard UID `vistalive`. This overlay-only change does not require a C++ rebuild.
5. Room Map connects to `ws://127.0.0.1:8766`; zoom/pan automatically requests LOD.
   Gaussian Map connects to `ws://127.0.0.1:8767` and draws GSP1 covariance ellipses.
   Live sensor 3D publishing on 8765 is retired; LiDAR/IMU health stays active.
   Server/panel budgets limit the visible view, not stored geometry.
   V5 now shows **Gaussian Room Map and Foreground** in one panel: main URL :8767,
   Foreground WebSocket URL :8768, independent checkboxes and one shared camera.
   The separate RoomMap LOD panel stays on :8766. The old standalone Foreground
   panel is removed; all numeric health/worker panels remain.
6. For ADLINK at 192.168.10.2, bind both map sockets to 192.168.10.2 (or 0.0.0.0)
   in DeviceConfig.txt and use ws://192.168.10.2:8766 and :8767 in the dashboard.
   Also bind the Foreground socket and set the overlay's Foreground URL to
   ws://192.168.10.2:8768. All streams must share the same world frame.
   Keep Grafana, this panel and vistalive on Windows. Allow only the trusted LAN
   through the firewall: these map WebSockets do not provide authentication/TLS.

If `data/plugins/vista-lidarpointcloud-panel-v1` also contains a plugin.json with
ID `vista-lidarpointcloud-panel`, stop Grafana and move that backup folder OUTSIDE
`data/plugins`. Keep only one installed folder declaring this ID. Renaming a
folder inside `plugins` does not change the plugin ID or disable its loading.
Do not replace the separate `vistalive` data-source folder for this update.

## First installation

1. Build the plugin with `pnpm install` and `pnpm run build`.
2. Create `C:\Program Files\GrafanaLabs\grafana\data\plugins\vista-lidarpointcloud-panel`.
3. Copy the **contents** of `dist` into that folder. `plugin.json` and `module.js` must be directly inside the plugin folder.
4. Open `C:\Program Files\GrafanaLabs\grafana\conf\custom.ini` as Administrator.
5. Under `[plugins]`, add `vista-lidarpointcloud-panel` to `allow_loading_unsigned_plugins`.
6. Restart the Windows service named `Grafana`.
7. In Grafana, open **Administration → Plugins and data → Plugins** and verify `lidarpointcloud` is listed.
8. Add a visualization, select `lidarpointcloud`, and configure its source.

Example when keeping the earlier unsigned panel enabled:

```ini
[plugins]
allow_loading_unsigned_plugins = vista-pointcloud-panel,vista-lidarpointcloud-panel
```
# Foreground behavior in the overlay

Replace the contents of the Grafana plugin folder `vista-lidarpointcloud-panel`
with this plugin's rebuilt `dist` contents and restart Grafana. Reimport the
existing V5 dashboard JSON (same UID). No new plugin ID or unsigned-plugin entry
is required. `vistalive` is unchanged. Foreground uses `ws://127.0.0.1:8768`
and clear on disconnect or 1000 ms without frames. Room Map/Gaussian Map retain
their views. When viewing a remote ADLINK host, change the new panel URL to
`ws://192.168.10.2:8768` and bind ForegroundWebSocketBindAddress on that host to
`192.168.10.2` (or a deliberately exposed interface); the browser connects directly.

The V5 panel is now **Gaussian Room Map and Foreground**. New stationary
objects and unmapped surfaces also appear; this is not a moving-object label.
Install this update directly even if you skipped 1.2.1; no intermediate import
or new plugin ID is needed.


## 1.3.1 — Full-panel sizing and V6 notes (18 November 2026)

Copy the rebuilt `dist` contents into the existing `vista-lidarpointcloud-panel`
installation, restart Grafana and hard-refresh the browser. No new plugin ID or
unsigned-plugin entry is needed. CSS canvas size now fills its panel independently
of device pixel ratio, Windows scaling and browser zoom. Both Room Map and the
Gaussian/Foreground overlay are covered, including dynamic panel resizing.

Reimport `dashboard/VISTA_Live_Dashboard_V6_Digital_Twin.json` (UID `vistalive`)
to move the two text notes to the bottom, side by side at 12 columns each, with
height 16. The current map URLs, queries, note text and other panel settings are
retained. Export your installed Grafana dashboard first if it has further edits.
The Digital Twin plugin and ADLINK runtime do not need an update for this fix.

## Gaussian corner radius

Set GaussianMapCornerRadiusMeters in DeviceConfig.txt beside vista_edge.
The default is 1.5 m. Choose Corners in the Gaussian panel and enter a radius
from 0 to 3 m. A radius of 0 shows the corner markers alone. The panel setting
applies to that panel's view, and Use device default restores the startup value.
Updating the radius does not change the stored Room Map or another viewer's
radius. Update both vista_edge and the Grafana plugin to use the radius input.

## Room Map Mount and LIO controls

Build the matching embedded_lidar application and replace the installed
lidarpointcloud plugin with this directory's dist contents. Restart Grafana
and reload the page. The controls appear on the Room Map WebSocket panel when
the application is running with Unitree L2, Grafana and live Room Map enabled.

Run vista_edge once to create MappingControlSecret.txt beside the executable.
Set MappingControlToken in that file to a private token (24–128 ASCII letters,
digits, hyphens or underscores). On Linux, generate one using openssl rand -hex 32.
Restart vista_edge normally to load the token. Keep the secret file local.
An existing nonempty VISTA_MAPPING_CONTROL_TOKEN environment variable takes
precedence over the file.

Enter the same token in the panel. Set all six Mount fields, then click Apply.
Toggling LIO immediately uses the last applied Mount, including when there are
unsubmitted edits in the fields. Keep the sensor steady while enabling LIO
until localization initializes. A fixed Mount assumes the sensor is stationary.
Each reset saves the confirmed PCD if it has points and starts another map.
The applied values are held in memory until the app restarts and reads
DeviceConfig.txt again.

Use the control token on the private wired network. Plain WebSocket traffic
is unencrypted. Use a TLS reverse proxy and a wss:// Room Map URL if the
network cannot be trusted. The token is different from the Grafana secret
and is not part of the dashboard JSON.

## VISTA Edge Settings dashboard

1. Build the matching embedded_lidar revision on the edge and deploy plugin
   from dist into the existing lidarpointcloud plugin folder on Windows.
   Restart Grafana and import VISTA_Edge_Settings.json. VISTA Sensor V5/V6
   include a link to this dashboard after reimport.
2. Set MappingControlToken in MappingControlSecret.txt beside vista_edge, then
   restart the app. This same token controls both Room Map pose and advanced
   settings. The settings WebSocket defaults to port 8769 and inherits the
   Room Map bind address. The optional SettingsWebSocketBindAddress and
   SettingsWebSocketPort can be set in DeviceConfig.txt before startup.
   In Grafana update the single Edge settings URL variable if required.
3. Enter that token in Capture Logging and click Connect. Every panel then
   reads the active values from the edge. Use Apply per panel. Edits survive
   dashboard navigation once applied. Restarting vista_edge reloads
   DeviceConfig.txt. The token stays in browser memory while the settings
   dashboard is open and is never stored in the dashboard JSON.
4. Camera and Radar currently use None. L515 depth controls are shown under
   LiDAR on builds that support the SDK. The LiDAR serial list is discovered
   by the edge. Map stream bind addresses and ports are shown as read-only.
   Use a relative file name under data/maps or an absolute path in that
   directory when selecting a reference PCD from the dashboard.

The CLI reports every effective setting change. Logging, telemetry and
foreground filter changes keep the existing Room Map. Changes to the LiDAR
source, Point-LIO settings or Room Map build/source settings can produce an
acquisition gap or start a separate Room Map. The previous confirmed PCD is
saved using the existing session-file rules. A LiDAR connection card may show
reconnecting after applying source settings.

Use the settings WebSocket on the private wired network or through a TLS
reverse proxy. A browser that opens Grafana via HTTPS generally needs a wss://
settings URL. Never add the secret to the URL.
