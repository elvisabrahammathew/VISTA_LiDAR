# Install lidarpointcloud on local Grafana for Windows

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
