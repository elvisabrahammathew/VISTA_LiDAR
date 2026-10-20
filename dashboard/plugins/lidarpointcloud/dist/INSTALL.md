# Install lidarpointcloud on local Grafana for Windows

## Updating an existing installation to 1.2.0

1. Use the newly built `dist` in this repository (no rebuild needed if supplied).
2. Stop the Grafana service, then copy all contents of `dist` over the existing
   `data/plugins/vista-lidarpointcloud-panel` folder. Do not create a nested `dist` folder.
3. Start Grafana and refresh the browser with Ctrl+F5. Plugin details must show 1.2.0.
4. Import `dashboard/VISTA_Live_Dashboard_V5_Gaussian_Map.json` (V4 is kept as a previous file),
   overwrite dashboard UID `vistalive`, and rebuild/run the updated C++ program.
5. Room Map connects to `ws://127.0.0.1:8766`; zoom/pan automatically requests LOD.
   Gaussian Map connects to `ws://127.0.0.1:8767` and draws GSP1 covariance ellipses.
   Live sensor 3D publishing on 8765 is retired; LiDAR/IMU health stays active.
   Server/panel budgets limit the visible view, not stored geometry.
6. For ADLINK at 192.168.10.2, bind both map sockets to 192.168.10.2 (or 0.0.0.0)
   in DeviceConfig.txt and use ws://192.168.10.2:8766 and :8767 in the dashboard.
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
