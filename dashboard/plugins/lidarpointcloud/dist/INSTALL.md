# Install lidarpointcloud on local Grafana for Windows

## Updating an existing installation to 1.1.0

1. Use the newly built `dist` in this repository (no rebuild needed if supplied).
2. Stop the Grafana service, then copy all contents of `dist` over the existing
   `data/plugins/vista-lidarpointcloud-panel` folder. Do not create a nested `dist` folder.
3. Start Grafana and refresh the browser with Ctrl+F5. Plugin details must show 1.1.0.
4. Import the updated `dashboard/VISTA_Live_Dashboard_V4_Free_Space.json`,
   overwrite dashboard UID `vistalive`, and rebuild/run the updated C++ program.
5. Room Map connects to `ws://127.0.0.1:8766`; zoom/pan automatically requests LOD.
   Port 8765 remains the live processed cloud. Server and panel budgets only limit
   the current rendered view, not all disk/PCD points.

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
