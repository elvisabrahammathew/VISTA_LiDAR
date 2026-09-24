# Install lidarpointcloud on local Grafana for Windows

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
