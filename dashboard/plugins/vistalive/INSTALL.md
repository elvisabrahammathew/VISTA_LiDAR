# VISTA Live: clear stale metrics

This companion **data source** plugin wraps the built-in Grafana Live source.
It requires no new port, server worker or publish field. It uses the existing
`stream/vista/grafana_bridge_health` heartbeat. After 6 seconds without a fresh
heartbeat, numeric/status panels, tables and graphs return **No data**. A 250 ms
browser timer performs expiry even if dashboard auto-refresh is off. The two
`lidarpointcloud` 3D panels are unchanged and retain their last geometry.

Keep the Windows/Grafana browser and Adlink calendar clocks synchronized.
Publish Grafana metrics every 1–2 seconds; intervals over 6 seconds will expire.
Live history remains available while the producer is connected. A Grafana
restart/page reload may discard Live history; this is not a persistent database.
This plugin does not detect a disconnected LiDAR if `vista_edge` keeps publishing.

## Build

From this folder: `npm ci`, `npm run typecheck`, `npm test`, `npm run build`.

## Install on the Windows Grafana host

### Update an existing installation (1.0.1)

Version 1.0.1 re-subscribes after both stream errors and normal completion. It
also emits independent ordinary data frames, avoiding Grafana 13.2.2's native
streaming fast path when recovering from an empty result. History rows, units,
labels and the six-second heartbeat timeout are preserved.

1. Back up the currently installed plugin outside Grafana's plugins directory.
2. Copy the new **dist contents** over the existing plugin directory. On this
   project host it is `C:\Program Files\GrafanaLabs\grafana\data\plugins\vistalive`.
   The folder name is not the plugin ID. Do not create a second directory with
   the same `vista-live-datasource` ID.
3. Restart Grafana, then hard-refresh the browser (Ctrl+F5).
4. No dashboard re-import, new data source, or 3D plugin update is needed if the
   dashboard already uses VISTA Live. Check installed plugin version **1.0.1**.
5. Stop/restart vista_edge: metrics should clear after six seconds, then resume
   when a fresh heartbeat and data arrive. Reconnect after a browser network
   interruption too. An expired Grafana login may require signing in again.

### First installation

1. Copy this folder's **dist contents** (module.js, plugin.json, img, etc.) into
   `C:\Program Files\GrafanaLabs\grafana\data\plugins\vista-live-datasource`.
   If Grafana uses a different `[paths] plugins` directory, use that directory.
2. In Grafana's active custom.ini, append the plugin ID to the existing list:
   `[plugins]` then `allow_loading_unsigned_plugins = vista-lidarpointcloud-panel,vista-live-datasource`.
   Preserve any other plugin IDs already present.
3. Restart the Windows Grafana service. You do not need Grafana on Adlink.
4. In Grafana: **Connections → Data sources → Add data source → VISTA Live**.
   Choose a name, e.g. **VISTA Live**, then **Save & test**.
5. Import `dashboard/VISTA_Live_Dashboard_V4_Free_Space.json`; select this
   **VISTA Live** source when the import asks for it. Keep UID `vistalive` and
   overwrite the existing dashboard. Do not select the built-in Grafana source
   for that import input, or expiry will not work.
6. Run vista_edge. Stop it: after about 6 seconds the metric panels should show
   No data, while both 3D views retain their last geometry. Restart it: metrics
   resume automatically. No update of the lidarpointcloud plugin is required.

Tested with Grafana 13.2.2. No user tokens belong in this plugin or dashboard.
