# Deploy V6: ADLINK Camera Gateway, Windows AI and Grafana

Prepared: 18 November 2026. Scope: one person with a body-relative mesh.
This deployment does not place the person in LiDAR map coordinates and does not use Virtual Ward.

## 1. Architecture and current status

```text
ADLINK: camera -> person_twin gateway :8100
                         | JPEG images      ^ mesh results
                         v                  |
Windows:            person_twin worker --worker lab --device cuda

Windows: Grafana/browser -> bridge 127.0.0.1:8101 -> ADLINK :8100/api/mesh
```

ADLINK is the only camera owner. YOLO, InstantHMR and mesh geometry generation run
on Windows. The ADLINK gateway receives mesh results from Windows and serves them
through its API. Grafana and interactive 3D rendering run on Windows. No AI worker
needs to run on ADLINK.

V6 retains UID `vistalive` and every V5 panel, query and option, and adds a row and
a `VISTA Digital Twin` panel. The V5 JSON file is unchanged. Importing V6 with the
same UID updates the existing dashboard; export your customized Grafana dashboard
before importing it.

The ADLINK camera, Windows CUDA execution and installed Grafana instance have not
been verified in this deployment. The plugin has been built; automated tests do
not replace testing on the actual hardware.

## 2. Test the panel before obtaining the models

### WINDOWS — Install the plugin

The prebuilt plugin is located at:
`D:\MASTER\VISTA\LiDar\Repo\dashboard\plugins\digitaltwin\dist`.

1. Identify the plugin directory used by your Grafana instance. A typical Windows
   installation uses `C:\Program Files\GrafanaLabs\grafana\data\plugins`.
2. Create a subdirectory named `vista-digitaltwin-panel`. Copy the **contents** of
   `dist` into it. `module.js` and `plugin.json` must be directly inside that folder.
3. In the active Grafana configuration file, usually `conf\custom.ini`, append
   `vista-digitaltwin-panel` to the existing `allow_loading_unsigned_plugins` list
   under `[plugins]`. Preserve the existing LiDAR and data source plugin IDs. Example:

```ini
[plugins]
allow_loading_unsigned_plugins = vista-lidarpointcloud-panel,vista-live-datasource,vista-digitaltwin-panel
```

4. Restart the Grafana service and hard-refresh the browser. The LiDAR plugin does
   not need an update for this change.
5. Open Grafana at `http://localhost:3000` on this Windows computer.
6. Import `D:\MASTER\VISTA\LiDar\Repo\dashboard\VISTA_Live_Dashboard_V6_Digital_Twin.json`.
   Select the existing **VISTA Live** data source. Check UID `vistalive` before
   confirming the dashboard update.
7. The new panel appears below the two LiDAR map panels and above Localization.

### WINDOWS — Run the synthetic display demo

In PowerShell, using an existing Python installation, version 3.8 or newer:

```powershell
cd D:\MASTER\VISTA\DigitalTwin\DigitalTwinRepo
python scripts\grafana_demo.py
```

Open the V6 panel. The default bridge URL is `http://127.0.0.1:8101`. Expect a
human-like figure made of ellipsoids and the label **DEMO — not live**. Drag to
rotate and use the mouse wheel to zoom. This is synthetic display test data, not
an AI model result or camera capture. Stop the demo with Ctrl+C; the figure should
disappear after disconnection. Stop this demo before starting the real bridge:
both use port 8101.

## 3. Obtain the real model assets — currently missing

The current repository does not contain the required model weights or bundle.
An NVIDIA GPU does not replace these assets. For reproducibility, obtain the
previously working asset package from your collaborator. Do not substitute stock
YOLO COCO weights or use Jetson TensorRT engines on Windows.

Ask your collaborator for:

1. A verified InstantHMR bundle, for example
   `vista-instanthmr-candidate01-assets.tar.gz`, containing `bundle`, or
   `vista-instanthmr-trt84-02-assets.tar.gz`, containing `bundle-trt84-02`.
   Both use ONNX on Windows. The TRT8 compatibility rewrite is not required on Windows.
2. The PIC fine-tuned YOLO file `best-800x1280-opset12.onnx`, SHA-256:
   `6ab7b30254c59eeaff0115fc9ccfb29262c74fd546ccd29a6ccd3b25ac20d3f8`.
   Keep the accompanying licenses and model provenance.

The full bundle must contain `manifest.json`, `weights/mhr-body.onnx`,
`weights/instanthmr-small.onnx`, `weights/instanthmr-medium.onnx`, all four
`reference/{small,medium}-{rgb,masked}.npz` files, `frame/image.png`,
`frame/mask.png`, and the `LICENSE` and `LICENSE-SAM` files. Downloading only the
three ONNX files is insufficient for the existing gateway and benchmark because
the topology and reference data are also required.

The pinned upstream sources are documented in `docs/INSTANTHMR_CANDIDATE.md`.
`prepare_instanthmr_candidate.py` requires the existing image/mask and official
runner outputs. Do not generate your own reference outputs and describe them as
a passed equivalence benchmark. If the existing bundle cannot be recovered,
asset preparation is a separate task. The live camera/AI steps below cannot run
until these assets are available.

Store assets outside Git, for example:

```text
D:\VISTA-assets\person-twin\bundle\manifest.json
D:\VISTA-assets\person-twin\bundle\weights\...
D:\VISTA-assets\person-twin\yolo\best-800x1280-opset12.onnx
```

If the archive extracts to `bundle-trt84-02`, use that actual path instead of
`bundle` in the commands below. Do not edit the manifest or bypass checksum validation.

## 4. WINDOWS — Create an isolated AI environment

Preserve existing working environments. Install **Python 3.11 x64** if necessary.
The default `python` command might point to Python 3.14, which is unsuitable for
the pinned packages below.

```powershell
cd D:\MASTER\VISTA\DigitalTwin\DigitalTwinRepo
nvidia-smi
py -3.11 -m venv .venv-windows-person
& .\.venv-windows-person\Scripts\python.exe -m pip install -r requirements\windows-person-gpu-py311.in
& .\.venv-windows-person\Scripts\python.exe -c "import onnxruntime as ort; ort.preload_dlls(); print(ort.__version__); print(ort.get_available_providers())"
```

Requirements: an NVIDIA driver compatible with CUDA 12 and the Microsoft Visual
C++ runtime required by ONNX Runtime. The requirements file pins ORT GPU 1.23.2
and installs CUDA/cuDNN pip libraries into this environment. This is an explicit
installation step, not an automatic download during AI execution. Do not install
the CPU `onnxruntime` package into the same environment. If the driver is too old
or DLLs are missing, follow the ORT instructions rather than changing ADLINK JetPack.
Seeing CUDA in the provider list does not prove that a model has run on the GPU.

Validate the assets and run the real model benchmark:

```powershell
$Bundle = 'D:\VISTA-assets\person-twin\bundle'
$Yolo = 'D:\VISTA-assets\person-twin\yolo\best-800x1280-opset12.onnx'
& .\.venv-windows-person\Scripts\python.exe scripts\check_windows_assets.py --bundle $Bundle --yolo $Yolo
& .\.venv-windows-person\Scripts\python.exe scripts\instanthmr_candidate.py --bundle $Bundle --out outputs\windows-v6-cuda-01 --device cuda --runs 10 --warmup 2
```

Expected: `assets_ready: true`, benchmark `passed: true`, equivalence passing for
all four cases, and successful CUDA provider initialization. Use a new output
directory when repeating the benchmark. This benchmark does not measure YOLO,
camera capture or end-to-end latency. The worker loads YOLO when it receives its
first image; check segmentation status separately in step 7.

Official runtime reference:
[ONNX Runtime CUDA Execution Provider](https://onnxruntime.ai/docs/execution-providers/CUDA-ExecutionProvider.html).

## 5. WINDOWS — Prepare a lightweight gateway bundle and transfer source to ADLINK

After obtaining and validating the bundle:

```powershell
cd D:\MASTER\VISTA\DigitalTwin\DigitalTwinRepo
$Bundle = 'D:\VISTA-assets\person-twin\bundle'
& .\.venv-windows-person\Scripts\python.exe scripts\prepare_gateway_bundle.py --bundle $Bundle --out outputs\gateway-bundle-v6-01
tar -czf outputs\gateway-bundle-v6-01.tar.gz -C outputs gateway-bundle-v6-01
```

This package contains topology, references, test images and licenses, but no AI
weights. The source archive was prepared separately at
`outputs\grafana-v6-20261118\vista-person-grafana-v6-src.tar.gz`.
For subsequent source changes, create a new archive using
`scripts/export_source.py --out NEW_ARCHIVE_PATH`.

Set the actual ADLINK IP address; do not assume a historical address is current:

```powershell
$AdlinkHost = 'REPLACE_WITH_ADLINK_IP'
Test-NetConnection $AdlinkHost -Port 22
scp outputs\grafana-v6-20261118\vista-person-grafana-v6-src.tar.gz outputs\gateway-bundle-v6-01.tar.gz "adlink@${AdlinkHost}:~/"
```

The source archive is prepared locally. The gateway asset archive is created by
the commands above after the model bundle becomes available. No automatic
transfer or SSH connection to ADLINK has been performed in this session.
The account `adlink` is an example; replace it if your account differs.

## 6. ADLINK — Start the camera gateway without AI inference

In your ADLINK SSH session, create a new deployment directory and preserve the old one:

```bash
mkdir ~/vista-person-grafana-v6-01
cd ~/vista-person-grafana-v6-01
tar -xzf ~/vista-person-grafana-v6-src.tar.gz
tar -xzf ~/gateway-bundle-v6-01.tar.gz
python3 -c "import cv2, numpy; print(cv2.__version__, numpy.__version__)"
```

Preserve vendor OpenCV/NumPy. Do not install the Windows requirements on ADLINK.
If imports fail, use the interpreter from the working capture setup instead of
upgrading the system. Stop any previous preview or gateway that owns the camera
normally before starting the new gateway. Use the camera configuration already
verified on this device. `configs/neon-person-twin.json` is supplied in the repo;
its presence does not establish compatibility with every camera.

Enter a private token shared by the gateway, worker and bridge. Do not write it
into Git or the dashboard:

```bash
read -rs -p 'VISTA token: ' VISTA_TWIN_TOKEN; echo
export VISTA_TWIN_TOKEN
python3 -B scripts/person_twin.py gateway \
  --bundle gateway-bundle-v6-01 --source camera \
  --camera-config configs/neon-person-twin.json \
  --bind 0.0.0.0 --port 8100 \
  --inference-worker lab --hmr small --segmenter yolo11m-seg --input-mode rgb --start
```

`--start` is essential: the Grafana panel only reads data and does not start or
stop AI inference. The gateway selects `lab`; do not launch a `neon` AI worker.
Camera warmup takes approximately eight seconds. Restrict port 8100 access to
the Windows computer on your trusted LAN. The gateway uses HTTP with a bearer
token and is not intended for public Internet hosting. Do not disable the entire firewall.

## 7. WINDOWS — Start the AI worker

Open a new PowerShell terminal:

```powershell
cd D:\MASTER\VISTA\DigitalTwin\DigitalTwinRepo
$AdlinkHost = 'REPLACE_WITH_ADLINK_IP'
$Bundle = 'D:\VISTA-assets\person-twin\bundle'
$Yolo = 'D:\VISTA-assets\person-twin\yolo\best-800x1280-opset12.onnx'
$env:VISTA_TWIN_TOKEN = [System.Net.NetworkCredential]::new('', (Read-Host 'VISTA token matching ADLINK' -AsSecureString)).Password
Test-NetConnection $AdlinkHost -Port 8100
& .\.venv-windows-person\Scripts\python.exe scripts\person_twin.py worker `
  --bundle $Bundle --gateway "http://${AdlinkHost}:8100" --worker lab --device cuda `
  --yolo $Yolo --yolo-sha256 6ab7b30254c59eeaff0115fc9ccfb29262c74fd546ccd29a6ccd3b25ac20d3f8 `
  --segmenter yolo11m-seg --hmr small
```

Expect gateway connection and body/model loading messages, followed by image
processing by `lab`. Initial model loading may take time. To supply a verified
camera focal length, add `--focal VERIFIED_FOCAL_PIXELS`. Otherwise the worker
uses a fallback and records it as uncalibrated. Do not reuse another camera's
focal length. Keep the entire body visible and only one clear subject in view.
Two sufficiently confident person detections clear the mesh. This pipeline does
not recognize personal identity.

## 8. WINDOWS — Start the real bridge and view V6

Stop `grafana_demo.py` if it is running. In another PowerShell terminal:

```powershell
cd D:\MASTER\VISTA\DigitalTwin\DigitalTwinRepo
$AdlinkHost = 'REPLACE_WITH_ADLINK_IP'
$env:VISTA_TWIN_TOKEN = [System.Net.NetworkCredential]::new('', (Read-Host 'VISTA token matching ADLINK' -AsSecureString)).Password
& .\.venv-windows-person\Scripts\python.exe scripts\grafana_bridge.py --gateway "http://${AdlinkHost}:8100"
```

Open the V6 dashboard in Grafana at `http://localhost:3000`. The panel uses
`http://127.0.0.1:8101`. The token stays in the processes, not in panel options.
The bridge does not expose control POST requests or forward camera images; it
only provides read access to status, topology and mesh.

If the actual Grafana URL differs, for example `http://COMPUTER_NAME:3000`, add
`--grafana-origin http://COMPUTER_NAME:3000` to the bridge command. The origin
must match the scheme, host and port. Without this option, the defaults allow
`http://localhost:3000` and `http://127.0.0.1:3000`. To retain those defaults while
adding another origin, specify all three using repeated `--grafana-origin` options.

The default setup requires the browser to run on the Windows computer hosting
the bridge. HTTPS Grafana or viewing from another computer requires a same-origin
HTTPS reverse proxy with the read routes configured. Do not expose the bridge
on `0.0.0.0` or put the token in the dashboard.

## 9. Validate and troubleshoot

In another Windows PowerShell terminal:

```powershell
$s = Invoke-RestMethod http://127.0.0.1:8101/api/status
$s | Select-Object source, source_error, capture_fps, mesh_fps, mesh_age_seconds, stale
$s.control
$s.workers.lab
```

Expected: `source=camera`, `control.worker=lab`, `running=True`,
`workers.lab.online=True`, no `source_error`, and `mesh_fps > 0` when one person
is visible. The panel shows LIVE, fresh mesh rate and frame age. Render FPS is
different from AI FPS; 30 FPS is not guaranteed.

1. Leave the camera view: the mesh should disappear when the worker reports
   absence or the result expires.
2. Stop the worker: the mesh should disappear after the gateway expiry period,
   two seconds by default, plus network delay.
3. Stop the gateway or disconnect the network: the panel should report an error
   and clear the mesh. The two map panels retain their existing behavior.
4. Restart the gateway and worker: the panel should reload session/topology and recover.
5. Restart the old camera preview only after stopping the gateway, keeping one camera owner.

Common problems:

| Symptom | Check |
|---|---|
| Panel plugin not found | Installation folder, plugin.json location, unsigned list, Grafana restart |
| Failed to fetch / CORS | Bridge running, exact browser origin, HTTP versus HTTPS |
| Bridge HTTP 502 | ADLINK gateway, IP/port 8100, shared token; inspect `/api/status` using PowerShell |
| Waiting for Windows AI worker | Worker running, `--worker lab`, gateway `--inference-worker lab --start` |
| CUDA DLL/provider error | Python 3.11 x64, NVIDIA driver, ORT CUDA/cuDNN and VC++ runtime |
| Checksum / trained YOLO rejected | Complete verified bundle and registered PIC ONNX export |
| Result rejected/stale | Inference/network latency; reduce load rather than hiding the problem by extending the age limit |
| Incorrect body proportions/pose | Full-body visibility, occlusion and focal length; the mesh is inferred and accuracy has not been measured |

Stop services in this order: Windows worker -> Windows bridge -> ADLINK gateway.
Grafana and LiDAR do not need to stop when AI processing ends. To restore the
previous dashboard, import V5 or your earlier dashboard export. The LiDAR plugin
and embedded_lidar code are unchanged by this integration.

For real deployment review, return the CUDA benchmark report, `/api/status`
output with secrets removed, a panel screenshot and a short full-body movement
clip. TensorRT engines do not need to be sent.

Grafana configuration reference:
[Allow loading unsigned plugins](https://grafana.com/docs/grafana/latest/setup-grafana/configure-grafana/#allow_loading_unsigned_plugins).
