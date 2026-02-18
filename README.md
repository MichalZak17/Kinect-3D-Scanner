<p align="center">
  <h1 align="center">Kinect 3D Scanner</h1>
  <p align="center">
    A client/server application for 3D scanning with the Xbox 360 Kinect (v1).<br/>
    Capture frames on a laptop, process on a server, export to Blender-compatible PLY/OBJ.
  </p>
</p>

<p align="center">
  <a href="#installation"><img src="https://img.shields.io/badge/platform-Linux-blue?logo=linux&logoColor=white" alt="Platform"></a>
  <a href="#installation"><img src="https://img.shields.io/badge/python-3.10+-yellow?logo=python&logoColor=white" alt="Python"></a>
  <a href="#usage"><img src="https://img.shields.io/badge/UI-PyQt6-green?logo=qt&logoColor=white" alt="PyQt6"></a>
  <a href="#architecture"><img src="https://img.shields.io/badge/server-FastAPI-009688?logo=fastapi&logoColor=white" alt="FastAPI"></a>
  <a href="docs/KINECT_V1_LINUX_PYTHON_REFERENCE.md"><img src="https://img.shields.io/badge/docs-reference-orange?logo=readthedocs&logoColor=white" alt="Docs"></a>
</p>

---

## Architecture

```
 LAPTOP (Client)                        SERVER
 ┌──────────────────────┐              ┌──────────────────────────┐
 │  Kinect v1 sensor     │              │  FastAPI + ScanEngine    │
 │  ─────────────────    │   HTTP/WS    │  ──────────────────────  │
 │  KinectWorker         │◄────────────►│  ICP registration (CPU)  │
 │  PyQt6 GUI            │  port 8000   │  TSDF integration (CPU)  │
 │  Live RGB + depth     │              │  Mesh extraction         │
 │  Frame capture        │              │  PLY/OBJ export          │
 └──────────────────────┘              └──────────────────────────┘
        shared/                                shared/
     (config, protocol)                    (config, protocol)
```

The **client** captures Kinect frames and displays live video. Frames are compressed (zlib) and sent to the **server** over HTTP in batches. The server runs ICP registration + TSDF volumetric integration using Open3D (VoxelBlockGrid on CPU), then serves the reconstructed mesh back to the client for preview and export.

| Component | Runs on | Key dependencies |
|-----------|---------|-----------------|
| **Client** (`kinect_scanner`) | Laptop with Kinect | PyQt6, freenect, httpx, websocket-client |
| **Server** (`scanner_server`) | Any machine with enough RAM | FastAPI, Open3D, uvicorn |
| **Shared** (`shared`) | Both | Pure Python (numpy only) |

---

## Scanning Workflow

```
Connect to Server ──> Start Scan ──> Capture Frames ──> Preview ──> Build Mesh ──> Export
                                      (batched to server)  (optional)  (on server)   PLY/OBJ
```

1. Enter the server IP and click **Connect**
2. Click **Start Scan** to begin a new session
3. Move the Kinect around the object, clicking **Capture Frame** or enabling auto-capture — frames are compressed, batched, and uploaded to the server
4. Optionally click **Preview Scan** to process frames and view the current mesh
5. Click **Stop & Build Mesh** to process all remaining frames and extract the final mesh on the server
6. **Export** as PLY or OBJ — the file is downloaded from the server and saved locally

---

## Server Setup

### Requirements
- Python 3.10+
- Open3D 0.19+

### Install

```bash
pip install -r requirements-server.txt
```

### Run

```bash
python -m scanner_server
```

The server listens on `0.0.0.0:8000` by default.

### API Endpoints

| Method | Endpoint | Purpose |
|--------|----------|---------|
| `GET` | `/api/health` | Connection check + status |
| `GET` | `/api/scan/status` | Stored/integrated counts, has_mesh |
| `POST` | `/api/scan/reset` | Reset engine, start new scan |
| `POST` | `/api/scan/frame` | Upload a single compressed frame (binary body) |
| `POST` | `/api/scan/frames` | Upload a batch of compressed frames (binary body) |
| `POST` | `/api/scan/build` | Process all frames + build mesh |
| `POST` | `/api/scan/preview` | Process + extract preview, return PLY |
| `GET` | `/api/scan/export/ply` | Download mesh as PLY |
| `GET` | `/api/scan/export/obj` | Download mesh as OBJ |
| `WebSocket` | `/ws/progress` | Real-time progress during build/preview |

---

## Client Setup (Laptop with Kinect)

### Requirements
- Xbox 360 Kinect (model 1414 / Kinect v1) with USB + power adapter
- libfreenect driver installed
- Same LAN as the server

### Install

```bash
pip install -r requirements-client.txt
```

#### Install libfreenect (Kinect driver)

```bash
git clone https://github.com/OpenKinect/libfreenect
cd libfreenect && mkdir build && cd build
cmake .. -DBUILD_PYTHON3=ON -DCMAKE_INSTALL_PREFIX=/usr
make && sudo make install
cd ../wrappers/python && pip install .
```

> See [docs/KINECT_V1_LINUX_PYTHON_REFERENCE.md](docs/KINECT_V1_LINUX_PYTHON_REFERENCE.md) for detailed installation, udev rules, and troubleshooting.

### Run

```bash
python -m kinect_scanner
```

### View Modes

| Mode | Description |
|------|-------------|
| **RGB** | Live color camera feed |
| **Depth** | Colorized depth map with adjustable near/far clipping |
| **Scanner** | Side-by-side RGB + depth for scanning |

---

## Network Protocol

Frames are serialized using the `shared.protocol` module.

### Single Frame

| Field | Size | Description |
|-------|------|-------------|
| `rgb_len` | 4 bytes (big-endian uint32) | Length of compressed RGB data |
| `rgb_compressed` | variable | zlib level=1 compressed RGB (480x640x3 uint8) |
| `depth_compressed` | remainder | zlib level=1 compressed depth (480x640 uint16) |

Typical compressed frame size: ~150-300 KB (vs ~1.5 MB uncompressed).

### Batch (multiple frames)

| Field | Size | Description |
|-------|------|-------------|
| `magic` | 4 bytes | `0x42415448` ("BATH") — identifies a batch payload |
| `frame_count` | 4 bytes (big-endian uint32) | Number of frames N |
| `lengths` | 4*N bytes | Per-frame packed byte lengths |
| `frames` | variable | Concatenated single-frame payloads |

The client automatically batches all queued frames into a single HTTP request (capped at 100 frames per request for bounded memory). Falls back to individual sends if the server lacks the batch endpoint.

---

## Scan Parameters

| Parameter | Value |
|-----------|-------|
| Voxel size | 5 mm |
| SDF truncation | 40 mm |
| Max depth | 4.0 m |
| Depth range | 500 - 4000 mm |

---

## Export Formats

| Format | Contents | Use case |
|--------|----------|----------|
| **PLY** | Point cloud or triangle mesh with vertex colors | MeshLab, CloudCompare, Blender |
| **OBJ** | Triangle mesh with vertex colors | Blender, 3D printing pipelines |

---

## Project Structure

```
Kinect-3D-Scanner/
├── shared/                        # Shared pure-Python utilities
│   ├── config.py                  # Camera intrinsics, scan presets
│   └── protocol.py                # Frame pack/unpack (single + batch)
│
├── kinect_scanner/                # CLIENT (runs on laptop)
│   ├── __main__.py                # Entry point: python -m kinect_scanner
│   ├── config.py                  # Re-exports shared.config + O3D_INTRINSIC
│   ├── worker.py                  # Background Kinect frame capture thread
│   ├── server_client.py           # HTTP + WebSocket client
│   ├── server_task_worker.py      # Background task queue for server calls
│   ├── viewer.py                  # 3D mesh/point cloud visualization
│   ├── engine.py                  # Local scan engine (kept for reference)
│   ├── task_manager.py            # Local task manager (kept for reference)
│   └── gui/
│       ├── main_window.py         # PyQt6 application window
│       └── widgets.py             # Depth colorization, image conversion
│
├── scanner_server/                # SERVER (runs on processing machine)
│   ├── __main__.py                # Entry point: python -m scanner_server
│   ├── app.py                     # FastAPI app with all endpoints
│   ├── config.py                  # Server-side O3D_INTRINSIC
│   └── engine.py                  # Scan pipeline (ICP, TSDF, mesh)
│
├── docs/
│   └── KINECT_V1_LINUX_PYTHON_REFERENCE.md
├── export/                        # PLY/OBJ exports (client-side)
├── mesh/                          # Auto-saved scan meshes (client-side)
├── requirements.txt               # All dependencies (client + server)
├── requirements-client.txt        # Client-only dependencies
├── requirements-server.txt        # Server-only dependencies
└── README.md
```

---

## Documentation

- **[Kinect v1 Technical Reference](docs/KINECT_V1_LINUX_PYTHON_REFERENCE.md)** — Hardware specs, driver installation, Python API, camera intrinsics, calibration, point cloud generation, registration algorithms, mesh reconstruction, and export formats.

---

## License

This project is provided as-is for educational and personal use.
