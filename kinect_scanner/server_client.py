"""HTTP + WebSocket client for communicating with the scanner server."""

import json
import logging
import tempfile
import threading
import time

import httpx
from PyQt6.QtCore import QObject, pyqtSignal

from shared.protocol import pack_frame, pack_frames

logger = logging.getLogger(__name__)


class ServerClient(QObject):
    """Network client that talks to the scanner_server REST + WebSocket API.

    Signals mirror the old ScanTaskManager interface for easy GUI wiring.
    """

    connected = pyqtSignal()
    disconnected = pyqtSignal(str)

    frame_stored = pyqtSignal(dict)
    process_progress = pyqtSignal(int, int, dict)
    build_mesh_done = pyqtSignal(bool, str)
    preview_done = pyqtSignal(str)         # PLY temp file path
    export_done = pyqtSignal(bool, str)
    save_mesh_done = pyqtSignal(bool, str)
    status_updated = pyqtSignal(dict)
    task_started = pyqtSignal(str)
    task_error = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._base_url: str | None = None
        self._ws_url: str | None = None
        self._http: httpx.Client | None = None
        self._ws_thread: threading.Thread | None = None
        self._ws_stop = threading.Event()
        self._connected = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    # ── Connection ─────────────────────────────────────────────────────

    def connect_to_server(self, host: str, port: int) -> bool:
        """Try to connect. Returns True on success. Starts WebSocket listener."""
        self._base_url = f"http://{host}:{port}"
        self._ws_url = f"ws://{host}:{port}/ws/progress"

        try:
            self._http = httpx.Client(base_url=self._base_url, timeout=30.0)
            resp = self._http.get("/api/health")
            resp.raise_for_status()
            data = resp.json()
            if data.get("status") != "ok":
                raise RuntimeError(f"Server not ok: {data}")
        except Exception as e:
            self._connected = False
            self.disconnected.emit(str(e))
            return False

        self._connected = True
        self._ws_stop.clear()
        self._ws_thread = threading.Thread(
            target=self._ws_listener, daemon=True, name="ws-listener"
        )
        self._ws_thread.start()
        self.connected.emit()
        return True

    def disconnect(self):
        """Close HTTP client and stop WebSocket thread."""
        self._ws_stop.set()
        if self._http:
            self._http.close()
            self._http = None
        self._connected = False
        if self._ws_thread and self._ws_thread.is_alive():
            self._ws_thread.join(timeout=2.0)
        self._ws_thread = None

    # ── WebSocket listener ─────────────────────────────────────────────

    def _ws_listener(self):
        """Background thread: listen for progress messages over WebSocket."""
        import websocket as ws_lib

        while not self._ws_stop.is_set():
            try:
                sock = ws_lib.WebSocket()
                sock.settimeout(5.0)
                sock.connect(self._ws_url)
                logger.info("WebSocket connected to %s", self._ws_url)

                while not self._ws_stop.is_set():
                    try:
                        raw = sock.recv()
                    except ws_lib.WebSocketTimeoutException:
                        continue
                    except ws_lib.WebSocketConnectionClosedException:
                        break

                    if not raw:
                        continue

                    try:
                        msg = json.loads(raw)
                    except json.JSONDecodeError:
                        continue

                    self._handle_ws_message(msg)

                sock.close()
            except Exception as e:
                if not self._ws_stop.is_set():
                    logger.warning("WebSocket error: %s, reconnecting...", e)
                    time.sleep(1.0)

    def _handle_ws_message(self, msg: dict):
        """Route a parsed WebSocket message to the appropriate Qt signal."""
        msg_type = msg.get("type")

        if msg_type == "progress":
            self.process_progress.emit(
                msg.get("current", 0),
                msg.get("total", 0),
                {"message": msg.get("message", "")},
            )
        elif msg_type == "done":
            success = msg.get("success", False)
            detail = msg.get("detail", "")
            self.build_mesh_done.emit(success, detail)
        elif msg_type == "error":
            self.task_error.emit(msg.get("message", "Unknown server error"))

    # ── API methods (called from ServerTaskWorker thread) ──────────────

    def send_frame(self, rgb, depth) -> dict:
        """Pack and upload a frame. Returns the server response dict."""
        data = pack_frame(rgb, depth)
        resp = self._http.post(
            "/api/scan/frame",
            content=data,
            headers={"Content-Type": "application/octet-stream"},
        )
        resp.raise_for_status()
        return resp.json()

    def send_frames_batch(self, frames: list[tuple]) -> dict:
        """Pack and upload multiple frames as a single batch."""
        data = pack_frames(frames)
        resp = self._http.post(
            "/api/scan/frames",
            content=data,
            headers={"Content-Type": "application/octet-stream"},
            timeout=120.0,
        )
        if resp.status_code == 404:
            # Server doesn't support batch — fall back to individual sends
            logger.warning("Server lacks batch endpoint, sending individually")
            result = None
            for rgb, depth in frames:
                result = self.send_frame(rgb, depth)
            return result
        resp.raise_for_status()
        return resp.json()

    def reset_scan(self) -> dict:
        resp = self._http.post("/api/scan/reset")
        resp.raise_for_status()
        return resp.json()

    def request_build(self) -> dict:
        """Start a build. Progress comes via WebSocket."""
        resp = self._http.post("/api/scan/build", timeout=600.0)
        resp.raise_for_status()
        return resp.json()

    def request_preview(self) -> str | None:
        """Request preview, save returned PLY to a temp file, return path."""
        resp = self._http.post("/api/scan/preview", timeout=600.0)
        resp.raise_for_status()

        content_type = resp.headers.get("content-type", "")
        if "octet-stream" not in content_type:
            # Got JSON error response
            return None

        fd, tmp_path = tempfile.mkstemp(suffix=".ply")
        import os
        os.close(fd)
        with open(tmp_path, "wb") as f:
            f.write(resp.content)
        return tmp_path

    def request_export(self, fmt: str, save_path: str) -> bool:
        """Download exported mesh and save to local path."""
        resp = self._http.get(f"/api/scan/export/{fmt}", timeout=120.0)
        resp.raise_for_status()

        content_type = resp.headers.get("content-type", "")
        if "octet-stream" not in content_type:
            return False

        with open(save_path, "wb") as f:
            f.write(resp.content)
        return True

    def get_status(self) -> dict:
        resp = self._http.get("/api/scan/status")
        resp.raise_for_status()
        return resp.json()
