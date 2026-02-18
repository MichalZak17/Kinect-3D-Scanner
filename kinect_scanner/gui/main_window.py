"""MainWindow — primary application window (client/server mode)."""

import os
import time
from datetime import datetime

import numpy as np
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QPixmap, QAction
from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QLabel, QVBoxLayout, QHBoxLayout,
    QPushButton, QGroupBox, QProgressBar, QToolBar, QFileDialog,
    QDockWidget, QMessageBox, QSpinBox, QDoubleSpinBox, QCheckBox,
    QLineEdit, QApplication,
)

from ..config import MODE_RGB, MODE_DEPTH, MODE_SCANNER, PRESET_DEFAULT
from ..worker import KinectWorker
from ..server_client import ServerClient
from ..server_task_worker import ServerTaskWorker, ServerTask, ServerTaskType
from ..viewer import launch_viewer_subprocess
from .widgets import colorize_depth, numpy_to_qimage

# Default export directory (relative to where the app is launched)
_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)
)))
EXPORT_DIR = os.path.join(_PROJECT_ROOT, "export")
MESH_DIR = os.path.join(_PROJECT_ROOT, "mesh")


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Kinect 3D Scanner")
        self.setMinimumSize(960, 600)

        self._mode = MODE_RGB
        self._scanning = False
        self._fps_counter = 0
        self._fps_value = 0.0
        self._last_fps_time = time.time()
        self._last_rgb = None
        self._last_depth = None
        self._last_preview_path: str | None = None

        # Server counts (cached from server responses)
        self._server_stored = 0
        self._server_integrated = 0

        # Server client + task worker
        self.server_client = ServerClient(self)
        self.server_client.frame_stored.connect(self._on_frame_stored)
        self.server_client.process_progress.connect(self._on_process_progress)
        self.server_client.build_mesh_done.connect(self._on_build_mesh_done)
        self.server_client.preview_done.connect(self._on_preview_done)
        self.server_client.export_done.connect(self._on_export_done)
        self.server_client.save_mesh_done.connect(self._on_save_mesh_done)
        self.server_client.task_started.connect(self._on_task_started)
        self.server_client.task_error.connect(self._on_task_error)
        self.server_client.connected.connect(self._on_server_connected)
        self.server_client.disconnected.connect(self._on_server_disconnected)

        self.task_worker = ServerTaskWorker(self.server_client)
        self.task_worker.start()

        self._build_ui()
        self._build_toolbar()
        self._build_dock()
        self._build_statusbar()

        self._fps_timer = QTimer(self)
        self._fps_timer.timeout.connect(self._update_fps)
        self._fps_timer.start(1000)

        self.worker = KinectWorker()
        self.worker.frame_ready.connect(self._on_frame)
        self.worker.error_occurred.connect(self._on_error)
        self.worker.start()

        # Disable scan controls until server connected
        self._set_scan_controls_enabled(False)

    # ── UI construction ───────────────────────────────────────────────
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        layout = QHBoxLayout(central)
        layout.setContentsMargins(4, 4, 4, 4)

        self.view_label = QLabel("Connecting to Kinect...")
        self.view_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.view_label.setMinimumSize(640, 480)
        self.view_label.setStyleSheet(
            "background-color: #1e1e1e; color: #aaa; font-size: 18px;"
        )
        layout.addWidget(self.view_label, stretch=1)

    def _build_toolbar(self):
        toolbar = QToolBar("Modes")
        toolbar.setMovable(False)
        self.addToolBar(toolbar)
        for mode in (MODE_RGB, MODE_DEPTH, MODE_SCANNER):
            action = QAction(mode, self)
            action.setCheckable(True)
            if mode == MODE_RGB:
                action.setChecked(True)
            action.triggered.connect(lambda checked, m=mode: self._switch_mode(m))
            toolbar.addAction(action)
        self._mode_actions = toolbar.actions()

    def _build_dock(self):
        dock = QDockWidget("Controls", self)
        dock.setFeatures(QDockWidget.DockWidgetFeature.NoDockWidgetFeatures)
        dock.setFixedWidth(260)
        container = QWidget()
        layout = QVBoxLayout(container)

        # ── Server Connection ─────────────────────────────────────
        server_group = QGroupBox("Server Connection")
        sg_layout = QVBoxLayout(server_group)

        ip_row = QHBoxLayout()
        self.server_ip_edit = QLineEdit()
        self.server_ip_edit.setPlaceholderText("Server IP, e.g. 10.0.0.107")
        ip_row.addWidget(self.server_ip_edit, stretch=1)
        self.server_port_spin = QSpinBox()
        self.server_port_spin.setRange(1, 65535)
        self.server_port_spin.setValue(8000)
        self.server_port_spin.setFixedWidth(70)
        ip_row.addWidget(self.server_port_spin)
        sg_layout.addLayout(ip_row)

        self.btn_connect = QPushButton("Connect")
        self.btn_connect.clicked.connect(self._toggle_connection)
        sg_layout.addWidget(self.btn_connect)

        self.server_status_label = QLabel("Not connected")
        self.server_status_label.setStyleSheet("color: #888;")
        self.server_status_label.setWordWrap(True)
        sg_layout.addWidget(self.server_status_label)

        layout.addWidget(server_group)

        # ── Depth visualisation range ─────────────────────────────
        viz_group = QGroupBox("Depth Visualisation")
        vg = QVBoxLayout(viz_group)

        vg.addWidget(QLabel("Near clip (mm):"))
        self.depth_near_spin = QSpinBox()
        self.depth_near_spin.setRange(0, 4000)
        self.depth_near_spin.setValue(500)
        self.depth_near_spin.setSingleStep(100)
        vg.addWidget(self.depth_near_spin)

        vg.addWidget(QLabel("Far clip (mm):"))
        self.depth_far_spin = QSpinBox()
        self.depth_far_spin.setRange(500, 8000)
        self.depth_far_spin.setValue(4000)
        self.depth_far_spin.setSingleStep(100)
        vg.addWidget(self.depth_far_spin)

        layout.addWidget(viz_group)

        # ── Scanner Controls ──────────────────────────────────────
        scan_group = QGroupBox("3D Scanner")
        sg = QVBoxLayout(scan_group)

        self.btn_start_scan = QPushButton("Start Scan")
        self.btn_start_scan.clicked.connect(self._start_scan)
        sg.addWidget(self.btn_start_scan)

        self.btn_capture = QPushButton("Capture Frame")
        self.btn_capture.setEnabled(False)
        self.btn_capture.clicked.connect(self._capture_frame)
        sg.addWidget(self.btn_capture)

        # Auto-capture
        auto_row = QHBoxLayout()
        self.auto_capture_cb = QCheckBox("Auto every")
        self.auto_capture_spin = QDoubleSpinBox()
        self.auto_capture_spin.setRange(0.03, 30.0)
        self.auto_capture_spin.setValue(0.5)
        self.auto_capture_spin.setSingleStep(0.01)
        self.auto_capture_spin.setDecimals(2)
        self.auto_capture_spin.setSuffix("s")
        self.auto_capture_cb.setEnabled(False)
        self.auto_capture_spin.setEnabled(False)
        self.auto_capture_cb.toggled.connect(self._toggle_auto_capture)
        auto_row.addWidget(self.auto_capture_cb)
        auto_row.addWidget(self.auto_capture_spin)
        sg.addLayout(auto_row)

        self.btn_preview_scan = QPushButton("Preview Scan")
        self.btn_preview_scan.setEnabled(False)
        self.btn_preview_scan.clicked.connect(self._preview_scan)
        sg.addWidget(self.btn_preview_scan)

        self.btn_stop_build = QPushButton("Stop && Build Mesh")
        self.btn_stop_build.setEnabled(False)
        self.btn_stop_build.clicked.connect(self._stop_and_build)
        sg.addWidget(self.btn_stop_build)

        export_row = QHBoxLayout()
        self.btn_export_ply = QPushButton("Export PLY")
        self.btn_export_ply.setEnabled(False)
        self.btn_export_ply.clicked.connect(self._export_ply)
        export_row.addWidget(self.btn_export_ply)
        self.btn_export_obj = QPushButton("Export OBJ")
        self.btn_export_obj.setEnabled(False)
        self.btn_export_obj.clicked.connect(self._export_obj)
        export_row.addWidget(self.btn_export_obj)
        sg.addLayout(export_row)

        # Preview 3D button (current scan)
        self.btn_preview_3d = QPushButton("Preview 3D")
        self.btn_preview_3d.setEnabled(False)
        self.btn_preview_3d.clicked.connect(self._preview_3d)
        sg.addWidget(self.btn_preview_3d)

        # Save / Load mesh
        mesh_row = QHBoxLayout()
        self.btn_save_mesh = QPushButton("Save Mesh")
        self.btn_save_mesh.setEnabled(False)
        self.btn_save_mesh.clicked.connect(self._save_mesh)
        mesh_row.addWidget(self.btn_save_mesh)
        self.btn_load_mesh = QPushButton("Load Mesh")
        self.btn_load_mesh.clicked.connect(self._load_mesh)
        mesh_row.addWidget(self.btn_load_mesh)
        sg.addLayout(mesh_row)

        # View 3D file from disk
        self.btn_view_file = QPushButton("View 3D File")
        self.btn_view_file.clicked.connect(self._view_3d_file)
        sg.addWidget(self.btn_view_file)

        self.frame_count_label = QLabel("Stored: 0 | Integrated: 0")
        sg.addWidget(self.frame_count_label)

        self.scan_status_label = QLabel("Idle")
        self.scan_status_label.setWordWrap(True)
        self.scan_status_label.setMaximumWidth(240)
        sg.addWidget(self.scan_status_label)

        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 0)
        self.progress_bar.setVisible(False)
        sg.addWidget(self.progress_bar)

        layout.addWidget(scan_group)
        layout.addStretch()
        dock.setWidget(container)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, dock)

        self._auto_timer = QTimer(self)
        self._auto_timer.timeout.connect(self._capture_frame)

    def _build_statusbar(self):
        self.statusBar().showMessage("Ready")
        self.fps_label = QLabel("FPS: --")
        self.kinect_label = QLabel("Kinect: connecting...")
        self.statusBar().addPermanentWidget(self.fps_label)
        self.statusBar().addPermanentWidget(self.kinect_label)

    def _set_scan_controls_enabled(self, enabled: bool):
        """Enable/disable scan controls based on server connection state."""
        self.btn_start_scan.setEnabled(enabled)

    # ── Server connection ─────────────────────────────────────────────
    def _toggle_connection(self):
        if self.server_client.is_connected:
            self.server_client.disconnect()
            self.btn_connect.setText("Connect")
            self.server_status_label.setText("Disconnected")
            self.server_status_label.setStyleSheet("color: #888;")
            self.server_ip_edit.setEnabled(True)
            self.server_port_spin.setEnabled(True)
            self._set_scan_controls_enabled(False)
            return

        host = self.server_ip_edit.text().strip()
        port = self.server_port_spin.value()
        if not host:
            self.server_status_label.setText("Enter a server IP")
            self.server_status_label.setStyleSheet("color: #c00;")
            return

        self.server_status_label.setText("Connecting...")
        self.server_status_label.setStyleSheet("color: #888;")
        self.btn_connect.setEnabled(False)
        QApplication.processEvents()

        ok = self.server_client.connect_to_server(host, port)
        self.btn_connect.setEnabled(True)

        if ok:
            self.btn_connect.setText("Disconnect")
            self.server_ip_edit.setEnabled(False)
            self.server_port_spin.setEnabled(False)
        # Signal handlers below update the rest

    def _on_server_connected(self):
        self.server_status_label.setText("Connected")
        self.server_status_label.setStyleSheet("color: #0a0; font-weight: bold;")
        self._set_scan_controls_enabled(True)
        self.statusBar().showMessage("Connected to server")

    def _on_server_disconnected(self, reason: str):
        self.server_status_label.setText(f"Error: {reason}")
        self.server_status_label.setStyleSheet("color: #c00;")
        self._set_scan_controls_enabled(False)
        self.statusBar().showMessage(f"Server connection failed: {reason}")

    # ── mode switching ────────────────────────────────────────────────
    def _switch_mode(self, mode: str):
        self._mode = mode
        for action in self._mode_actions:
            action.setChecked(action.text() == mode)

    # ── frame display ─────────────────────────────────────────────────
    def _on_frame(self, video: np.ndarray, depth: np.ndarray):
        self._fps_counter += 1
        self.kinect_label.setText("Kinect: connected")
        self._last_rgb = video
        self._last_depth = depth

        if self._mode == MODE_RGB:
            self._show_rgb(video)
        elif self._mode == MODE_DEPTH:
            self._show_depth(depth)
        elif self._mode == MODE_SCANNER:
            self._show_scanner(video, depth)

    def _show_rgb(self, rgb):
        self._set_pixmap(numpy_to_qimage(rgb))

    def _show_depth(self, depth):
        near = self.depth_near_spin.value()
        far = self.depth_far_spin.value()
        self._set_pixmap(numpy_to_qimage(colorize_depth(depth, near, far)))

    def _show_scanner(self, rgb, depth):
        near = self.depth_near_spin.value()
        far = self.depth_far_spin.value()
        combined = np.hstack([rgb, colorize_depth(depth, near, far)])
        self._set_pixmap(numpy_to_qimage(combined))

    def _set_pixmap(self, qimg):
        self.view_label.setPixmap(
            QPixmap.fromImage(qimg).scaled(
                self.view_label.size(),
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation,
            )
        )

    # ── scanning workflow ─────────────────────────────────────────────
    def _start_scan(self):
        if not self.server_client.is_connected:
            QMessageBox.warning(self, "Not Connected",
                                "Connect to the server first.")
            return

        self.task_worker.submit(ServerTask(ServerTaskType.RESET))
        self._server_stored = 0
        self._server_integrated = 0

        self._scanning = True
        self._switch_mode(MODE_SCANNER)
        self.btn_start_scan.setEnabled(False)
        self.btn_capture.setEnabled(True)
        self.btn_stop_build.setEnabled(True)
        self.btn_export_ply.setEnabled(False)
        self.btn_export_obj.setEnabled(False)
        self.btn_preview_3d.setEnabled(False)
        self.btn_save_mesh.setEnabled(False)
        self.btn_preview_scan.setEnabled(True)
        self.auto_capture_cb.setEnabled(True)
        self.auto_capture_spin.setEnabled(True)
        self.frame_count_label.setText("Stored: 0 | Integrated: 0")
        self.scan_status_label.setText("Scanning — capture frames")
        self.depth_near_spin.setValue(PRESET_DEFAULT.depth_near_mm)
        self.depth_far_spin.setValue(PRESET_DEFAULT.depth_far_mm)
        self.statusBar().showMessage(
            "Scan started. Move Kinect and press Capture Frame."
        )

    def _capture_frame(self):
        if not self._scanning:
            return
        if self._last_rgb is None or self._last_depth is None:
            self.scan_status_label.setText("No frame available yet")
            return

        self.task_worker.submit(ServerTask(
            ServerTaskType.SEND_FRAME,
            {"rgb": self._last_rgb.copy(), "depth": self._last_depth.copy()},
        ))

    def _toggle_auto_capture(self, checked: bool):
        if checked and self._scanning:
            self._auto_timer.start(int(self.auto_capture_spin.value() * 1000))
        else:
            self._auto_timer.stop()

    def _stop_and_build(self):
        self._scanning = False
        self._auto_timer.stop()
        self.auto_capture_cb.setChecked(False)
        self.auto_capture_cb.setEnabled(False)
        self.auto_capture_spin.setEnabled(False)
        self.btn_capture.setEnabled(False)
        self.btn_stop_build.setEnabled(False)
        self.btn_preview_scan.setEnabled(False)

        stored = self._server_stored
        self.scan_status_label.setText(
            f"Processing {stored} frames on server..."
        )
        self.progress_bar.setRange(0, 0)  # indeterminate until progress arrives
        self.progress_bar.setVisible(True)

        self.task_worker.submit(ServerTask(ServerTaskType.BUILD_MESH))

    def _preview_scan(self):
        if self._server_stored == 0:
            QMessageBox.information(
                self, "Preview", "Capture at least one frame first."
            )
            return
        self.btn_preview_scan.setEnabled(False)
        self.scan_status_label.setText("Generating preview on server...")
        self.progress_bar.setRange(0, 0)
        self.progress_bar.setVisible(True)

        self.task_worker.submit(ServerTask(ServerTaskType.PREVIEW))

    def _preview_3d(self):
        if self._last_preview_path:
            launch_viewer_subprocess(self._last_preview_path)

    def _ensure_export_dir(self):
        os.makedirs(EXPORT_DIR, exist_ok=True)
        return EXPORT_DIR

    def _export_ply(self):
        default_path = os.path.join(self._ensure_export_dir(), "scan.ply")
        path, _ = QFileDialog.getSaveFileName(
            self, "Export PLY", default_path, "PLY files (*.ply)"
        )
        if path:
            self.btn_export_ply.setEnabled(False)
            self.task_worker.submit(
                ServerTask(ServerTaskType.EXPORT_PLY, {"path": path})
            )

    def _export_obj(self):
        default_path = os.path.join(self._ensure_export_dir(), "scan.obj")
        path, _ = QFileDialog.getSaveFileName(
            self, "Export OBJ", default_path, "OBJ files (*.obj)"
        )
        if path:
            self.btn_export_obj.setEnabled(False)
            self.task_worker.submit(
                ServerTask(ServerTaskType.EXPORT_OBJ, {"path": path})
            )

    def _save_mesh(self):
        os.makedirs(MESH_DIR, exist_ok=True)
        filename = f"scan_{datetime.now().strftime('%Y%m%d_%H%M%S')}.ply"
        path = os.path.join(MESH_DIR, filename)
        self.btn_save_mesh.setEnabled(False)
        self.task_worker.submit(
            ServerTask(ServerTaskType.SAVE_MESH, {"path": path})
        )

    def _load_mesh(self):
        start_dir = MESH_DIR if os.path.isdir(MESH_DIR) else ""
        path, _ = QFileDialog.getOpenFileName(
            self, "Load Mesh", start_dir,
            "PLY files (*.ply);;All 3D files (*.obj *.ply *.stl)"
        )
        if path:
            self.statusBar().showMessage(f"Opening: {path}")
            launch_viewer_subprocess(path)

    def _view_3d_file(self):
        start_dir = EXPORT_DIR if os.path.isdir(EXPORT_DIR) else ""
        path, _ = QFileDialog.getOpenFileName(
            self, "Open 3D File", start_dir,
            "3D files (*.obj *.ply *.stl);;OBJ files (*.obj);;PLY files (*.ply);;STL files (*.stl)"
        )
        if path:
            self.statusBar().showMessage(f"Opening: {path}")
            launch_viewer_subprocess(path)

    # ── Task/server signal handlers ──────────────────────────────────
    def _on_frame_stored(self, result: dict):
        self._server_stored = result.get("stored_count", self._server_stored + 1)
        self.frame_count_label.setText(
            f"Stored: {self._server_stored} | Integrated: {self._server_integrated}"
        )
        self.scan_status_label.setText(result.get("message", "Frame stored"))
        self.statusBar().showMessage(result.get("message", "Frame stored"))

    def _on_process_progress(self, current: int, total: int, result: dict):
        self.progress_bar.setRange(0, total)
        self.progress_bar.setValue(current)
        self._server_integrated = current  # approximate
        self.frame_count_label.setText(
            f"Stored: {total} | Integrated: {current}"
        )
        self.scan_status_label.setText(
            f"Processing {current}/{total}: {result.get('message', '')}"
        )

    def _on_build_mesh_done(self, success: bool, detail: str):
        self.progress_bar.setVisible(False)
        self.scan_status_label.setText(detail)

        if success:
            self.btn_export_ply.setEnabled(True)
            self.btn_export_obj.setEnabled(True)
            self.btn_save_mesh.setEnabled(True)
            self.statusBar().showMessage("Mesh built. Ready to export or preview.")
        else:
            self.statusBar().showMessage("Mesh build failed.")

        self.btn_start_scan.setEnabled(True)

    def _on_preview_done(self, path: str):
        self.progress_bar.setVisible(False)
        if self._scanning:
            self.btn_preview_scan.setEnabled(True)

        if path:
            self._last_preview_path = path
            self.btn_preview_3d.setEnabled(True)
            self.scan_status_label.setText(
                f"Scanning — {self._server_stored} frames stored"
            )
            launch_viewer_subprocess(path)
        else:
            self.scan_status_label.setText("Preview extraction failed")

    def _on_export_done(self, success: bool, path: str):
        self.btn_export_ply.setEnabled(True)
        self.btn_export_obj.setEnabled(True)
        if success:
            self.statusBar().showMessage(f"Exported: {path}")
            QMessageBox.information(self, "Export", f"Saved to:\n{path}")
        else:
            QMessageBox.warning(self, "Export", f"Failed to export:\n{path}")

    def _on_save_mesh_done(self, success: bool, path: str):
        self.btn_save_mesh.setEnabled(True)
        if success:
            self.statusBar().showMessage(f"Mesh saved: {path}")
            QMessageBox.information(self, "Save Mesh", f"Saved to:\n{path}")
        else:
            QMessageBox.warning(self, "Save Mesh", "Failed to save mesh.")

    def _on_task_started(self, msg: str):
        self.statusBar().showMessage(msg)

    def _on_task_error(self, msg: str):
        self.scan_status_label.setText(f"Error: {msg}")
        self.statusBar().showMessage(f"Error: {msg}")

    # ── FPS / errors / cleanup ────────────────────────────────────────
    def _update_fps(self):
        now = time.time()
        elapsed = now - self._last_fps_time
        if elapsed > 0:
            self._fps_value = self._fps_counter / elapsed
        self._fps_counter = 0
        self._last_fps_time = now
        self.fps_label.setText(f"FPS: {self._fps_value:.1f}")

    def _on_error(self, msg: str):
        self.kinect_label.setText("Kinect: error")
        self.statusBar().showMessage(f"Error: {msg}")

    def closeEvent(self, event):
        self.task_worker.stop()
        self.task_worker.wait(3000)
        self.server_client.disconnect()
        self.worker.stop()
        self.worker.wait(3000)
        super().closeEvent(event)
