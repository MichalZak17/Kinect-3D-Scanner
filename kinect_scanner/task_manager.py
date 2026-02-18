"""Background task queue for ScanEngine operations.

All heavy computation (ICP, marching cubes, export) runs on this single
QThread so the GUI stays responsive.  The engine is not thread-safe, so
tasks are processed sequentially from a queue.

Frame storage (STORE_FRAME) is near-instant.  Processing (PROCESS_FRAMES,
BUILD_MESH, EXTRACT_PREVIEW) runs the heavy ICP + TSDF pipeline on demand.
"""

import queue
import traceback
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Any

from PyQt6.QtCore import QThread, pyqtSignal


class TaskType(Enum):
    STORE_FRAME = auto()
    BUILD_MESH = auto()
    EXTRACT_PREVIEW = auto()
    EXPORT_PLY = auto()
    EXPORT_OBJ = auto()
    SAVE_MESH = auto()


@dataclass
class Task:
    task_type: TaskType
    kwargs: dict[str, Any] = field(default_factory=dict)


class ScanTaskManager(QThread):
    """Sequential task runner backed by a thread-safe queue."""

    frame_stored = pyqtSignal(dict)          # result from store_frame
    process_progress = pyqtSignal(int, int, dict)  # current, total, frame_result
    build_mesh_done = pyqtSignal(bool, str)
    preview_done = pyqtSignal(object, object)
    export_done = pyqtSignal(bool, str)
    save_mesh_done = pyqtSignal(bool, str)
    task_started = pyqtSignal(str)
    task_error = pyqtSignal(str)

    def __init__(self, engine, parent=None):
        super().__init__(parent)
        self._engine = engine
        self._queue: queue.Queue[Task | None] = queue.Queue()
        self._stop_flag = False

    def submit(self, task: Task):
        """Thread-safe enqueue of a task."""
        self._queue.put(task)

    def stop(self):
        """Signal the thread to exit and unblock the queue."""
        self._stop_flag = True
        self._queue.put(None)  # sentinel

    def run(self):
        while not self._stop_flag:
            task = self._queue.get()
            if task is None:
                break
            try:
                self._dispatch(task)
            except Exception as exc:
                traceback.print_exc()
                self.task_error.emit(f"{task.task_type.name}: {exc}")

    def _drain_store_frames(self, task: Task) -> Task:
        """Keep only the latest STORE_FRAME, skip older ones."""
        latest = task
        requeue = []
        while not self._queue.empty():
            try:
                t = self._queue.get_nowait()
            except queue.Empty:
                break
            if t is None:
                self._queue.put(None)
                break
            if t.task_type == TaskType.STORE_FRAME:
                # Store all of them (they're instant), just batch them
                self._engine.store_frame(
                    t.kwargs["rgb"], t.kwargs["depth"]
                )
            else:
                requeue.append(t)
        for t in requeue:
            self._queue.put(t)
        return latest

    def _progress_cb(self, current, total, result):
        """Emit progress signal during frame processing."""
        self.process_progress.emit(current, total, result)

    def _dispatch(self, task: Task):
        tt = task.task_type

        if tt == TaskType.STORE_FRAME:
            # Store this frame, then drain and store any queued ones
            result = self._engine.store_frame(
                task.kwargs["rgb"], task.kwargs["depth"]
            )
            self._drain_store_frames(task)
            # Emit with updated count after draining
            result["stored_count"] = self._engine.stored_count
            result["message"] = f"Frame {self._engine.stored_count} stored"
            self.frame_stored.emit(result)

        elif tt == TaskType.BUILD_MESH:
            unprocessed = self._engine.unprocessed_count
            if unprocessed > 0:
                self.task_started.emit(
                    f"Processing {unprocessed} frames..."
                )
            success, proc_result = self._engine.build_mesh(
                progress_cb=self._progress_cb
            )
            if success:
                self.task_started.emit("Building mesh (marching cubes)...")
                nv = len(self._engine.mesh.vertices) if self._engine.mesh else 0
                nt = len(self._engine.mesh.triangles) if self._engine.mesh else 0
                detail = (f"Mesh ready: {nv:,} vertices, {nt:,} triangles "
                          f"({proc_result['frame_count']} frames integrated, "
                          f"{proc_result['errors']} skipped)")
            else:
                detail = "Mesh build failed. Try capturing more frames."
            self.build_mesh_done.emit(success, detail)

        elif tt == TaskType.EXTRACT_PREVIEW:
            unprocessed = self._engine.unprocessed_count
            if unprocessed > 0:
                self.task_started.emit(
                    f"Processing {unprocessed} frames for preview..."
                )
            else:
                self.task_started.emit("Extracting preview...")
            mesh, pcd, proc_result = self._engine.extract_preview(
                progress_cb=self._progress_cb
            )
            self.preview_done.emit(mesh, pcd)

        elif tt == TaskType.EXPORT_PLY:
            path = task.kwargs["path"]
            self.task_started.emit(f"Exporting PLY to {path}...")
            success = self._engine.export_ply(path)
            self.export_done.emit(success, path)

        elif tt == TaskType.EXPORT_OBJ:
            path = task.kwargs["path"]
            self.task_started.emit(f"Exporting OBJ to {path}...")
            success = self._engine.export_obj(path)
            self.export_done.emit(success, path)

        elif tt == TaskType.SAVE_MESH:
            path = task.kwargs["path"]
            self.task_started.emit(f"Saving mesh to {path}...")
            success = self._engine.export_ply(path)
            self.save_mesh_done.emit(success, path)
