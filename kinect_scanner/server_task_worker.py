"""Background task queue dispatching work to ServerClient.

Same pattern as the original ScanTaskManager, but sends requests to the
remote server instead of calling ScanEngine directly.
"""

import queue
import traceback
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Any

from PyQt6.QtCore import QThread


class ServerTaskType(Enum):
    SEND_FRAME = auto()
    BUILD_MESH = auto()
    PREVIEW = auto()
    EXPORT_PLY = auto()
    EXPORT_OBJ = auto()
    SAVE_MESH = auto()
    RESET = auto()


@dataclass
class ServerTask:
    task_type: ServerTaskType
    kwargs: dict[str, Any] = field(default_factory=dict)


class ServerTaskWorker(QThread):
    """Sequential task runner backed by a thread-safe queue.

    Dispatches to ServerClient methods instead of local ScanEngine.
    """

    def __init__(self, client, parent=None):
        super().__init__(parent)
        self._client = client
        self._queue: queue.Queue[ServerTask | None] = queue.Queue()
        self._stop_flag = False

    def submit(self, task: ServerTask):
        self._queue.put(task)

    def stop(self):
        self._stop_flag = True
        self._queue.put(None)

    def run(self):
        while not self._stop_flag:
            task = self._queue.get()
            if task is None:
                break
            try:
                self._dispatch(task)
            except Exception as exc:
                traceback.print_exc()
                self._client.task_error.emit(
                    f"{task.task_type.name}: {exc}"
                )

    _MAX_BATCH = 100  # max frames per HTTP request

    def _drain_send_frames(self, first_task: ServerTask) -> dict:
        """Collect all queued SEND_FRAME tasks and send as one batch."""
        frames = [(first_task.kwargs["rgb"], first_task.kwargs["depth"])]
        requeue = []

        while not self._queue.empty():
            try:
                t = self._queue.get_nowait()
            except queue.Empty:
                break
            if t is None:
                self._queue.put(None)
                break
            if t.task_type == ServerTaskType.SEND_FRAME:
                frames.append((t.kwargs["rgb"], t.kwargs["depth"]))
            else:
                requeue.append(t)

        for t in requeue:
            self._queue.put(t)

        # Send in sub-batches
        result = None
        for i in range(0, len(frames), self._MAX_BATCH):
            chunk = frames[i : i + self._MAX_BATCH]
            if len(chunk) == 1:
                result = self._client.send_frame(chunk[0][0], chunk[0][1])
            else:
                result = self._client.send_frames_batch(chunk)

        return result

    def _dispatch(self, task: ServerTask):
        tt = task.task_type

        if tt == ServerTaskType.SEND_FRAME:
            result = self._drain_send_frames(task)
            self._client.frame_stored.emit(result)

        elif tt == ServerTaskType.RESET:
            self._client.task_started.emit("Resetting scan on server...")
            self._client.reset_scan()

        elif tt == ServerTaskType.BUILD_MESH:
            self._client.task_started.emit("Building mesh on server...")
            result = self._client.request_build()
            success = result.get("success", False)
            detail = result.get("detail", "Build complete")
            # build_mesh_done is also emitted via WebSocket "done" message,
            # but emit here as well for reliability
            self._client.build_mesh_done.emit(success, detail)

        elif tt == ServerTaskType.PREVIEW:
            self._client.task_started.emit("Generating preview on server...")
            path = self._client.request_preview()
            if path:
                self._client.preview_done.emit(path)
            else:
                self._client.task_error.emit("Preview failed — no geometry")

        elif tt == ServerTaskType.EXPORT_PLY:
            path = task.kwargs["path"]
            self._client.task_started.emit(f"Downloading PLY to {path}...")
            success = self._client.request_export("ply", path)
            self._client.export_done.emit(success, path)

        elif tt == ServerTaskType.EXPORT_OBJ:
            path = task.kwargs["path"]
            self._client.task_started.emit(f"Downloading OBJ to {path}...")
            success = self._client.request_export("obj", path)
            self._client.export_done.emit(success, path)

        elif tt == ServerTaskType.SAVE_MESH:
            path = task.kwargs["path"]
            self._client.task_started.emit(f"Saving mesh to {path}...")
            success = self._client.request_export("ply", path)
            self._client.save_mesh_done.emit(success, path)
