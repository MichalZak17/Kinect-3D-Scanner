"""KinectWorker — background thread for continuous frame grabbing."""

import time

import freenect
import numpy as np
from PyQt6.QtCore import QThread, pyqtSignal


class KinectWorker(QThread):
    """Grabs RGB + registered-depth frames via freenect sync interface."""

    frame_ready = pyqtSignal(np.ndarray, np.ndarray)  # (rgb, depth_mm)
    error_occurred = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._running = False

    def stop(self):
        self._running = False

    def run(self):
        self._running = True
        while self._running:
            try:
                depth_result = freenect.sync_get_depth(
                    format=freenect.DEPTH_REGISTERED
                )
                if depth_result is None:
                    self.error_occurred.emit("Failed to get depth frame")
                    time.sleep(0.1)
                    continue
                depth, _ = depth_result

                vid_result = freenect.sync_get_video(format=freenect.VIDEO_RGB)
                if vid_result is None:
                    self.error_occurred.emit("Failed to get video frame")
                    time.sleep(0.1)
                    continue
                video, _ = vid_result

                self.frame_ready.emit(video.copy(), depth.copy())
            except Exception as e:
                self.error_occurred.emit(str(e))
                time.sleep(0.1)

        try:
            freenect.sync_stop()
        except Exception:
            pass
