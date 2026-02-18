"""Reusable widgets and utility functions for the scanner GUI."""

import cv2
import numpy as np
from PyQt6.QtGui import QImage


def colorize_depth(depth: np.ndarray, near: int, far: int) -> np.ndarray:
    """Convert a uint16 depth map to a JET-colorized RGB image."""
    d = depth.astype(np.float32)
    valid = (d > 0) & (d < far)
    norm = np.zeros_like(d, dtype=np.uint8)
    if valid.any():
        norm[valid] = (255.0 * (d[valid] - near) / max(far - near, 1)).clip(
            0, 255
        ).astype(np.uint8)
    colored = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
    colored[~valid] = 0
    return cv2.cvtColor(colored, cv2.COLOR_BGR2RGB)


def numpy_to_qimage(arr: np.ndarray) -> QImage:
    """Convert an RGB numpy array to a QImage."""
    h, w, ch = arr.shape
    return QImage(arr.data, w, h, ch * w, QImage.Format.Format_RGB888)
