"""Camera intrinsics, view modes, and scan presets.

Re-exports everything from shared.config and adds the Open3D intrinsic object.
"""

from shared.config import *  # noqa: F401,F403

import open3d as o3d

O3D_INTRINSIC = o3d.camera.PinholeCameraIntrinsic(
    DEPTH_W, DEPTH_H, FX, FY, CX, CY  # noqa: F405
)
