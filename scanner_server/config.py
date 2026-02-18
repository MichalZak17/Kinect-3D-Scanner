"""Server-side config — creates the Open3D intrinsic object."""

from shared.config import *  # noqa: F401,F403

import open3d as o3d
import open3d.core as o3c

O3D_INTRINSIC = o3d.camera.PinholeCameraIntrinsic(
    DEPTH_W, DEPTH_H, FX, FY, CX, CY  # noqa: F405
)

# 3×3 intrinsic tensor for VoxelBlockGrid (tensor API)
O3D_INTRINSIC_TENSOR = o3c.Tensor(
    [[FX, 0, CX], [0, FY, CY], [0, 0, 1]],  # noqa: F405
    dtype=o3c.float64,
)
