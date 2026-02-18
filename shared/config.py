"""Camera intrinsics, view modes, and scan presets (pure Python — no Open3D)."""

from dataclasses import dataclass

# ── Kinect v1 camera intrinsics (factory defaults) ───────────────────────
FX, FY = 594.21, 591.04
CX, CY = 339.31, 242.74
DEPTH_W, DEPTH_H = 640, 480

# ── View modes ────────────────────────────────────────────────────────────
MODE_RGB = "RGB"
MODE_DEPTH = "Depth"
MODE_SCANNER = "Scanner"


# ── Scan presets ──────────────────────────────────────────────────────────
@dataclass
class ScanPreset:
    name: str
    voxel_size: float        # metres
    sdf_trunc: float         # metres
    max_depth_m: float       # metres
    icp_coarse_mult: float   # multiplier on voxel_size for coarse ICP
    reg_voxel_mult: float    # multiplier on voxel_size for registration cloud
    fitness_threshold: float
    depth_near_mm: int
    depth_far_mm: int


PRESET_DEFAULT = ScanPreset(
    name="Default",
    voxel_size=0.005,
    sdf_trunc=0.04,
    max_depth_m=4.0,
    icp_coarse_mult=15.0,
    reg_voxel_mult=3.0,
    fitness_threshold=0.25,
    depth_near_mm=500,
    depth_far_mm=4000,
)
