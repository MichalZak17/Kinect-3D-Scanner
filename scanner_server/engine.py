"""ScanEngine — CPU-based 3D scanning pipeline using Open3D tensor API.

Two-phase workflow:
  Phase 1 (capture): Store raw RGB+depth frames as fast as possible.
  Phase 2 (process): On demand, process stored frames through the
      registration + TSDF integration pipeline, then extract mesh.

Processing is incremental — only unprocessed frames are run through ICP/TSDF,
so mid-scan previews don't re-process already-integrated frames.

TSDF integration and mesh extraction use VoxelBlockGrid on CPU for stability.
Registration (ICP / FGR) also runs on CPU (legacy API).
"""

import logging
import time
import traceback

import numpy as np
import open3d as o3d
import open3d.core as o3c
import trimesh

from .config import O3D_INTRINSIC, O3D_INTRINSIC_TENSOR
from shared.config import ScanPreset, PRESET_DEFAULT

_REG = o3d.pipelines.registration
logger = logging.getLogger("scanner_server")

class ScanEngine:

    MODEL_REFRESH_INTERVAL = 5  # re-extract model_pcd every N integrations
    BLOCK_COUNT = 50000         # VBG block budget (CPU, RAM is plentiful)

    def __init__(self):
        self.preset = PRESET_DEFAULT
        self.device = o3c.Device("CPU:0")
        logger.info("ScanEngine using device: %s", self.device)
        self.reset()

    # ── VBG creation ──────────────────────────────────────────────────

    def _create_vbg(self):
        """Create a fresh VoxelBlockGrid on self.device."""
        return o3d.t.geometry.VoxelBlockGrid(
            attr_names=("tsdf", "weight", "color"),
            attr_dtypes=(o3c.float32, o3c.float32, o3c.float32),
            attr_channels=((1,), (1,), (3,)),
            voxel_size=self.voxel_size,
            block_resolution=16,
            block_count=self.BLOCK_COUNT,
            device=self.device,
        )

    # ── reset ─────────────────────────────────────────────────────────

    def reset(self, preset: ScanPreset | None = None):
        if preset is not None:
            self.preset = preset
        p = self.preset
        self.voxel_size = p.voxel_size
        self.sdf_trunc = p.sdf_trunc
        self.max_depth_m = p.max_depth_m
        self.reg_voxel = p.voxel_size * p.reg_voxel_mult

        self.vbg = self._create_vbg()

        self.frame_count = 0
        self.model_pcd = None
        self._model_fpfh = None
        self._integrations_since_model = 0
        self.cumulative_T = np.eye(4)
        self.mesh = None
        self.point_cloud = None

        # Raw frame storage
        self.raw_frames: list[tuple[np.ndarray, np.ndarray]] = []
        self._processed_count = 0

    # ── helpers ────────────────────────────────────────────────────────

    def _make_rgbd(self, rgb, depth):
        """Create legacy Open3D RGBDImage for registration point cloud.

        depth is uint16 in mm from DEPTH_REGISTERED.
        """
        depth_f = depth.astype(np.float32)
        depth_f[(depth == 0) | (depth > self.max_depth_m * 1000)] = 0.0

        color_o3d = o3d.geometry.Image(np.ascontiguousarray(rgb, dtype=np.uint8))
        depth_o3d = o3d.geometry.Image(np.ascontiguousarray(depth_f, dtype=np.float32))
        return o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_o3d, depth_o3d,
            depth_scale=1000.0,
            depth_trunc=self.max_depth_m,
            convert_rgb_to_intensity=False,
        )

    def _integrate_vbg(self, rgb, depth, extrinsic):
        """Run TSDF integration via VoxelBlockGrid.

        Creates tensor images, integrates, and frees them immediately.
        """
        depth_img = o3d.t.geometry.Image(
            o3c.Tensor(np.ascontiguousarray(depth))
        ).to(self.device)
        color_img = o3d.t.geometry.Image(
            o3c.Tensor(np.ascontiguousarray(rgb))
        ).to(self.device)
        extrinsic_t = o3c.Tensor(extrinsic, dtype=o3c.float64)

        frustum_block_coords = self.vbg.compute_unique_block_coordinates(
            depth_img, O3D_INTRINSIC_TENSOR, extrinsic_t,
            depth_scale=1000.0,
            depth_max=self.max_depth_m,
        )
        self.vbg.integrate(
            frustum_block_coords,
            depth_img, color_img,
            O3D_INTRINSIC_TENSOR, O3D_INTRINSIC_TENSOR,
            extrinsic_t,
            depth_scale=1000.0,
            depth_max=self.max_depth_m,
        )
        del depth_img, color_img, frustum_block_coords, extrinsic_t

    def _make_reg_pcd(self, rgbd):
        """Build a coarser point cloud for registration (not integration)."""
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, O3D_INTRINSIC)
        pcd = pcd.voxel_down_sample(self.reg_voxel)
        pcd.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(
                radius=self.reg_voxel * 4, max_nn=30
            )
        )
        return pcd

    def _extract_model_pcd(self):
        """Extract and downsample a model point cloud from the TSDF volume.

        Also precomputes FPFH features for the model (cached for fallback).
        """
        t_pcd = self.vbg.extract_point_cloud(weight_threshold=0.5)
        pcd = t_pcd.to_legacy()
        del t_pcd
        pcd = pcd.voxel_down_sample(self.reg_voxel)
        pcd.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(
                radius=self.reg_voxel * 4, max_nn=30
            )
        )
        self.model_pcd = pcd

        # Precompute FPFH for fallback — avoids recomputing on every fallback
        search_param = o3d.geometry.KDTreeSearchParamHybrid(
            radius=self.reg_voxel * 5, max_nn=100
        )
        self._model_fpfh = _REG.compute_fpfh_feature(pcd, search_param)

        self._integrations_since_model = 0

    def _icp(self, source, target, init=np.eye(4)):
        """Single-stage point-to-plane ICP with early-exit criteria."""
        vs = self.voxel_size
        p = self.preset
        return _REG.registration_icp(
            source, target,
            max_correspondence_distance=vs * p.icp_coarse_mult,
            init=init,
            estimation_method=_REG.TransformationEstimationPointToPlane(),
            criteria=_REG.ICPConvergenceCriteria(
                relative_fitness=1e-6,
                relative_rmse=1e-6,
                max_iteration=30,
            ),
        )

    def _fpfh_fallback(self, source, target):
        """Fast Global Registration using cached model FPFH, refined with ICP."""
        search_param = o3d.geometry.KDTreeSearchParamHybrid(
            radius=self.reg_voxel * 5, max_nn=100
        )
        src_feat = _REG.compute_fpfh_feature(source, search_param)
        tgt_feat = self._model_fpfh  # cached from _extract_model_pcd

        fgr = _REG.registration_fgr_based_on_feature_matching(
            source, target, src_feat, tgt_feat,
            _REG.FastGlobalRegistrationOption(
                maximum_correspondence_distance=self.reg_voxel * 3,
            ),
        )

        # Refine with ICP
        refined = self._icp(source, target, init=fgr.transformation)
        return refined

    def _register(self, source_pcd):
        """Register source (camera coords) against model_pcd (world coords).

        Uses cumulative_T as initial guess so ICP starts near the true pose.
        Returns (result, method_str) or (None, error_str).
        """
        threshold = self.preset.fitness_threshold

        # Stage 1: point-to-plane ICP seeded with last known pose
        result = self._icp(source_pcd, self.model_pcd,
                           init=self.cumulative_T)
        if result.fitness >= threshold:
            return result, "icp"

        # Stage 2: Fast Global Registration fallback
        try:
            result = self._fpfh_fallback(source_pcd, self.model_pcd)
        except Exception as e:
            return None, f"FGR fallback failed: {e}"

        if result.fitness >= threshold:
            return result, "global"

        return None, (f"Poor alignment (fitness={result.fitness:.2f}). "
                      "Move slower or add more overlap.")

    # ── public API: capture (fast) ─────────────────────────────────────
    def store_frame(self, rgb: np.ndarray, depth: np.ndarray) -> dict:
        """Store a raw frame for later processing. Very fast — no ICP/TSDF."""
        idx = len(self.raw_frames)
        self.raw_frames.append((rgb, depth))
        return {
            "success": True,
            "stored_count": idx + 1,
            "message": f"Frame {idx + 1} stored",
        }

    @property
    def stored_count(self) -> int:
        return len(self.raw_frames)

    @property
    def unprocessed_count(self) -> int:
        return len(self.raw_frames) - self._processed_count

    # ── public API: process on demand ──────────────────────────────────
    def process_frames(self, progress_cb=None) -> dict:
        """Process all unprocessed stored frames through ICP + TSDF.

        This is incremental — only frames after _processed_count are processed.
        progress_cb(current_idx, total, result_dict) is called after each frame.

        Returns summary dict.
        """
        total = len(self.raw_frames)
        start = self._processed_count
        processed = 0
        errors = 0

        for i in range(start, total):
            rgb, depth = self.raw_frames[i]
            result = self._process_single_frame(rgb, depth)
            processed += 1
            if not result["success"]:
                errors += 1
            if progress_cb:
                progress_cb(i + 1, total, result)

        return {
            "processed": processed,
            "errors": errors,
            "total": total,
            "frame_count": self.frame_count,
        }

    def _process_single_frame(self, rgb: np.ndarray, depth: np.ndarray) -> dict:
        """Process a single frame through ICP registration + TSDF integration."""
        t0 = time.monotonic()

        # Legacy RGBD + registration point cloud (CPU)
        rgbd = self._make_rgbd(rgb, depth)
        current_pcd = self._make_reg_pcd(rgbd)

        if len(current_pcd.points) < 100:
            self._processed_count += 1
            return {"success": False, "frame_count": self.frame_count,
                    "message": "Too few depth points. Skipped."}

        # First frame — integrate directly (need model_pcd immediately)
        if self.frame_count == 0:
            extrinsic = np.linalg.inv(self.cumulative_T)
            self._integrate_vbg(rgb, depth, extrinsic)
            self._extract_model_pcd()
            self.frame_count = 1
            self._processed_count += 1
            elapsed_ms = (time.monotonic() - t0) * 1000
            return {"success": True, "frame_count": 1,
                    "message": f"Reference frame ({elapsed_ms:.0f}ms)"}

        # Register current → model
        try:
            result, method = self._register(current_pcd)
        except Exception as e:
            self._processed_count += 1
            return {"success": False, "frame_count": self.frame_count,
                    "message": f"Registration failed: {e}"}

        if result is None:
            self._processed_count += 1
            return {"success": False, "frame_count": self.frame_count,
                    "message": method}

        # Update pose
        self.cumulative_T = result.transformation
        self.frame_count += 1
        self._integrations_since_model += 1
        self._processed_count += 1

        # TSDF integration
        extrinsic = np.linalg.inv(self.cumulative_T)
        self._integrate_vbg(rgb, depth, extrinsic)

        # Refresh model periodically
        should_extract = (
            method == "global"
            or self._integrations_since_model >= self.MODEL_REFRESH_INTERVAL
        )
        if should_extract:
            self._extract_model_pcd()

        elapsed_ms = (time.monotonic() - t0) * 1000

        if method == "global":
            msg = (f"Frame {self.frame_count} via global "
                   f"(fitness={result.fitness:.3f}, {elapsed_ms:.0f}ms)")
        else:
            msg = (f"Frame {self.frame_count} "
                   f"(fitness={result.fitness:.3f}, "
                   f"RMSE={result.inlier_rmse:.4f}, {elapsed_ms:.0f}ms)")

        return {
            "success": True,
            "frame_count": self.frame_count,
            "message": msg,
        }

    def extract_preview(self, progress_cb=None):
        """Process unprocessed frames, then extract current mesh/point cloud.

        Returns (mesh, point_cloud, process_result) or (None, None, result).
        """
        process_result = self.process_frames(progress_cb=progress_cb)
        if self.frame_count == 0:
            return None, None, process_result
        try:
            t_mesh = self.vbg.extract_triangle_mesh(weight_threshold=0.5)
            mesh = t_mesh.to_legacy()
            del t_mesh
            mesh.compute_vertex_normals()
            t_pcd = self.vbg.extract_point_cloud(weight_threshold=0.5)
            pcd = t_pcd.to_legacy()
            del t_pcd
            return mesh, pcd, process_result
        except Exception:
            traceback.print_exc()
            return None, None, process_result

    def build_mesh(self, progress_cb=None) -> tuple[bool, dict]:
        """Process unprocessed frames, then build final mesh.

        Returns (success, process_result).
        """
        process_result = self.process_frames(progress_cb=progress_cb)
        if self.frame_count == 0:
            return False, process_result
        try:
            t_mesh = self.vbg.extract_triangle_mesh(weight_threshold=0.5)
            self.mesh = t_mesh.to_legacy()
            del t_mesh
            self.mesh.compute_vertex_normals()
            t_pcd = self.vbg.extract_point_cloud(weight_threshold=0.5)
            self.point_cloud = t_pcd.to_legacy()
            del t_pcd
            return True, process_result
        except Exception:
            traceback.print_exc()
            return False, process_result

    def export_ply(self, filepath: str, as_mesh: bool = True) -> bool:
        try:
            if as_mesh and self.mesh is not None:
                o3d.io.write_triangle_mesh(filepath, self.mesh)
            elif self.point_cloud is not None:
                o3d.io.write_point_cloud(filepath, self.point_cloud)
            else:
                return False
            return True
        except Exception:
            traceback.print_exc()
            return False

    def export_obj(self, filepath: str) -> bool:
        if self.mesh is None:
            return False
        try:
            verts = np.asarray(self.mesh.vertices)
            faces = np.asarray(self.mesh.triangles)
            colors = np.asarray(self.mesh.vertex_colors)
            tm = trimesh.Trimesh(
                vertices=verts, faces=faces,
                vertex_colors=(colors * 255).astype(np.uint8)
                if len(colors) > 0 else None,
            )
            tm.export(filepath)
            return True
        except Exception:
            traceback.print_exc()
            return False

    def shutdown(self):
        """Clean up resources. Call on application exit."""
        pass
