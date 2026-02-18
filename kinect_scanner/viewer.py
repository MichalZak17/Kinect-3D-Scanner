"""3D mesh/point cloud preview using Open3D visualizer with WASD+QE controls.

Can also be invoked as a subprocess:
    python -m kinect_scanner.viewer '{"mode":"file","filepath":"scan.ply"}'
"""

import json
import os
import subprocess
import sys
import tempfile

import numpy as np
import open3d as o3d
import trimesh


# Movement/rotation step sizes (tuned for smoother control)
_MOVE_STEP = 0.08
_ROTATE_DEG = 5.0


def _run_viewer(geometries, window_name="Kinect 3D Scanner — Preview"):
    """Open an interactive Open3D window with WASD+QE keyboard controls.

    Controls:
        W/S — move forward/backward
        A/D — strafe left/right
        Q/E — pitch up/down (vertical rotation)
        Mouse — default Open3D controls (drag to orbit, scroll to zoom)
    """
    if not geometries:
        return

    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(window_name=window_name, width=1024, height=768)

    for g in geometries:
        vis.add_geometry(g)

    def _get_camera(v):
        ctr = v.get_view_control()
        param = ctr.convert_to_pinhole_camera_parameters()
        return ctr, param

    def _move(vis, dx, dy, dz):
        ctr, param = _get_camera(vis)
        ext = np.array(param.extrinsic)
        # Camera axes: right=col0, down=col1, forward=col2 (in camera frame)
        right = ext[:3, 0]
        forward = ext[:3, 2]
        # Translate the camera (moving the extrinsic origin)
        ext[:3, 3] += right * dx + forward * dz
        param.extrinsic = ext
        ctr.convert_from_pinhole_camera_parameters(param, allow_arbitrary=True)
        return False

    def _rotate_pitch(vis, angle_deg):
        """Rotate around the camera's local X-axis (pitch up/down)."""
        ctr, param = _get_camera(vis)
        ext = np.array(param.extrinsic)
        rad = np.radians(angle_deg)
        cos_a, sin_a = np.cos(rad), np.sin(rad)
        # Rotation around X axis
        rot = np.eye(4)
        rot[1, 1] = cos_a
        rot[1, 2] = -sin_a
        rot[2, 1] = sin_a
        rot[2, 2] = cos_a
        param.extrinsic = ext @ rot
        ctr.convert_from_pinhole_camera_parameters(param, allow_arbitrary=True)
        return False

    # WASD movement
    vis.register_key_callback(ord('W'), lambda v: _move(v, 0, 0, _MOVE_STEP))
    vis.register_key_callback(ord('S'), lambda v: _move(v, 0, 0, -_MOVE_STEP))
    vis.register_key_callback(ord('A'), lambda v: _move(v, -_MOVE_STEP, 0, 0))
    vis.register_key_callback(ord('D'), lambda v: _move(v, _MOVE_STEP, 0, 0))

    # QE vertical rotation (pitch)
    vis.register_key_callback(ord('Q'), lambda v: _rotate_pitch(v, _ROTATE_DEG))
    vis.register_key_callback(ord('E'), lambda v: _rotate_pitch(v, -_ROTATE_DEG))

    vis.get_render_option().point_size = 2.0
    vis.reset_view_point(True)
    vis.run()
    vis.destroy_window()


def show_mesh_preview(mesh=None, point_cloud=None):
    """Open an interactive Open3D window to inspect the scan result.

    Shows the mesh if available, otherwise falls back to the point cloud.
    The window is blocking — the Qt main window freezes while it's open.
    """
    geometries = []
    if mesh is not None and len(mesh.vertices) > 0:
        geometries.append(mesh)
    elif point_cloud is not None and len(point_cloud.points) > 0:
        geometries.append(point_cloud)

    _run_viewer(geometries)


def show_file_preview(filepath: str) -> bool:
    """Load a mesh/point cloud file from disk and show it in the Open3D viewer.

    Supports OBJ, PLY, and STL files. Returns False if loading fails.
    """
    ext = filepath.lower().rsplit(".", 1)[-1] if "." in filepath else ""

    if ext == "obj":
        try:
            tm = trimesh.load(filepath)
            mesh = o3d.geometry.TriangleMesh()
            mesh.vertices = o3d.utility.Vector3dVector(tm.vertices)
            mesh.triangles = o3d.utility.Vector3iVector(tm.faces)
            if hasattr(tm.visual, "vertex_colors") and tm.visual.vertex_colors is not None:
                colors = tm.visual.vertex_colors[:, :3].astype(float) / 255.0
                mesh.vertex_colors = o3d.utility.Vector3dVector(colors)
            mesh.compute_vertex_normals()
            show_mesh_preview(mesh=mesh)
            return True
        except Exception:
            return False

    elif ext == "ply":
        try:
            mesh = o3d.io.read_triangle_mesh(filepath)
            if len(mesh.vertices) > 0:
                mesh.compute_vertex_normals()
                show_mesh_preview(mesh=mesh)
                return True
            pcd = o3d.io.read_point_cloud(filepath)
            if len(pcd.points) > 0:
                show_mesh_preview(point_cloud=pcd)
                return True
            return False
        except Exception:
            return False

    elif ext == "stl":
        try:
            mesh = o3d.io.read_triangle_mesh(filepath)
            if len(mesh.vertices) == 0:
                return False
            mesh.compute_vertex_normals()
            show_mesh_preview(mesh=mesh)
            return True
        except Exception:
            return False

    return False


# ── Subprocess-based viewer (non-blocking) ────────────────────────────


def launch_viewer_subprocess(filepath: str):
    """Launch the viewer in a separate process. Returns immediately."""
    args = json.dumps({"mode": "file", "filepath": os.path.abspath(filepath)})
    subprocess.Popen(
        [sys.executable, "-m", "kinect_scanner.viewer", args],
        start_new_session=True,
    )


def launch_mesh_viewer(mesh, point_cloud):
    """Write mesh/pcd to a temp PLY and launch a subprocess viewer.

    The subprocess deletes the temp file after the viewer closes.
    """
    fd, tmp_path = tempfile.mkstemp(suffix=".ply")
    os.close(fd)

    if mesh is not None and len(mesh.vertices) > 0:
        o3d.io.write_triangle_mesh(tmp_path, mesh)
    elif point_cloud is not None and len(point_cloud.points) > 0:
        o3d.io.write_point_cloud(tmp_path, point_cloud)
    else:
        os.unlink(tmp_path)
        return

    args = json.dumps({"mode": "mesh_data", "temp_path": tmp_path})
    subprocess.Popen(
        [sys.executable, "-m", "kinect_scanner.viewer", args],
        start_new_session=True,
    )


# ── Subprocess entry point ────────────────────────────────────────────

def _main():
    """Entry point when invoked as ``python -m kinect_scanner.viewer <json>``."""
    if len(sys.argv) < 2:
        print("Usage: python -m kinect_scanner.viewer '<json_args>'")
        sys.exit(1)

    cfg = json.loads(sys.argv[1])
    mode = cfg.get("mode")

    if mode == "file":
        filepath = cfg["filepath"]
        if not show_file_preview(filepath):
            print(f"Failed to load: {filepath}")
            sys.exit(1)

    elif mode == "mesh_data":
        tmp_path = cfg["temp_path"]
        try:
            if not show_file_preview(tmp_path):
                print(f"Failed to load temp file: {tmp_path}")
                sys.exit(1)
        finally:
            # Clean up temp file after viewer closes
            try:
                os.unlink(tmp_path)
            except OSError:
                pass
    else:
        print(f"Unknown mode: {mode}")
        sys.exit(1)


if __name__ == "__main__":
    _main()
