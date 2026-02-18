"""FastAPI application wrapping ScanEngine for remote 3D scanning."""

import asyncio
import logging
import os
import tempfile

from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import Response

from shared.protocol import unpack_frame, unpack_frames
from .engine import ScanEngine

logger = logging.getLogger("scanner_server")
logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

app = FastAPI(title="Kinect 3D Scanner Server")
engine = ScanEngine()
_build_lock = asyncio.Lock()

# Connected WebSocket clients for progress broadcasting
_ws_clients: set[WebSocket] = set()


async def _broadcast(msg: dict):
    """Send a JSON message to all connected WebSocket clients."""
    import json
    data = json.dumps(msg)
    closed = set()
    for ws in _ws_clients:
        try:
            await ws.send_text(data)
        except Exception:
            closed.add(ws)
    _ws_clients.difference_update(closed)


# ── Health / Status ────────────────────────────────────────────────────

@app.get("/api/health")
async def health():
    return {
        "status": "ok",
        "stored_count": engine.stored_count,
        "frame_count": engine.frame_count,
        "has_mesh": engine.mesh is not None,
    }


@app.get("/api/scan/status")
async def scan_status():
    return {
        "stored_count": engine.stored_count,
        "frame_count": engine.frame_count,
        "unprocessed_count": engine.unprocessed_count,
        "has_mesh": engine.mesh is not None,
    }


# ── Scan Control ───────────────────────────────────────────────────────

@app.post("/api/scan/reset")
async def scan_reset():
    await asyncio.to_thread(engine.reset)
    logger.info("Scan reset")
    return {"success": True, "message": "Scan reset"}


@app.post("/api/scan/frame")
async def scan_frame(request: Request):
    """Accept a compressed frame (binary body from pack_frame)."""
    body = await request.body()
    if not body:
        return {"success": False, "message": "Empty body"}
    try:
        rgb, depth = unpack_frame(body)
    except Exception as e:
        return {"success": False, "message": f"Unpack error: {e}"}

    result = await asyncio.to_thread(engine.store_frame, rgb, depth)
    return result


@app.post("/api/scan/frames")
async def scan_frames_batch(request: Request):
    """Accept a batch of compressed frames (binary body from pack_frames)."""
    body = await request.body()
    if not body:
        return {"success": False, "message": "Empty body"}

    def _unpack_and_store(raw_data: bytes) -> tuple[int, int]:
        frames = unpack_frames(raw_data)
        for rgb, depth in frames:
            engine.store_frame(rgb, depth)
        return len(frames), engine.stored_count

    try:
        count, total = await asyncio.to_thread(_unpack_and_store, body)
    except Exception as e:
        return {"success": False, "message": f"Batch error: {e}"}

    return {
        "success": True,
        "stored_count": total,
        "batch_size": count,
        "message": f"{count} frames stored (total: {total})",
    }


@app.post("/api/scan/build")
async def scan_build():
    """Process all frames and build the final mesh."""
    if _build_lock.locked():
        return {"success": False, "message": "Build already in progress"}

    async with _build_lock:
        loop = asyncio.get_event_loop()
        progress_queue: asyncio.Queue = asyncio.Queue()

        def progress_cb(current, total, result):
            loop.call_soon_threadsafe(
                progress_queue.put_nowait,
                {"type": "progress", "current": current, "total": total,
                 "message": result.get("message", "")},
            )

        async def drain_progress():
            while True:
                try:
                    msg = progress_queue.get_nowait()
                    await _broadcast(msg)
                except asyncio.QueueEmpty:
                    break

        build_task = asyncio.create_task(
            asyncio.to_thread(engine.build_mesh, progress_cb=progress_cb)
        )

        while not build_task.done():
            await asyncio.wait({build_task}, timeout=0.05)
            await drain_progress()
        await drain_progress()

        success, proc_result = build_task.result()

        if success:
            nv = len(engine.mesh.vertices) if engine.mesh else 0
            nt = len(engine.mesh.triangles) if engine.mesh else 0
            detail = (f"Mesh ready: {nv:,} vertices, {nt:,} triangles "
                      f"({proc_result['frame_count']} frames integrated, "
                      f"{proc_result['errors']} skipped)")
            await _broadcast({"type": "done", "success": True, "detail": detail})
        else:
            detail = "Mesh build failed. Try capturing more frames."
            await _broadcast({"type": "done", "success": False, "detail": detail})

        return {"success": success, "detail": detail, **proc_result}


@app.post("/api/scan/preview")
async def scan_preview():
    """Process frames and return the preview mesh as PLY bytes."""
    if _build_lock.locked():
        return Response(status_code=409, content="Build in progress")

    async with _build_lock:
        loop = asyncio.get_event_loop()
        progress_queue: asyncio.Queue = asyncio.Queue()

        def progress_cb(current, total, result):
            loop.call_soon_threadsafe(
                progress_queue.put_nowait,
                {"type": "progress", "current": current, "total": total,
                 "message": result.get("message", "")},
            )

        async def drain_progress():
            while True:
                try:
                    msg = progress_queue.get_nowait()
                    await _broadcast(msg)
                except asyncio.QueueEmpty:
                    break

        preview_task = asyncio.create_task(
            asyncio.to_thread(engine.extract_preview, progress_cb=progress_cb)
        )

        while not preview_task.done():
            await asyncio.wait({preview_task}, timeout=0.05)
            await drain_progress()
        await drain_progress()

        mesh, pcd, proc_result = preview_task.result()

        if mesh is None and pcd is None:
            await _broadcast({"type": "done", "success": False,
                              "detail": "Preview extraction failed"})
            return {"success": False, "message": "No geometry to preview"}

        # Write to temp PLY and stream back
        import open3d as o3d
        fd, tmp_path = tempfile.mkstemp(suffix=".ply")
        os.close(fd)

        try:
            if mesh is not None and len(mesh.vertices) > 0:
                o3d.io.write_triangle_mesh(tmp_path, mesh)
            elif pcd is not None and len(pcd.points) > 0:
                o3d.io.write_point_cloud(tmp_path, pcd)

            with open(tmp_path, "rb") as f:
                data = f.read()
        finally:
            os.unlink(tmp_path)

        await _broadcast({"type": "done", "success": True,
                          "detail": f"Preview ready ({len(data)} bytes)"})
        return Response(
            content=data,
            media_type="application/octet-stream",
            headers={"Content-Disposition": "attachment; filename=preview.ply"},
        )


# ── Export endpoints ───────────────────────────────────────────────────

@app.get("/api/scan/export/ply")
async def export_ply():
    """Download the mesh as PLY."""
    if engine.mesh is None and engine.point_cloud is None:
        return {"success": False, "message": "No mesh available. Build first."}

    fd, tmp_path = tempfile.mkstemp(suffix=".ply")
    os.close(fd)

    try:
        success = await asyncio.to_thread(engine.export_ply, tmp_path)
        if not success:
            return {"success": False, "message": "Export failed"}
        with open(tmp_path, "rb") as f:
            data = f.read()
    finally:
        os.unlink(tmp_path)

    return Response(
        content=data,
        media_type="application/octet-stream",
        headers={"Content-Disposition": "attachment; filename=scan.ply"},
    )


@app.get("/api/scan/export/obj")
async def export_obj():
    """Download the mesh as OBJ."""
    if engine.mesh is None:
        return {"success": False, "message": "No mesh available. Build first."}

    fd, tmp_path = tempfile.mkstemp(suffix=".obj")
    os.close(fd)

    try:
        success = await asyncio.to_thread(engine.export_obj, tmp_path)
        if not success:
            return {"success": False, "message": "Export failed"}
        with open(tmp_path, "rb") as f:
            data = f.read()
    finally:
        os.unlink(tmp_path)

    return Response(
        content=data,
        media_type="application/octet-stream",
        headers={"Content-Disposition": "attachment; filename=scan.obj"},
    )


# ── WebSocket for progress ─────────────────────────────────────────────

@app.websocket("/ws/progress")
async def ws_progress(websocket: WebSocket):
    await websocket.accept()
    _ws_clients.add(websocket)
    logger.info("WebSocket client connected (%d total)", len(_ws_clients))
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        pass
    finally:
        _ws_clients.discard(websocket)
        logger.info("WebSocket client disconnected (%d remaining)", len(_ws_clients))
