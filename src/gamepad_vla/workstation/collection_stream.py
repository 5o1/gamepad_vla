"""Completed-state latest-frame streaming, isolated from physics and recording.

Retains the existing operator view's independent renderer/gateway architecture;
MJPEG supports CPU-only MuJoCo without requiring NVENC. Overlay uses the asset
project's licensed renderer. Commands only control recording, never hardware.
"""
import asyncio
import json
import multiprocessing as mp
from pathlib import Path
import queue
import time


class OperatorStream:
    def __init__(self, backend, spec, resources, output, host, port, environment_collisions=True):
        ctx = mp.get_context("spawn")
        self.snapshots, self.controls = ctx.Queue(maxsize=1), ctx.Queue(maxsize=32)
        self.stop = ctx.Event()
        self.worker = ctx.Process(target=_serve, args=(backend, str(spec), resources, str(output), host,
                                                       port, self.snapshots, self.controls, self.stop, environment_collisions))
        self.worker.start()

    def publish(self, row):
        if not self.worker.is_alive():
            raise RuntimeError(f"operator stream exited with code {self.worker.exitcode}")
        try:
            self.snapshots.put_nowait(row)
        except queue.Full:
            try:
                self.snapshots.get_nowait()
                self.snapshots.put_nowait(row)
            except (queue.Full, queue.Empty):
                pass

    def events(self):
        events = []
        while True:
            try:
                events.append(self.controls.get_nowait())
            except queue.Empty:
                return events

    def close(self):
        self.stop.set()
        self.worker.join(timeout=15)
        if self.worker.is_alive():
            self.worker.terminate()
            self.worker.join()
            raise RuntimeError("operator stream did not shut down cleanly")
        if self.worker.exitcode:
            raise RuntimeError(f"operator stream failed: exit {self.worker.exitcode}")


def _serve(name, spec_path, resources, output, host, port, snapshots, controls, stop, environment_collisions):
    from gamepad_vla.workstation.collection import create_backend
    from gamepad_vla.workstation.spec import load_spec
    import cv2
    from aiohttp import web
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    backend = create_backend(name, load_spec(spec_path), work_dir=output / "scene", render=True,
                             resource_root=resources, environment_collisions=environment_collisions)
    root = Path(__file__).with_name("collection_web")
    last_row = None
    jpeg = None
    frame_count = 0
    log = (output / "frames.jsonl").open("x", buffering=1)

    async def index(request):
        return web.FileResponse(root / "index.html")

    async def state(request):
        return web.json_response(last_row or {})

    async def control(request):
        event = await request.json()
        if event.get("action") not in {"start", "stop", "note"}:
            raise web.HTTPBadRequest(text="expected start/stop/note")
        try:
            controls.put_nowait(event)
        except queue.Full:
            raise web.HTTPServiceUnavailable(text="recording control queue is full")
        return web.json_response({"queued": True})

    async def video(request):
        response = web.StreamResponse(headers={"Content-Type": "multipart/x-mixed-replace; boundary=frame"})
        await response.prepare(request)
        seen = -1
        try:
            while not stop.is_set():
                if jpeg is not None and seen != frame_count:
                    data, seen = jpeg, frame_count
                    await asyncio.wait_for(response.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + data + b"\r\n"), 2)
                await asyncio.sleep(.005)
        except (ConnectionError, asyncio.TimeoutError):
            pass
        return response

    async def render():
        nonlocal last_row, jpeg, frame_count
        while not stop.is_set():
            started = time.monotonic()
            try:
                row = snapshots.get_nowait()
            except queue.Empty:
                await asyncio.sleep(.002)
                continue
            backend.set_render_state(row)
            rgb = backend.render_rgb()
            ok, encoded = cv2.imencode(".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 85])
            if not ok:
                raise RuntimeError("JPEG encoder failed")
            jpeg = encoded.tobytes()
            last_row = row
            frame_count += 1
            encoded_ns = time.monotonic_ns()
            applied_ns = row.get("applied_monotonic_ns")
            log.write(json.dumps(dict(frame_id=frame_count, sim_time_s=row["sim_time_s"],
                                      encoded_monotonic_ns=encoded_ns, applied_monotonic_ns=applied_ns,
                                      apply_to_encode_ms=None if applied_ns is None else (encoded_ns - applied_ns) / 1e6,
                                      render_encode_s=time.monotonic() - started)) + "\n")
            # Leave scheduling headroom for the >=30 fps acceptance target.
            # Only new completed states are rendered; no duplicate frames.
            await asyncio.sleep(max(0, 1 / 60 - (time.monotonic() - started)))

    async def run():
        app = web.Application()
        app.router.add_get("/", index)
        app.router.add_get("/state", state)
        app.router.add_get("/video", video)
        app.router.add_post("/control", control)
        app.router.add_static("/overlay", root / "overlay")
        runner = web.AppRunner(app)
        await runner.setup()
        await web.TCPSite(runner, host, port).start()
        task = asyncio.create_task(render())
        try:
            while not stop.is_set():
                if task.done():
                    task.result()
                await asyncio.sleep(.05)
        finally:
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
            await runner.cleanup()
    try:
        asyncio.run(run())
    finally:
        backend.close()
        log.close()
