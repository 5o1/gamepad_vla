"""Completed-state latest-frame streaming, isolated from physics and recording.

Retains the existing operator view's independent renderer/gateway architecture;
MJPEG supports CPU-only MuJoCo without requiring NVENC. Overlay uses the asset
project's licensed renderer. Commands only control recording, never hardware.
"""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import asyncio
import json
import multiprocessing as mp
from pathlib import Path
import queue
import time

from .collection_health import require_loopback, detect_safety, FPS_MIN, LATENCY_P95_MAX_MS


class OperatorStream:
    def __init__(self, backend, spec, resources, output, host, port, environment_collisions=True):
        require_loopback(host)
        ctx = mp.get_context("spawn")
        self.snapshots, self.controls = ctx.Queue(maxsize=1), ctx.Queue(maxsize=32)
        self.stop = ctx.Event()
        self.ready = ctx.Event()
        self.first_frame = ctx.Event()
        self.worker = ctx.Process(target=_serve, args=(backend, str(spec), resources, str(output), host,
                                                       port, self.snapshots, self.controls, self.stop, self.ready, self.first_frame, environment_collisions))
        self.worker.start()

    def wait_ready(self, cancelled=None, timeout=120):
        deadline = time.monotonic()+timeout
        while not self.ready.wait(.1):
            if not self.worker.is_alive():
                raise RuntimeError(f"operator stream exited with code {self.worker.exitcode}")
            if cancelled and cancelled.is_set():
                return False
            if time.monotonic()>deadline:
                raise RuntimeError("operator stream startup exceeded 120 s")
        return True

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

    def close(self, *, cancelled=False):
        if cancelled and not self.ready.is_set():
            # Initialization has no episode journal. Cancel only this owned
            # renderer rather than waiting for CPU asset compilation to finish.
            self.stop.set()
            if self.worker.is_alive():self.worker.terminate()
            self.worker.join(timeout=15)
            if self.worker.is_alive():raise RuntimeError('owned initializing renderer did not exit')
            return
        # A short replay may finish before its first image is encoded. Drain
        # that first image before shutting down the local video response.
        if not cancelled and self.ready.is_set() and self.worker.is_alive():
            self.first_frame.wait(timeout=10)
            time.sleep(.1)
        self.stop.set()
        self.worker.join(timeout=15)
        if self.worker.is_alive():
            self.worker.terminate()
            self.worker.join()
            raise RuntimeError("operator stream did not shut down cleanly")
        if self.worker.exitcode:
            raise RuntimeError(f"operator stream failed: exit {self.worker.exitcode}")


def _serve(name, spec_path, resources, output, host, port, snapshots, controls, stop, ready, first_frame, environment_collisions):
    from gamepad_vla.workstation.collection import create_backend
    from gamepad_vla.workstation.spec import load_spec
    import cv2
    from aiohttp import web
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='ego-renderer')
    def initialize():
        backend = create_backend(name, load_spec(spec_path), work_dir=output / 'scene', render=True,
                                 resource_root=resources, environment_collisions=environment_collisions)
        from .collection_camera import EgoRenderer
        return backend, EgoRenderer(backend)
    backend, ego = executor.submit(initialize).result()
    root = Path(__file__).with_name("collection_web")
    last_row = None
    jpeg = None
    frame_count = 0
    samples = deque(maxlen=600)
    safety = {}
    log = (output / "frames.jsonl").open("x", buffering=1)

    def frame_state():
        now = time.monotonic_ns()
        recent = [s for s in samples if now-s[0] <= 10e9]
        import numpy as np
        fps = (len(recent)-1)/((now-recent[0][0])/1e9) if len(recent)>1 else 0.
        delays = [s[1] for s in recent if s[1] is not None]
        p50, p95 = (map(float, np.percentile(delays, [50,95])) if delays else (None,None))
        return dict(last_row or {}, stream=dict(frames=frame_count, fps=fps,
            apply_to_encode_p50_ms=p50, apply_to_encode_p95_ms=p95,
            warning=fps<FPS_MIN or p95 is None or p95>LATENCY_P95_MAX_MS))

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
            def encode_frame():
                t0 = time.monotonic_ns()
                backend.set_render_state(row)
                t1 = time.monotonic_ns()
                rgb = ego.render()
                t2 = time.monotonic_ns()
                ok, encoded = cv2.imencode('.jpg', cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY,85])
                t3 = time.monotonic_ns()
                return ok, encoded, dict(render_started_ns=t0, state_sync_ms=(t1-t0)/1e6,
                    render_rgb_ms=(t2-t1)/1e6, jpeg_ms=(t3-t2)/1e6, **ego.timings)
            ok, encoded, stages = await asyncio.get_running_loop().run_in_executor(executor, encode_frame)
            if not ok:
                raise RuntimeError("JPEG encoder failed")
            jpeg = encoded.tobytes()
            last_row = row
            frame_count += 1
            first_frame.set()
            encoded_ns = time.monotonic_ns()
            applied_ns = row.get("applied_monotonic_ns")
            samples.append((encoded_ns, None if applied_ns is None else (encoded_ns-applied_ns)/1e6))
            log.write(json.dumps(dict(frame_id=frame_count, sim_time_s=row["sim_time_s"],
                                      encoded_monotonic_ns=encoded_ns, applied_monotonic_ns=applied_ns,
                                      apply_to_encode_ms=None if applied_ns is None else (encoded_ns - applied_ns) / 1e6,
                                      render_encode_s=time.monotonic() - started, **stages)) + "\n")
            # Leave scheduling headroom for the >=30 fps acceptance target.
            # Only new completed states are rendered; no duplicate frames.
            await asyncio.sleep(max(0, 1 / 60 - (time.monotonic() - started)))

    async def run():
        from types import SimpleNamespace
        from .collection_server import make_app, SessionManager
        class AttachedManager(SessionManager):
            def __init__(self):
                self.root=output
                self.output=output.parent
                self.controls=controls
                self.worker=SimpleNamespace(is_alive=lambda:not stop.is_set())
                self.port=None
                self.active_options={}
                self.latest={}
                self.pending_start=False
                self.export_process=None
            def refresh(self):
                self.latest=dict(last_row or {},session_state='running')
                return self.latest
            async def start(self, values):
                raise ValueError('CLI 已持有仿真会话，请在 CLI 终端结束它。')
            async def stop(self, client='web'):
                raise ValueError('请在 CLI 终端停止仿真会话。')
        app = make_app(AttachedManager(),video_handler=video,frame_provider=frame_state,managed=False)
        # Private render telemetry only; public clients use /api/v1/status.
        async def render_state(request):return web.json_response(frame_state())
        app.router.add_get('/state',render_state)
        runner = web.AppRunner(app)
        await runner.setup()
        await web.TCPSite(runner, host, port).start()
        ready.set()
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
        executor.submit(backend.close).result()
        executor.shutdown(wait=True)
        log.close()
