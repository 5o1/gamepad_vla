"""Downstream adapter for the asset repository's latest-state/NVENC stream.

Physics owns publication only. Renderer and gateway may block or die without
waiting in the input -> backend -> recorder loop. No hardware input is exposed.
"""
import argparse
import asyncio
from collections import deque
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from gamepad_vla.site import site_value


def load_config(path):
    import yaml
    cfg = yaml.safe_load(Path(path).read_text())
    if cfg['candidate'] not in ('rt', 'raster') or cfg['fps'] < 30:
        raise ValueError('expected rt/raster and fps >= 30')
    return cfg


class OperatorView:
    """Backend proxy; adds completed-state publication, never render/encode IO."""
    def __init__(self, backend, config, run, spec, port=8780):
        from xbox_gamepad_physics.teleop.stream.ipc import Latest, SNAPSHOT_CAPACITY, FRAME_CAPACITY
        self.backend = backend
        self.run = Path(run).resolve()
        self.run.mkdir(parents=True, exist_ok=False)
        cfg = load_config(config)
        (self.run/'config.json').write_text(json.dumps(cfg, indent=2))
        (self.run/'spec_path.txt').write_text(str(Path(spec).resolve()))
        self.snapshot = Latest(self.run/'snapshot.mmap', SNAPSHOT_CAPACITY, True)
        frame = Latest(self.run/'frame.mmap', FRAME_CAPACITY, True)
        frame.close()
        self.children = []
        self.logs = []
        self.last_commands = []
        self.command_id = 0
        self.command_ns = None
        self.applied_ns = time.monotonic_ns()
        self.step_id = 0
        self.last_tick_ns = None
        self.timing = (self.run/'physics_timing.jsonl').open('w', buffering=1)
        env = dict(os.environ)
        env['PYTHONPATH'] = os.pathsep.join(str(Path(p or '.').resolve())
            for p in env.get('PYTHONPATH', '').split(os.pathsep))
        for role in ('render', 'gateway'):
            log = (self.run/f'{role}.log').open('w')
            self.logs.append(log)
            self.children.append(subprocess.Popen(
                [sys.executable, '-u', '-m', __name__, role, '--run', str(self.run), '--port', str(port)],
                cwd=self.run, env=env, stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True))
        (self.run/'pids.json').write_text(json.dumps({'physics':os.getpid(),
            'render':self.children[0].pid,'gateway':self.children[1].pid,'port':port}))

    def __getattr__(self, key):
        return getattr(self.backend, key)

    def apply(self, commands):
        self.backend.apply(commands)
        self.applied_ns = time.monotonic_ns()
        if commands and (len(commands) != len(self.last_commands) or
                         any(a is not b for a,b in zip(commands,self.last_commands))):
            self.last_commands = list(commands)
            self.command_id += 1
            self.command_ns = self.applied_ns
            self.timing.write(json.dumps({'kind':'command','command_id':self.command_id,
                                         'applied_ns':self.command_ns,'step':self.step_id,
                                         'stamps':[c.stamp for c in commands]})+'\n')

    def observe(self):
        observation = self.backend.observe()
        start = time.monotonic_ns()
        b = self.backend
        snapshot = dict(step=self.step_id, applied_ns=self.applied_ns,
            command_id=self.command_id, command_applied_ns=self.command_ns,
            left_q=b.robot.get_qpos().tolist(),
            right_q=None if b.right_robot is None else b.right_robot.get_qpos().tolist(),
            gamepad_q=b.gamepad.articulation.get_qpos().tolist(),
            signals=observation.gamepad_signal.tolist())
        self.snapshot.write(json.dumps(snapshot, separators=(',', ':')).encode())
        end = time.monotonic_ns()
        self.timing.write(json.dumps({'kind':'tick','step':self.step_id,'applied_ns':self.applied_ns,
            'published_ns':end,'publish_ns':end-start,
            'interval_ns':None if self.last_tick_ns is None else end-self.last_tick_ns})+'\n')
        self.last_tick_ns = end
        self.step_id += 1
        return observation

    def close(self):
        (self.run/'stop').touch()
        for child in self.children:
            try:
                child.wait(timeout=8)
            except subprocess.TimeoutExpired:
                child.terminate()
                try:child.wait(timeout=3)
                except subprocess.TimeoutExpired:child.kill();child.wait()
        self.timing.close()
        self.snapshot.close()
        for log in self.logs:log.close()
        self.backend.close()
        statuses = {role:child.returncode for role,child in zip(('render','gateway'),self.children)}
        (self.run/'worker_exit.json').write_text(json.dumps(statuses))
        if any(statuses.values()):
            raise RuntimeError(f'operator view worker failed: {statuses}; see {self.run}')


def render(run):
    import numpy as np
    import cv2
    from PIL import Image
    import sapien
    from xbox_gamepad_physics.examples.scene import look_at
    from xbox_gamepad_physics.teleop.views import RENDER_CANDIDATES
    from xbox_gamepad_physics.teleop.stream.ipc import Latest, SNAPSHOT_CAPACITY, FRAME_CAPACITY, packet
    from .sapien_backend import SapienBackend
    from .spec import load_spec, pose_to_matrix
    from .render_review import create_spec_camera
    cfg = json.loads((run/'config.json').read_text())
    spec = load_spec((run/'spec_path.txt').read_text())
    source = Latest(run/'snapshot.mmap', SNAPSHOT_CAPACITY)
    output = Latest(run/'frame.mmap', FRAME_CAPACITY)
    # SapienBackend uses the asset example runner's create_scene and fixed lights.
    backend = SapienBackend(spec, render=True, work_dir=run/'render_work')
    settings = RENDER_CANDIDATES[cfg['candidate']]
    # Installed v0.3.1 predates robotwin.apply_review_rendering. The scene
    # already came from example create_scene; select its documented candidate.
    sapien.render.set_camera_shader_dir(settings['shader'])
    sapien.render.set_ray_tracing_samples_per_pixel(settings['samples_per_pixel'])
    sapien.render.set_ray_tracing_denoiser(settings['denoiser'])
    c = cfg['camera']
    camera = backend.scene.add_camera('operator_only', cfg['width'],cfg['height'],
                                     math.radians(c['fovy_degrees']), .001, 20.)
    transform = pose_to_matrix(spec.gamepad.root_pose)
    def world(point):return (transform @ np.r_[point,1])[:3]
    camera.entity.set_pose(look_at(world(c['position_gamepad']),world(c['target_gamepad']),
                                  transform[:3,:3] @ np.array(c['up_gamepad'])))
    pip = cfg.get('pip', {})
    ego = None
    if pip.get('enabled'):
        definition = next(c for c in spec.cameras if c.name == pip.get('camera', 'camera_3'))
        ego = create_spec_camera(backend, definition)
    frame_id = 0
    deadline = time.monotonic()
    captures = set()
    with (run/'render_timing.jsonl').open('w', buffering=1) as timing:
        while not (run/'stop').exists():
            data = source.read()
            if not data:
                time.sleep(.005)
                continue
            delay = deadline-time.monotonic()
            if delay > 0:time.sleep(delay)
            start = time.monotonic_ns()
            deadline = max(deadline+1/cfg['fps'], start/1e9)
            # Read again after sleeping: render the newest completed state.
            data = source.read() or data
            state = json.loads(data[1])
            backend.robot.set_qpos(np.array(state['left_q']))
            if backend.right_robot is not None:backend.right_robot.set_qpos(np.array(state['right_q']))
            backend.gamepad.articulation.set_qpos(np.array(state['gamepad_q']))
            backend.scene.update_render()
            updated_ns = time.monotonic_ns()
            camera.take_picture()
            color = camera.get_picture('Color')
            rgb = cv2.cvtColor(cv2.convertScaleAbs(color,alpha=255),cv2.COLOR_RGBA2RGB)
            main_ns = time.monotonic_ns()
            if ego is not None:
                ego.take_picture()
                small = cv2.cvtColor(cv2.convertScaleAbs(ego.get_picture('Color'),alpha=255),cv2.COLOR_RGBA2RGB)
                width = pip.get('width',320)
                height = round(width*small.shape[0]/small.shape[1])
                rgb[8:8+height,-width-8:-8] = cv2.resize(small,(width,height))
                status = 'coarse' if definition.status == '粗配准' else definition.status
                cv2.putText(rgb,f'ego ({status})',(cfg['width']-width,28),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1)
            end = time.monotonic_ns()
            meta = dict(state, frame_id=frame_id,width=cfg['width'],height=cfg['height'],capture_ns=end,
                        render_ns=end-start,update_ns=updated_ns-start,
                        main_camera_ns=main_ns-updated_ns,pip_ns=end-main_ns)
            for key in ('left_q','right_q','gamepad_q'):meta.pop(key)
            output.write(packet(meta,rgb.tobytes()))
            timing.write(json.dumps(meta)+'\n')
            # Three evidence frames, after a new command has physically settled.
            slot = state['command_id']
            if slot in (1,2,3) and slot not in captures and state['command_applied_ns'] and end-state['command_applied_ns']>700_000_000:
                Image.fromarray(rgb).save(run/f'operator_command_{slot}.jpg',quality=92)
                captures.add(slot)
            frame_id += 1
    source.close();output.close();backend.close()


async def gateway(run, port):
    from aiohttp import web
    from xbox_gamepad_physics.teleop.stream.gateway import client_operation
    from xbox_gamepad_physics.teleop.stream.h264 import AccessUnits, info
    from xbox_gamepad_physics.teleop.stream.ipc import Latest, FRAME_CAPACITY, packet, unpack
    cfg = json.loads((run/'config.json').read_text())
    frames = Latest(run/'frame.mmap', FRAME_CAPACITY)
    peers = {}
    fifo = asyncio.Queue(maxsize=3)
    encoded_count = 0
    codec = None
    encoder_log = (run/'encoder.log').open('w')
    encoder = await asyncio.create_subprocess_exec(
        'ffmpeg','-hide_banner','-loglevel','warning','-threads','2','-filter_threads','2',
        '-f','rawvideo','-pixel_format','rgb24','-video_size',f"{cfg['width']}x{cfg['height']}",
        '-framerate',str(cfg['fps']),'-i','pipe:0','-an','-c:v','h264_nvenc','-preset','p1',
        '-tune','ull','-profile:v','main','-level:v','4.2','-gpu','0','-bf','0','-rc-lookahead','0',
        '-zerolatency','1','-delay','0','-g','30','-b:v','6M','-maxrate','6M','-bufsize','300k',
        '-aud','1','-vsync','0','-flush_packets','1','-f','h264','pipe:1',
        stdin=asyncio.subprocess.PIPE,stdout=asyncio.subprocess.PIPE,stderr=encoder_log)
    async def video(request):
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        peers[ws] = True
        try:
            async for _ in ws:pass
        finally:peers.pop(ws,None)
        return ws
    async def health(request):
        return web.json_response(dict(ready=encoded_count>0,encoded=encoded_count,clients=len(peers),
                                      encoder_alive=encoder.returncode is None))
    async def clock(request):
        return web.json_response({'server_ns':time.monotonic_ns()})
    async def telemetry(request):
        # Client timestamps retain their own clock domain; no fake one-way latency.
        data = await request.json()
        browser.write(json.dumps({'server_received_ns':time.monotonic_ns(),'browser':data})+'\n')
        return web.Response(status=204)
    async def raw_writer():
        generation = 0
        while True:
            data = frames.read()
            if data and data[0]!=generation:
                generation = data[0]
                metadata,rgb = unpack(data[1])
                metadata['encoder_input_ns'] = time.monotonic_ns()
                await fifo.put(metadata)
                encoder.stdin.write(rgb)
                await encoder.stdin.drain()
            else:await asyncio.sleep(.001)
    async def encoded_reader():
        nonlocal codec,encoded_count
        parser = AccessUnits()
        while data := await encoder.stdout.read(65536):
            for unit in parser.feed(data):
                metadata = await fifo.get()
                key,new_codec = info(unit)
                codec = new_codec or codec
                end = time.monotonic_ns()
                metadata.update(key=key,codec=codec,encoded_ns=end,
                                encode_ns=end-metadata['encoder_input_ns'])
                recording.write(unit)
                message = packet(metadata,unit)
                for ws in list(peers):
                    if peers.get(ws) and not key:continue
                    peers[ws] = False
                    try:await client_operation(ws.send_bytes(message),.05)
                    except Exception:
                        peers.pop(ws,None)
                        if ws._req and ws._req.transport:ws._req.transport.abort()
                metadata['sent_ns'] = time.monotonic_ns()
                metadata['clients'] = len(peers)
                timing.write(json.dumps(metadata)+'\n')
                encoded_count += 1
        raise RuntimeError(f'encoder stopped ({encoder.returncode}); see encoder.log')
    root = Path(__file__).with_name('operator_web')
    app = web.Application()
    app.router.add_get('/video',video)
    app.router.add_get('/health',health)
    app.router.add_get('/clock',clock)
    app.router.add_post('/telemetry',telemetry)
    app.router.add_get('/',lambda request:web.FileResponse(root/'index.html'))
    app.router.add_static('/web/',root)
    runner = web.AppRunner(app)
    tasks = []
    browser = (run/'browser_timing.jsonl').open('w',buffering=1)
    timing = (run/'stream_timing.jsonl').open('w',buffering=1)
    recording = (run/'operator.h264').open('wb')
    try:
        await runner.setup()
        await web.TCPSite(runner,site_value('operator.bind_host', '127.0.0.1'),port).start()
        tasks = [asyncio.create_task(raw_writer()),asyncio.create_task(encoded_reader())]
        while not (run/'stop').exists():
            for task in tasks:
                if task.done():task.result()
            await asyncio.sleep(.1)
    finally:
        for task in tasks:task.cancel()
        await asyncio.gather(*tasks,return_exceptions=True)
        for ws in list(peers):await ws.close()
        encoder.stdin.close()
        try:await asyncio.wait_for(encoder.wait(),3)
        except asyncio.TimeoutError:encoder.kill();await encoder.wait()
        await runner.cleanup()
        frames.close();encoder_log.close();browser.close();timing.close();recording.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('role',choices=('render','gateway'))
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--port',type=int,default=8780)
    args = parser.parse_args()
    signal.signal(signal.SIGTERM,lambda *_:(args.run/'stop').touch())
    if args.role=='render':render(args.run)
    else:asyncio.run(gateway(args.run,args.port))


if __name__=='__main__':main()
