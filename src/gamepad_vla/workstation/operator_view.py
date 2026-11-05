"""Downstream adapter for the asset repository's latest-state/NVENC stream.

Physics owns publication only. Renderer and gateway may block or die without
waiting in the input -> backend -> recorder loop. No hardware input is exposed.
"""
import argparse
import asyncio
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from gamepad_vla.site import site_value
from .runtime_logs import diagnostic_file, CappedProcessLog


def load_config(path):
    import yaml
    cfg = yaml.safe_load(Path(path).read_text())
    if cfg['candidate'] not in ('rt', 'raster') or cfg['fps'] < 30:
        raise ValueError('expected rt/raster and fps >= 30')
    return cfg


class OperatorView:
    """Backend proxy; adds completed-state publication, never render/encode IO."""
    def __init__(self, backend, config, run, spec, port=8780, *, collection=False, host="127.0.0.1", resources=None, diagnostics=False):
        from xbox_gamepad_physics.teleop.stream.ipc import Latest, SNAPSHOT_CAPACITY
        from .collection_health import require_loopback
        require_loopback(host)
        self.backend = backend
        self.run = Path(run).resolve()
        self.run.mkdir(parents=True, exist_ok=False)
        cfg = load_config(config)
        cfg.update(collection=collection, host=host, resources=None if resources is None else str(resources),
                   diagnostics=diagnostics is True, diagnostics_started=time.monotonic())
        if collection:
            from .scene_snapshot import code_identity
            cfg.update(spec_text=Path(spec).read_bytes().decode('utf-8'), spec_name=Path(spec).name, code_identity=code_identity())
            from .spec import load_spec
            intr = next(c.intrinsics for c in load_spec(spec).cameras if c.name == 'camera_3')
            # Collection main view is always the unmodified calibrated camera_3.
            size = cfg.get('collection_main', {})
            cfg.update(width=max(intr.width, size.get('width',1280)), height=max(intr.height, size.get('height',720)), view='camera_3')
            cfg['pip'] = dict(enabled=False)
        self.collection = collection
        self.roles = ('render', 'auxiliary', 'gateway') if collection else ('render', 'gateway')
        self.port = port
        self._event_offset = 0
        (self.run/'controls.jsonl').touch()
        (self.run/'config.json').write_text(json.dumps(cfg, indent=2))
        (self.run/'spec_path.txt').write_text(str(Path(spec).resolve()))
        self.snapshot = Latest(self.run/'snapshot.mmap', SNAPSHOT_CAPACITY, True)
        from .auxiliary_views import frame_capacity, auxiliary_options
        frame = Latest(self.run/'frame.mmap', frame_capacity(cfg['width'],cfg['height']), True)
        frame.close()
        if collection:
            options = auxiliary_options(cfg)
            for name, capacity in (('aux-frame', frame_capacity(options['width'],options['height'])), ('aux-demand', 4096)):
                mmap = Latest(self.run/(name+'.mmap'), capacity, True)
                mmap.close()
        self.children = []
        self.logs = []
        self.last_commands = []
        self.command_id = 0
        self.command_ns = None
        self.applied_ns = time.monotonic_ns()
        self.step_id = 0
        self.last_tick_ns = None
        self.timing = diagnostic_file(self.run, 'physics_timing.jsonl', cfg)
        env = dict(os.environ)
        env['PYTHONPATH'] = os.pathsep.join(str(Path(p or '.').resolve())
            for p in env.get('PYTHONPATH', '').split(os.pathsep))
        for role in self.roles:
            log = CappedProcessLog(self.run/f'{role}.log')
            self.logs.append(log)
            self.children.append(subprocess.Popen(
                [sys.executable, '-u', '-m', __name__, role, '--run', str(self.run), '--port', str(port)],
                cwd=self.run, env=env, stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True))
        (self.run/'pids.json').write_text(json.dumps({'physics':os.getpid(),
            **{role: child.pid for role, child in zip(self.roles, self.children)},'port':port}))

    def wait_ready(self, cancelled=None, timeout=120):
        deadline = time.monotonic()+timeout
        while not (self.run/'ready').exists():
            if any(c.poll() is not None for c in self.children):
                raise RuntimeError(f'operator view worker exited; see {self.run}')
            if cancelled and cancelled.is_set():return False
            if time.monotonic()>deadline:raise RuntimeError('operator view startup exceeded 120 s')
            time.sleep(.05)
        return True

    def publish(self, row):
        if any(c.poll() is not None for c in self.children):
            raise RuntimeError(f'operator view worker exited; see {self.run}')
        state = dict(row, step=self.step_id, applied_ns=row['applied_monotonic_ns'],
                     signals=row['gamepad_signal'], command_id=0, command_applied_ns=None)
        self.snapshot.write(json.dumps(state,separators=(',', ':')).encode())
        self.step_id += 1

    def events(self):
        with (self.run/'controls.jsonl').open() as f:
            f.seek(self._event_offset)
            rows = f.readlines()
            self._event_offset = f.tell()
        return [json.loads(row) for row in rows]

    def set_view(self, view):
        if view not in ('camera_3','operator'):raise ValueError('unknown camera view')
        if self.collection and view != 'camera_3':
            raise ValueError('Collection main view is fixed to camera_3. Use the positioning panes.')
        temporary = self.run/'view.tmp'
        temporary.write_text(json.dumps(dict(view=view)))
        temporary.replace(self.run/'view.json')

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
            self.timing.json({'kind':'command','command_id':self.command_id,
                                         'applied_ns':self.command_ns,'step':self.step_id,
                                         'stamps':[c.stamp for c in commands]})

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
        self.timing.json({'kind':'tick','step':self.step_id,'applied_ns':self.applied_ns,
            'published_ns':end,'publish_ns':end-start,
            'interval_ns':None if self.last_tick_ns is None else end-self.last_tick_ns})
        self.last_tick_ns = end
        self.step_id += 1
        return observation

    def close(self, *, cancelled=False):
        if self.collection and not cancelled and (self.run/'ready').exists():
            deadline=time.monotonic()+10
            while not (self.run/'first_frame').exists() and all(c.poll() is None for c in self.children) and time.monotonic()<deadline:
                time.sleep(.025)
        (self.run/'stop').touch()
        for child in self.children:
            try:
                child.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGTERM)
                try:child.wait(timeout=3)
                except subprocess.TimeoutExpired:os.killpg(child.pid, signal.SIGKILL);child.wait()
        self.timing.close()
        self.snapshot.close()
        for log in self.logs:log.close()
        if self.backend is not None:self.backend.close()
        statuses = {role:child.returncode for role,child in zip(self.roles,self.children)}
        (self.run/'worker_exit.json').write_text(json.dumps(statuses))
        if any(statuses.values()):
            raise RuntimeError(f'operator view worker failed: {statuses}; see {self.run}')


def apply_render_state(backend, state, *, collection=True):
    """Apply one completed physical state to the existing operator scene."""
    import numpy as np
    if collection:
        backend.set_review_state({side+'_'+kind: state[kind+'_q'][i]
            for i, side in enumerate(('left', 'right')) for kind in ('arm', 'hand')})
        backend.gamepad.articulation.set_qpos(np.array([state['control_joint_positions'][j.name]
            for j in backend.gamepad.articulation.get_active_joints()]))
    else:
        backend.robot.set_qpos(np.array(state['left_q']))
        if backend.right_robot is not None:
            backend.right_robot.set_qpos(np.array(state['right_q']))
        backend.gamepad.articulation.set_qpos(np.array(state['gamepad_q']))
    backend.scene.update_render()


def configure_operator_rendering(cfg):
    import sapien
    from xbox_gamepad_physics.teleop.views import RENDER_CANDIDATES
    sapien.render.set_camera_shader_dir(RENDER_CANDIDATES[cfg['candidate']]['shader'])
    if cfg.get('collection') and cfg['candidate'] == 'raster':
        # The operator pipeline produces RGB8. Avoid downloading RGBA float32
        # (4x the bytes) before converting to the same 8-bit display format.
        sapien.render.set_picture_format('Color', 'r8g8b8a8unorm')


def render_auxiliary(run):
    """Same operator scene and mmap pipeline, isolated from the 60 Hz main view."""
    import sapien
    from xbox_gamepad_physics.teleop.views import RENDER_CANDIDATES
    from xbox_gamepad_physics.teleop.stream.ipc import Latest, SNAPSHOT_CAPACITY, packet
    from .auxiliary_views import AuxiliaryCameras, auxiliary_options, frame_capacity, atlas_tiles
    from .sapien_backend import SapienBackend
    from .spec import load_spec
    from .scene_snapshot import render_pending_snapshot
    cfg = json.loads((run/'config.json').read_text())
    source = Latest(run/'snapshot.mmap', SNAPSHOT_CAPACITY)
    demand = Latest(run/'aux-demand.mmap', 4096)
    options = auxiliary_options(cfg)
    output = Latest(run/'aux-frame.mmap', frame_capacity(options['width'],options['height']))
    backend = cameras = None
    last_job = None
    frame_id = 0
    deadline = time.monotonic()
    try:
        while not (run/'stop').exists():
            wanted = demand.read()
            names = json.loads(wanted[1]) if wanted else []
            job_path = run/'scene-snapshot-request.json'
            job = json.loads(job_path.read_text()) if job_path.exists() else None
            pending = job and job['id'] != last_job
            if not names and not pending:
                time.sleep(.05)
                continue
            if backend is None:
                configure_operator_rendering(cfg)
                if cfg.get('resources'):
                    os.environ['XBOX_GAMEPAD_PHYSICS_RESOURCES'] = cfg['resources']
                spec = load_spec((run/'spec_path.txt').read_text())
                backend = SapienBackend(spec, render=True, work_dir=run/'aux-render-work')
                cameras = AuxiliaryCameras(backend,cfg)
            if pending:
                render_pending_snapshot(backend, job, cfg)
                last_job = job['id']
                continue
            data = source.read()
            if not data:
                time.sleep(.005)
                continue
            time.sleep(max(0, deadline-time.monotonic()))
            start = time.monotonic_ns()
            deadline = max(deadline+1/options['fps'], start/1e9)
            # Re-read both state and demand after sleeping; a closed pane gets no picture.
            wanted = demand.read()
            names = json.loads(wanted[1]) if wanted else []
            if not names:
                continue
            state = json.loads((source.read() or data)[1])
            apply_render_state(backend, state)
            rgb = cameras.capture(names)
            end = time.monotonic_ns()
            meta = dict(frame_id=frame_id, width=options['width'], height=options['height'], fps=options['fps'],
                        views=names, tiles=atlas_tiles(cfg), capture_ns=end, render_ns=end-start,
                        applied_ns=state['applied_ns'], step=state.get('step'), stream='auxiliary')
            output.write(packet(meta, rgb.tobytes()))
            frame_id += 1
    finally:
        source.close(); demand.close(); output.close()
        if backend is not None:
            backend.close()


def render(run):
    import numpy as np
    import cv2
    from PIL import Image
    import sapien
    from xbox_gamepad_physics.examples.scene import look_at
    from xbox_gamepad_physics.teleop.views import RENDER_CANDIDATES
    from xbox_gamepad_physics.teleop.stream.ipc import Latest, SNAPSHOT_CAPACITY, packet
    from .sapien_backend import SapienBackend
    from .spec import load_spec, pose_to_matrix
    from .render_review import create_spec_camera
    cfg = json.loads((run/'config.json').read_text())
    spec = load_spec((run/'spec_path.txt').read_text())
    source = Latest(run/'snapshot.mmap', SNAPSHOT_CAPACITY)
    from .auxiliary_views import frame_capacity, scaled_camera_definition
    output = Latest(run/'frame.mmap', frame_capacity(cfg['width'],cfg['height']))
    # Select raster BEFORE constructing a scene, rather than initializing RT
    # and changing its shader after the fact.
    configure_operator_rendering(cfg)
    if cfg.get('resources'):
        os.environ['XBOX_GAMEPAD_PHYSICS_RESOURCES'] = cfg['resources']
    backend = SapienBackend(spec, render=True, work_dir=run/'render_work')
    camera = None
    if not cfg.get('collection') or cfg.get('review_camera'):
        c = cfg['camera']
        camera = backend.scene.add_camera('operator_only', cfg['width'],cfg['height'],
                                         math.radians(c['fovy_degrees']), .001, 20.)
        transform = pose_to_matrix(spec.gamepad.root_pose)
        def world(point):return (transform @ np.r_[point,1])[:3]
        camera.entity.set_pose(look_at(world(c['position_gamepad']),world(c['target_gamepad']),
                                      transform[:3,:3] @ np.array(c['up_gamepad'])))
    pip = cfg.get('pip', {})
    ego = None
    if pip.get('enabled') or cfg.get('collection'):
        definition = next(c for c in spec.cameras if c.name == pip.get('camera', 'camera_3'))
        ego = create_spec_camera(backend, scaled_camera_definition(definition, cfg['width'], cfg['height']) if cfg.get('collection') else definition)
    over = cfg.get('overview', {})
    overview = None
    if over.get('enabled') and not cfg.get('collection'):
        # World-fixed debug camera above the table centre, looking straight down;
        # never a calibrated dataset camera.
        overview = backend.scene.add_camera('overview_only', over.get('render_width', 640),
                                            over.get('render_height', 360),
                                            math.radians(over['fovy_degrees']), .01, 20.)
        overview.entity.set_pose(look_at(over['position'], over['target'], over['up']))
    frame_id = 0
    deadline = time.monotonic()
    captures = set()
    with diagnostic_file(run, 'render_timing.jsonl', cfg) as timing:
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
            apply_render_state(backend, state, collection=cfg.get('collection'))
            updated_ns = time.monotonic_ns()
            selected = cfg.get('view','operator')
            if not cfg.get('collection') or cfg.get('review_camera'):
                if (run/'view.json').is_file():selected=json.loads((run/'view.json').read_text())['view']
            active_camera = ego if (selected=='camera_3' or camera is None) else camera
            active_camera.take_picture()
            taken_ns = time.monotonic_ns()
            from .collection_camera import color_to_rgb8
            color = active_camera.get_picture('Color')
            downloaded_ns = time.monotonic_ns()
            rgb = color_to_rgb8(color)
            main_ns = time.monotonic_ns()
            if ego is not None and pip.get("enabled"):
                ego.take_picture()
                small = cv2.cvtColor(cv2.convertScaleAbs(ego.get_picture('Color'),alpha=255),cv2.COLOR_RGBA2RGB)
                width = pip.get('width',320)
                height = round(width*small.shape[0]/small.shape[1])
                rgb[8:8+height,-width-8:-8] = cv2.resize(small,(width,height))
                status = 'coarse' if definition.status == '\u7c97\u914d\u51c6' else definition.status
                cv2.putText(rgb,f'ego ({status})',(cfg['width']-width,28),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1)
            if overview is not None:
                overview.take_picture()
                small = color_to_rgb8(overview.get_picture('Color'))
                width = over.get('width', 480)
                height = round(width*small.shape[0]/small.shape[1])
                rgb[8:8+height,8:8+width] = cv2.resize(small,(width,height))
                cv2.putText(rgb,'overview',(16,28),cv2.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),1)
            end = time.monotonic_ns()
            meta = dict(view=selected, frame_id=frame_id,width=cfg['width'],height=cfg['height'],capture_ns=end,
                        render_ns=end-start,update_ns=updated_ns-start,
                        main_camera_ns=main_ns-updated_ns,pip_ns=end-main_ns,
                        camera_render_ns=taken_ns-updated_ns,camera_download_ns=downloaded_ns-taken_ns,
                        color_convert_ns=main_ns-downloaded_ns)
            # Only the live stream needs overlay data; never journal simulation state.
            meta.update({k:state[k] for k in ('step','applied_ns','command_id','command_applied_ns') if k in state})
            live = dict(meta, **{k:state[k] for k in ('overlay','signals','gamepad_signal','sim_time_s','arm_clamps','candidate_side') if k in state})
            output.write(packet(live,rgb.tobytes()))
            timing.json(meta)
            # Three evidence frames, after a new command has physically settled.
            slot = state['command_id']
            if timing.enabled and slot in (1,2,3) and slot not in captures and state['command_applied_ns'] and end-state['command_applied_ns']>700_000_000:
                Image.fromarray(rgb).save(run/f'operator_command_{slot}.jpg',quality=92)
                captures.add(slot)
            frame_id += 1
    source.close();output.close();backend.close()


async def gateway(run, port):
    from aiohttp import web
    from .operator_stream import OperatorStream
    cfg = json.loads((run/'config.json').read_text())
    main_stream = OperatorStream(run, cfg)
    auxiliary = OperatorStream(run, cfg, auxiliary=True) if cfg.get('collection') else None
    streams = [main_stream] + ([auxiliary] if auxiliary else [])
    browser = diagnostic_file(run, 'browser_timing.jsonl', cfg)

    def frame_state():
        result = main_stream.state()
        if auxiliary:
            result['auxiliary_stream'] = auxiliary.state()['stream']
        return result

    async def health(request):
        return web.json_response(dict(ready=main_stream.encoded_count>0,
            encoded=main_stream.encoded_count, clients=len(main_stream.peers),
            encoder_alive=bool(main_stream.encoder and main_stream.encoder.returncode is None),
            auxiliary=auxiliary.state()['stream'] if auxiliary else None))

    async def clock(request):
        return web.json_response({'server_ns':time.monotonic_ns()})

    async def telemetry(request):
        data = await request.json()
        browser.json(dict(server_received_ns=time.monotonic_ns(), browser=dict(frames=[
            {k:v for k,v in frame.items() if k in ('encoded_ns','command_id','server_clock_estimate_ns',
                'clock_rtt_ms','presented_ms','received_ms') and (v is None or isinstance(v,(int,float)))}
            for frame in data.get('frames',[])[:120]])))
        return web.Response(status=204)

    if cfg.get('collection'):
        from types import SimpleNamespace
        from .collection_server import make_app, SessionManager
        from xbox_gamepad_physics.teleop.stream.ipc import Latest, SNAPSHOT_CAPACITY
        physical = Latest(run/'snapshot.mmap', SNAPSHOT_CAPACITY)
        class Controls:
            def put_nowait(self,event):
                with (run/'controls.jsonl').open('a') as f:
                    f.write(json.dumps(event)+'\n')
        class AttachedManager(SessionManager):
            def __init__(self):
                self.root=run; self.output=run.parent; self.controls=Controls()
                self.worker=SimpleNamespace(is_alive=lambda:not (run/'stop').exists())
                self.port=None; self.active_options={}; self.latest={}; self.pending_start=False; self.export_process=None
                self.view_run=run
            def refresh(self):
                data=physical.read()
                self.latest=dict(json.loads(data[1]) if data else {},session_state='running')
                self.latest.pop('snapshot_inputs',None)  # Snapshot worker reads the mmap directly.
                return self.latest
            async def start(self,values):
                raise ValueError('CLI owns this session. End it in the CLI terminal.')
            async def stop(self,client='web'):
                raise ValueError('Stop the simulation session in the CLI terminal.')
        app = make_app(AttachedManager(), video_handler=main_stream.video,
            auxiliary_video_handler=auxiliary.video, frame_provider=frame_state, managed=False)
        async def state(request):
            return web.json_response(frame_state())
        app.router.add_get('/state',state)
    else:
        app=web.Application()
        app.router.add_get('/video',main_stream.video)
        root=Path(__file__).with_name('operator_web')
        app.router.add_get('/',lambda request:web.FileResponse(root/'index.html'))
        app.router.add_static('/web/',root)
    app.router.add_get('/health',health)
    app.router.add_get('/clock',clock)
    app.router.add_post('/telemetry',telemetry)
    runner = web.AppRunner(app, access_log=None)
    try:
        await runner.setup()
        await web.TCPSite(runner,cfg.get('host',site_value('operator.bind_host','127.0.0.1')),port).start()
        (run/'ready').touch()
        for stream in streams:
            stream.start()
        while not (run/'stop').exists():
            for stream in streams:
                for task in stream.tasks:
                    if task.done():
                        task.result()
            await asyncio.sleep(.1)
    finally:
        for stream in streams:
            await stream.close()
        await runner.cleanup()
        if cfg.get('collection'):
            physical.close()
        browser.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('role',choices=('render','auxiliary','gateway'))
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--port',type=int,default=8780)
    args = parser.parse_args()
    signal.signal(signal.SIGTERM,lambda *_:(args.run/'stop').touch())
    if args.role=='render':render(args.run)
    elif args.role=='auxiliary':render_auxiliary(args.run)
    else:asyncio.run(gateway(args.run,args.port))


if __name__=='__main__':main()
