"""Single simulation/recording path used by CLI and local web sessions."""
from dataclasses import dataclass
import json
import os
from pathlib import Path
import queue
import time

from gamepad_vla.site import configure_runtime, repository_root, site_value
from .arm_mapping import ArmMapper
from .collection import Session, create_backend
from .collection_sources import CollectionReplay, CollectionUdpSource, Ros2Source
from .spec import load_spec
from .collection_health import require_loopback


@dataclass
class CollectionOptions:
    backend: str
    source: str
    output: Path
    duration: float | None
    spec: Path = repository_root()/'configs/workcell/station_g20_v4_4.json'
    scene_variant: str = 'full'
    mujoco_workers: int = 1
    replay: Path | None = None
    ros_topic_prefix: str = ''
    loop_replay: bool = False
    resource_root: Path | None = None
    rate_hz: int = 30
    realtime: bool = False
    record: bool = False
    note: str = ''
    view: bool = False
    host: str = '127.0.0.1'
    port: int = 8780
    udp_host: str = '127.0.0.1'
    udp_port: int = 50200

    def validate(self):
        if self.backend not in ('sapien', 'mujoco') or self.source not in ('ros2', 'replay', 'udp'):
            raise ValueError('invalid simulation backend or source')
        if self.scene_variant not in ('full', 'free-space'):
            raise ValueError('invalid scene variant')
        if (self.duration is not None and (self.duration <= 0 or not __import__('math').isfinite(self.duration))) or self.rate_hz <= 0:
            raise ValueError('duration and rate must be positive (null duration means unlimited)')
        if self.mujoco_workers < 1 or (self.backend != 'mujoco' and self.mujoco_workers != 1):
            raise ValueError('--mujoco-workers must be positive and applies only to MuJoCo')
        if self.source == 'replay' and self.replay is None:
            raise ValueError('--replay is required')
        require_loopback(self.host)


class CollectionController:
    def __init__(self, options, *, controls=None, stop=None, publish=None):
        options.validate()
        self.options = options
        self.controls = controls if controls is not None else queue.Queue()
        self.stop = stop
        self.publish = publish or (lambda state: None)
        self.session = self.source = self.backend = self.view = None
        self.last_row = {}
        self.state = 'starting'
        self.error = None
        self.receiver_start = 0
        self.last_publish = 0.
        self.control_result = None

    def status(self):
        return dict(self.last_row, session_state=self.state, error=self.error,
                    backend=self.options.backend, scene_variant=self.options.scene_variant, source=self.options.source,
                    control_result=self.control_result,
                    recording_status=self.session.status() if self.session else dict(state='idle'),
                    input_health=self.source.health.snapshot() if self.source else {},
                    output=str(self.options.output))

    def emit(self, force=False):
        if force or time.monotonic()-self.last_publish >= .1:
            self.publish(self.status())
            if force and self.view and self.last_row:
                self.view.publish(self.status())
            self.last_publish = time.monotonic()

    def boundary(self):
        events, count = self.source.boundary()
        self.session.record_inputs(events)
        return count

    def control(self, event):
        action = event.get('action')
        request_id = event.get('request_id')
        self.control_result = dict(request_id=request_id, accepted=True, changed_wall_time_ns=time.time_ns())
        if action == 'start':
            health = self.source.health.snapshot()
            unhealthy = [k for k,c in health.items() if c['warning']]
            error = ('recording_active' if self.session.writer or self.session.recording_state in ('saving','checking') else
                     'input_unhealthy' if self.options.source=='ros2' and unhealthy and not event.get('force') else None)
            if error:
                self.control_result = dict(request_id=request_id, accepted=False, error_code=error, unhealthy=unhealthy)
                self.emit(True)
                return
            event = dict(event, health_snapshot=health)
        if action == 'start' and self.session.writer is None:
            self.receiver_start = self.boundary()
            self.session.control(event)
            self.session.writer.meta.update(receive_start_sequence=self.receiver_start,
                                             receive_end_sequence=self.receiver_start, received_input_messages=0)
        elif action == 'stop' and self.session.writer:
            count = self.boundary()
            self.session.writer.meta.update(receive_end_sequence=count, received_input_messages=count-self.receiver_start)
            self.session.control(event)
        else:
            self.session.control(event)
        self.emit(True)

    def run(self):
        a = self.options
        success = False
        ticks = 0
        begin = time.monotonic()
        try:
            configure_runtime()
            spec = load_spec(a.spec)
            mapper = ArmMapper({s: getattr(spec, s+'_arm').home_q for s in ('left', 'right')})
            if a.source == 'ros2':
                self.source = Ros2Source(mapper, topic_prefix=a.ros_topic_prefix)
            elif a.source == 'udp':
                self.source = CollectionUdpSource(a.udp_host, a.udp_port, stale_after_s=.25,
                    allow_source=site_value('udp.allow_sources', []) or None, arm_mapper=mapper)
            else:
                self.source = CollectionReplay(a.replay, loop=a.loop_replay)
            self.backend = create_backend(a.backend, spec, work_dir=a.output.with_name(a.output.name+'-physics'),
                render=False, resource_root=a.resource_root, environment_collisions=a.scene_variant=='full')
            self.backend.reset()
            if a.backend == 'mujoco' and a.mujoco_workers > 1:
                self.backend.mj.mju_threadpool(self.backend.data, a.mujoco_workers)
            self.session = Session(a.output, spec, self.backend,
                dict(kind=a.source, replay=None if a.replay is None else str(a.replay), loop=a.loop_replay),
                mapper=mapper, fps=a.rate_hz)
            self.session.meta['native_cpu_workers'] = a.mujoco_workers
            self.session.on_status = lambda: self.emit(True)
            if a.note:
                self.control(dict(action='note', text=a.note))
            if a.record:
                self.control(dict(action='start'))
            if a.view:
                from .collection_stream import OperatorStream
                self.view = OperatorStream(a.backend, a.spec, a.resource_root, a.output/'view', a.host, a.port,
                                           environment_collisions=a.scene_variant=='full')
                self.emit(True)
                self.view.wait_ready(self.stop)
            self.state = 'running'
            begin = time.monotonic()
            n = None if a.duration is None else max(1, round(a.duration*a.rate_hz))
            while (n is None or ticks < n) and not (self.stop and self.stop.is_set()):
                if a.realtime:
                    delay = max(0., begin+ticks/a.rate_hz-time.monotonic())
                    if self.stop:
                        if self.stop.wait(delay):
                            break
                    else:
                        time.sleep(delay)
                self.session.record_inputs(self.source.drain_events())
                while True:
                    try:
                        self.control(self.controls.get_nowait())
                    except queue.Empty:
                        break
                if self.view:
                    for event in self.view.events():
                        self.control(event)
                commands = self.source.poll(time.monotonic() if a.source!='replay' else ticks/a.rate_hz)
                self.backend.apply(commands)
                applied_ns = time.monotonic_ns()
                self.backend.step(1/a.rate_hz)
                self.last_row = self.session.record(self.backend.observe(), commands,
                    applied_monotonic_ns=applied_ns, input_sequences=getattr(self.source, 'polled_sequences', None))
                if self.view:
                    self.view.publish(self.status())
                ticks += 1
                self.emit()
            success = True
        except KeyboardInterrupt:
            success = False  # CLI retains interrupted status on Ctrl+C
        except Exception as exc:
            self.error = str(exc)
            raise
        finally:
            try:
                if self.source:
                    self.source.close()
                if self.session:
                    count = self.boundary()
                    if self.session.writer:
                        self.session.writer.meta.update(receive_end_sequence=count, received_input_messages=count-self.receiver_start)
                    self.session.close('complete' if success else 'interrupted')
            finally:
                try:
                    if self.view:
                        closing_view, self.view = self.view, None
                        closing_view.close(cancelled=bool(self.stop and self.stop.is_set()))
                finally:
                    if self.backend:
                        self.backend.close()
                    self.state = 'stopped' if success or not self.error else 'failed'
                    self.emit(True)
        result = dict(ticks=ticks, sim_time_s=self.last_row.get('sim_time_s', 0),
                      wall_elapsed_s=time.monotonic()-begin, decode_errors=self.source.decode_errors,
                      source_cycles=getattr(self.source, 'cycles', 0))
        (a.output/'result.json').write_text(json.dumps(result, indent=2)+'\n')
        return result


def run_managed(options, controls, stop, statuses):
    """Spawn target: all simulator and ROS ownership remains in this child."""
    def publish(state):
        # Health/status is latest-only; command journals never use this queue.
        try:
            statuses.put_nowait(state)
        except queue.Full:
            try:
                statuses.get_nowait()
                statuses.put_nowait(state)
            except (queue.Empty, queue.Full):
                pass
    try:
        controller = CollectionController(options, controls=controls, stop=stop, publish=publish)
        if os.environ.get('GAMEPAD_VLA_COLLECTION_PROFILE') == '1':
            import cProfile
            profiler = cProfile.Profile()
            try:profiler.runcall(controller.run)
            finally:
                if options.output.exists():profiler.dump_stats(str(options.output/'controller-profile.pstats'))
        else:
            controller.run()
    except Exception as exc:
        publish(dict(session_state='failed', error=str(exc), recording_status=dict(state='failed')))
        raise
