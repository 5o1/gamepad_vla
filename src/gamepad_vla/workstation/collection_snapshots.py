"""One bounded, cancellable scene snapshot; countdown is shared in SSE state."""
import asyncio
import json
import math
from pathlib import Path
import re
import shutil
import time
from uuid import uuid4


class SnapshotConflict(Exception):
    pass


class SceneSnapshots:
    def __init__(self, root, *, delay_s=30, max_bytes=64*1024*1024,
                 width=1280, height=720, reader=None, capture=None):
        self.root = Path(root).expanduser().resolve()
        self.default_delay = float(delay_s)
        self.max_bytes = int(max_bytes)
        self.width, self.height = int(width), int(height)
        if not math.isfinite(self.default_delay) or not 0 <= self.default_delay <= 3600:
            raise ValueError('snapshot delay must be between 0 and 3600 seconds')
        if self.max_bytes <= 0 or not 1280 <= self.width <= 4096 or not 720 <= self.height <= 2160:
            raise ValueError('snapshot size limit must be positive; resolution must be 1280x720 to 4096x2160')
        self.reader = reader or self.read_state
        self.capture = capture or self.request_capture
        self.task = None
        self.current = dict(state='idle')

    def status(self):
        result = dict(self.current)
        if result['state'] == 'countdown':
            result['remaining_s'] = max(0., (result['deadline_wall_time_ns'] - time.time_ns())/1e9)
        return result

    def start(self, run, *, delay_s=None, label=''):
        if self.task and not self.task.done():
            raise SnapshotConflict('A scene snapshot is already pending or capturing.')
        delay_s = self.default_delay if delay_s is None else delay_s
        if isinstance(delay_s, bool) or not isinstance(delay_s, (int, float)) or not math.isfinite(delay_s) or not 0 <= delay_s <= 3600:
            raise ValueError('delay_s must be between 0 and 3600 seconds')
        if not isinstance(label, str) or len(label) > 120:
            raise ValueError('label must be text of at most 120 characters')
        now = time.time_ns()
        identifier = uuid4().hex
        self.current = dict(state='countdown', id=identifier, label=label,
                            requested_wall_time_ns=now, deadline_wall_time_ns=now+round(delay_s*1e9),
                            delay_s=delay_s, max_bytes=self.max_bytes)
        self.task = asyncio.create_task(self._run(Path(run), identifier, float(delay_s), label))
        return self.status()

    async def cancel(self):
        if self.current['state'] == 'capturing':
            raise SnapshotConflict('Snapshot capture has started; wait for its result.')
        if self.task and not self.task.done():
            self.task.cancel()
            await asyncio.gather(self.task, return_exceptions=True)
            self.current = dict(self.current, state='cancelled', remaining_s=0)
        return self.status()

    @staticmethod
    def read_state(run):
        from xbox_gamepad_physics.teleop.stream.ipc import Latest, SNAPSHOT_CAPACITY
        mmap = Latest(run/'snapshot.mmap', SNAPSHOT_CAPACITY)
        try:
            data = mmap.read()
            if data is None:
                raise RuntimeError('No completed simulation state is available.')
            return json.loads(data[1])
        finally:
            mmap.close()

    async def request_capture(self, run, folder, identifier):
        request = dict(id=identifier, folder=str(folder), width=self.width,
                       height=self.height, max_bytes=self.max_bytes)
        temporary = run/'scene-snapshot-request.tmp'
        temporary.write_text(json.dumps(request))
        temporary.replace(run/'scene-snapshot-request.json')
        deadline = time.monotonic()+180
        result = folder/'render-result.json'
        while not result.exists():
            if (run/'stop').exists():
                raise RuntimeError('Simulation ended before snapshot rendering completed.')
            if time.monotonic() > deadline:
                raise RuntimeError('Snapshot rendering timed out; inspect the auxiliary renderer.')
            await asyncio.sleep(.05)
        data = json.loads(result.read_text())
        if data.get('error'):
            raise RuntimeError(data['error'])
        return data

    async def _run(self, run, identifier, delay_s, label):
        staging = None
        try:
            await asyncio.sleep(delay_s)
            self.current = dict(self.current, state='capturing', remaining_s=0)
            # Read exactly once at the deadline; all seven cameras use this state.
            state = self.reader(run)
            frozen_mono, frozen_wall = time.monotonic_ns(), time.time_ns()
            applied = state.get('applied_ns') or state.get('applied_monotonic_ns')
            if applied is None or frozen_mono-applied > 1_000_000_000:
                raise RuntimeError('Latest completed simulation state is older than one second.')
            self.root.mkdir(parents=True, exist_ok=True)
            date = time.strftime('%Y-%m-%d', time.localtime(frozen_wall/1e9))
            stamp = time.strftime('%H%M%S', time.localtime(frozen_wall/1e9))+f'_{frozen_wall%10**9:09d}'
            slug = re.sub(r'[^\w-]+', '_', label, flags=re.UNICODE).strip('_')[:60]
            name = f'{date}/{stamp}'+('_'+slug if slug else '')
            target = self.root/name
            staging = self.root/date/('.pending-'+identifier)
            staging.mkdir(parents=True, exist_ok=False)
            meta = dict(self.current, state='capturing', frozen_wall_time_ns=frozen_wall,
                        frozen_monotonic_ns=frozen_mono, state_age_s=(frozen_mono-applied)/1e9,
                        storage_name=name, input_source=state.get('source'),
                        mapping_versions=state.get('mapping_versions', {}), arm_mapping=state.get('arm_mapping'),
                        raw_glove=dict(available=False, reason='Current collection source subscribes to command topics only; raw_nodes and ergonomics are not available.'))
            inputs = {}
            for channel, event in state.get('snapshot_inputs', {}).items():
                received = event.get('received_monotonic_ns')
                inputs[channel] = dict(event, topic=state.get('input_topics', {}).get(channel), age_s=(frozen_mono-received)/1e9 if received is not None else None)
            (staging/'state.json').write_text(json.dumps(state, indent=2))
            (staging/'inputs.json').write_text(json.dumps(inputs, indent=2))
            (staging/'meta.json').write_text(json.dumps(meta, indent=2))
            if sum(p.stat().st_size for p in staging.iterdir()) > self.max_bytes:
                raise RuntimeError('Snapshot metadata exceeds the configured size limit.')
            result = await self.capture(run, staging, identifier)
            meta.update(result, state='saved', saved_wall_time_ns=time.time_ns())
            (staging/'meta.json').write_text(json.dumps(meta, indent=2))
            total = sum(p.stat().st_size for p in staging.iterdir())
            if total > self.max_bytes:
                raise RuntimeError('Snapshot exceeds the configured size limit.')
            # Publish only a complete capture. Listings never expose staging folders.
            staging.replace(target)
            staging = None
            self.current = dict(self.current, state='saved', remaining_s=0, name=name,
                                path=str(target), bytes=total, files=sorted(p.name for p in target.iterdir()))
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            self.current = dict(self.current, state='failed', remaining_s=0, error=str(exc))
        finally:
            if staging is not None:
                shutil.rmtree(staging, ignore_errors=True)

    def list(self):
        if not self.root.exists():
            return []
        return [dict(name=str(p.parent.relative_to(self.root)), meta=json.loads(p.read_text()),
                     files=sorted(f.name for f in p.parent.iterdir() if f.is_file()))
                for p in sorted(self.root.glob('*/*/meta.json'), reverse=True)
                if not p.parent.name.startswith('.')]

    def file(self, name):
        parts = Path(name).parts
        if Path(name).is_absolute() or '..' in parts or any(p.startswith('.') for p in parts):
            raise FileNotFoundError(name)
        path = self.root/name
        if not path.is_file():
            raise FileNotFoundError(name)
        return path

    async def close(self):
        if self.task and not self.task.done():
            if self.current['state'] == 'capturing':
                await self.task
            else:
                self.task.cancel()
                await asyncio.gather(self.task, return_exceptions=True)
