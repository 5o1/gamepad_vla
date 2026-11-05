"""One teleop mmap -> NVENC -> WebSocket channel; used for both live streams."""
import asyncio
from collections import deque
import json
import time

from .runtime_logs import diagnostic_file, CappedProcessLog


class OperatorStream:
    def __init__(self, run, cfg, *, auxiliary=False):
        from xbox_gamepad_physics.teleop.stream.ipc import Latest
        from .auxiliary_views import auxiliary_options, frame_capacity
        options = auxiliary_options(cfg)
        self.run, self.cfg, self.auxiliary = run, cfg, auxiliary
        self.name = 'aux' if auxiliary else 'main'
        self.width = options['width'] if auxiliary else cfg['width']
        self.height = options['height'] if auxiliary else cfg['height']
        self.fps = options['fps'] if auxiliary else cfg['fps']
        self.frames = Latest(run/('aux-frame.mmap' if auxiliary else 'frame.mmap'), frame_capacity(self.width,self.height))
        self.demand = Latest(run/'aux-demand.mmap', 4096) if auxiliary else None
        self.peers = {}
        self.fifo = asyncio.Queue(maxsize=3)
        self.encoded_count = 0
        self.codec = None
        self.recent = deque(maxlen=600)
        self.latest = {}
        self.encoder = None
        self.encoder_ready = asyncio.Event()
        self.encoder_log = CappedProcessLog(run/('aux-encoder.log' if auxiliary else 'encoder.log'))
        self.timing = diagnostic_file(run, 'aux_stream_timing.jsonl' if auxiliary else 'stream_timing.jsonl', cfg)
        self.recording = diagnostic_file(run, 'auxiliary.h264' if auxiliary else 'operator.h264', cfg)
        self.tasks = []
        if self.demand:
            self._update_demand()

    def _update_demand(self):
        from .auxiliary_views import selected_views
        names = selected_views(list({v for p in self.peers.values() for v in p['views']}))
        self.demand.write(json.dumps(names).encode())

    def encoder_args(self):
        size = self.cfg.get('auxiliary' if self.auxiliary else 'collection_main', {})
        bitrate = size.get('bitrate', '16M' if self.width*self.height > 1920*1080 else '8M' if self.auxiliary else '12M' if self.width>=1920 else '6M')
        level = '5.1' if self.width*self.height > 1920*1080 else '4.2'
        return ['ffmpeg','-hide_banner','-loglevel','warning','-threads','2','-filter_threads','2',
            '-f','rawvideo','-pixel_format','rgb24','-video_size',f'{self.width}x{self.height}',
            '-framerate',str(self.fps),'-i','pipe:0','-an','-c:v','h264_nvenc','-preset','p1',
            '-tune','ull','-profile:v','main','-level:v',level,'-gpu','0','-bf','0','-rc-lookahead','0',
            '-zerolatency','1','-delay','0','-g',str(max(1,self.fps//2)),'-b:v',bitrate,'-maxrate',bitrate,
            '-bufsize','300k','-aud','1','-vsync','0','-flush_packets','1','-f','h264','pipe:1']

    async def video(self, request):
        from aiohttp import web, WSMsgType
        from .auxiliary_views import selected_views
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        self.peers[ws] = dict(key=True, views=[])
        try:
            async for message in ws:
                if self.auxiliary and message.type == WSMsgType.TEXT:
                    try:
                        names = selected_views(json.loads(message.data)['views'])
                    except (ValueError, TypeError, KeyError):
                        await ws.close(code=1008, message=b'invalid auxiliary views')
                        break
                    self.peers[ws]['views'] = names
                    self._update_demand()
        finally:
            self.peers.pop(ws, None)
            if self.demand:
                self._update_demand()
        return ws

    async def raw_writer(self):
        from xbox_gamepad_physics.teleop.stream.ipc import unpack
        generation = 0
        while True:
            if self.auxiliary and not any(p['views'] for p in self.peers.values()):
                await asyncio.sleep(.05)
                continue
            data = self.frames.read()
            if data and data[0] != generation:
                generation = data[0]
                metadata, rgb = unpack(data[1])
                if self.encoder is None:
                    self.encoder = await asyncio.create_subprocess_exec(*self.encoder_args(),
                        stdin=asyncio.subprocess.PIPE, stdout=asyncio.subprocess.PIPE, stderr=self.encoder_log)
                    self.encoder_ready.set()
                metadata['encoder_input_ns'] = time.monotonic_ns()
                await self.fifo.put(metadata)
                self.encoder.stdin.write(rgb)
                await self.encoder.stdin.drain()
            else:
                await asyncio.sleep(.001)

    async def encoded_reader(self):
        from xbox_gamepad_physics.teleop.stream.gateway import client_operation
        from xbox_gamepad_physics.teleop.stream.h264 import AccessUnits, info
        from xbox_gamepad_physics.teleop.stream.ipc import packet
        await self.encoder_ready.wait()
        parser = AccessUnits()
        while data := await self.encoder.stdout.read(65536):
            for unit in parser.feed(data):
                metadata = await self.fifo.get()
                key, new_codec = info(unit)
                self.codec = new_codec or self.codec
                end = time.monotonic_ns()
                metadata.update(key=key, codec=self.codec, encoded_ns=end,
                                encode_ns=end-metadata['encoder_input_ns'])
                self.recording.write(unit)
                message = packet(metadata, unit)
                for ws, peer in list(self.peers.items()):
                    if peer['key'] and not key:
                        continue
                    peer['key'] = False
                    try:
                        await client_operation(ws.send_bytes(message), .05)
                    except Exception:
                        self.peers.pop(ws, None)
                        if self.demand:
                            self._update_demand()
                        if ws._req and ws._req.transport:
                            ws._req.transport.abort()
                metadata['sent_ns'] = time.monotonic_ns()
                metadata['clients'] = len(self.peers)
                self.timing.json({k:v for k,v in metadata.items() if k not in (
                    'overlay','signals','gamepad_signal','sim_time_s','arm_clamps','candidate_side')})
                self.encoded_count += 1
                self.recent.append((end, (end-metadata['applied_ns'])/1e6))
                self.latest = metadata
                if self.encoded_count == 1 and not self.auxiliary:
                    (self.run/'first_frame').touch()
        raise RuntimeError(f'{self.name} encoder stopped; see {self.run / ("aux-encoder.log" if self.auxiliary else "encoder.log")}')

    def state(self):
        import numpy as np
        now = time.monotonic_ns()
        samples = [v for v in self.recent if now-v[0] <= 10e9]
        fps = (len(samples)-1)/((now-samples[0][0])/1e9) if len(samples)>1 else 0.
        p50,p95 = map(float,np.percentile([v[1] for v in samples],[50,95])) if samples else (None,None)
        return dict(self.latest, stream=dict(frames=self.encoded_count, fps=fps,
            apply_to_encode_p50_ms=p50, apply_to_encode_p95_ms=p95,
            warning=fps < (15 if self.auxiliary else 55) or p95 is None or p95>50))

    def start(self):
        self.tasks = [asyncio.create_task(self.raw_writer()), asyncio.create_task(self.encoded_reader())]

    async def close(self):
        for task in self.tasks:
            task.cancel()
        await asyncio.gather(*self.tasks, return_exceptions=True)
        for ws in list(self.peers):
            await ws.close()
        if self.encoder:
            self.encoder.stdin.close()
            try:
                await asyncio.wait_for(self.encoder.wait(),3)
            except asyncio.TimeoutError:
                self.encoder.kill()
                await self.encoder.wait()
        if self.demand:
            self.demand.write(b'[]')
            self.demand.close()
        self.frames.close()
        self.encoder_log.close(); self.timing.close(); self.recording.close()
