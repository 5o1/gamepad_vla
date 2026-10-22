"""Loopback-only aiohttp collection desk; owns only its simulation children."""
import asyncio
import json
import multiprocessing as mp
from pathlib import Path
import queue
import shutil
import secrets
from collections import deque
import socket
import sys
import time
from uuid import uuid4

from gamepad_vla.site import site_value, workspace_root, repository_root
from .collection_controller import CollectionOptions, run_managed
from .collection_health import detect_safety, require_loopback, DISK_MIN_BYTES, SAFETY_INTERVAL_S
from .collection_integrity import discard_episode, read_integrity


def defaults():
    return dict(backend=site_value('collection.backend'), source=site_value('collection.source'),
                scene_variant=site_value('collection.scene_variant'), duration=site_value('collection.duration'),
                rate_hz=site_value('collection.rate_hz'), replay=site_value('collection.replay'),
                loop_replay=site_value('collection.loop_replay'),
                ros_topic_prefix=site_value('collection.ros_topic_prefix', ''))


class SessionManager:
    def __init__(self, root=None, default_values=None):
        self.root = Path(root or site_value('collection.output_root') or workspace_root()/'runs/collection-web').expanduser().resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self.ctx = mp.get_context('spawn')
        self.worker = self.controls = self.stop_event = self.statuses = None
        self.latest = dict(session_state='idle', recording_status=dict(state='idle'))
        self.port = None
        self.output = None
        self.lock = asyncio.Lock()
        self.export_process = None
        self.pending_start = False
        self.active_options = {}
        self.default_values = default_values or {}

    def refresh(self):
        if self.statuses:
            while True:
                try:
                    self.latest = self.statuses.get_nowait()
                except queue.Empty:
                    break
        if self.worker and not self.worker.is_alive() and self.latest.get('session_state') not in ('stopped','failed'):
            self.latest = dict(self.latest, session_state='failed', error=f'session worker exited: {self.worker.exitcode}')
        if self.latest.get('recording_status',{}).get('state') == 'recording':
            self.pending_start = False
        if self.worker and self.worker.is_alive() and self.latest.get('session_state') == 'stopped':
            return dict(self.latest, session_state='stopping')
        return self.latest

    def episode(self, identifier):
        path = (self.root/identifier.replace('~','/')).resolve()
        if not path.is_relative_to(self.root) or not (path/'meta.json').is_file():
            raise ValueError('episode not found within collection root')
        return path

    def episodes(self):
        items = []
        for path in sorted(self.root.glob('session-*/episode-*/meta.json'), reverse=True):
            try:
                meta = json.loads(path.read_text())
                check = path.with_name('integrity.json')
                integrity = read_integrity(path.parent)
                current=self.latest.get('recording_status',{})
                live=(meta.get('status')=='recording' and self.output==path.parent.parent and current.get('episode')==path.parent.name)
                duration=(meta['recording_end_sim_time_s']-meta['recording_start_sim_time_s']
                          if 'recording_end_sim_time_s' in meta else None)
                items.append(dict(id=str(path.parent.relative_to(self.root)).replace('/','~'), episode=path.parent.name,
                    created_wall_time_ns=meta.get('created_wall_time_ns'), rows=current.get('rows') if live else meta.get('rows') if meta.get('status')!='recording' else None,
                    duration_s=current.get('duration_s') if live else duration,
                    notes=current.get('notes',[]) if live else meta.get('operator_notes',[]), status=meta.get('status'), integrity=integrity,
                    discarded=path.with_name('discarded.json').exists()))
            except (OSError, ValueError) as exc:
                items.append(dict(id=str(path.parent.relative_to(self.root)).replace('/','~'), status='failed', error=str(exc)))
        return items

    async def start(self, values):
        async with self.lock:
            if self.worker and self.worker.is_alive():
                raise ValueError('only one simulation session is allowed; stop it first')
            v = dict(defaults(), **self.default_values)
            v.update(values)
            replay = v.get('replay')
            if v.get('episode'):
                path = self.episode(v['episode'])
                meta = json.loads((path/'meta.json').read_text())
                v.update(source='replay', replay=str(path), backend=meta['backend_name'],
                    scene_variant=meta['scene_variant']['name'], rate_hz=int(meta['fps']),
                    duration=meta['rows']/float(meta['fps']), loop_replay=False)
                replay = v['replay']
            stamp = time.strftime('%Y%m%d-%H%M%S')
            output = self.root/f'session-{stamp}-{uuid4().hex[:6]}'
            with socket.socket() as sock:
                sock.bind(('127.0.0.1',0))
                port = sock.getsockname()[1]
            options = CollectionOptions(backend=v['backend'], source=v['source'], output=output,
                duration=None if v['duration'] is None else float(v['duration']),
                rate_hz=int(v['rate_hz']), scene_variant=v['scene_variant'],
                replay=Path(replay).expanduser().resolve() if replay else None,
                ros_topic_prefix=v.get('ros_topic_prefix',''), loop_replay=bool(v.get('loop_replay')),
                spec=Path(v.get('spec') or site_value('collection.spec') or repository_root()/'configs/workcell/station_g20_v4_4.json'),
                arm_startup=v.get('arm_startup') or site_value('collection.arm_startup','spec'),
                resource_root=Path(site_value('collection.resource_root')) if site_value('collection.resource_root') else None,
                view=True, realtime=True, host='127.0.0.1', port=port,
                udp_host=site_value('udp.bind_host','127.0.0.1'), udp_port=site_value('udp.port',50200))
            options.validate()
            if options.replay and not options.replay.exists():
                raise ValueError('replay file does not exist; select a file in station configuration')
            self.controls = self.ctx.Queue(maxsize=32)
            self.statuses = self.ctx.Queue(maxsize=8)
            self.stop_event = self.ctx.Event()
            self.latest = dict(session_state='starting', recording_status=dict(state='idle'))
            self.port, self.output = port, output
            self.active_options = dict(backend=options.backend, source=options.source, scene_variant=options.scene_variant)
            self.pending_start = False
            self.worker = self.ctx.Process(target=run_managed, args=(options,self.controls,self.stop_event,self.statuses))
            self.worker.start()
            return dict(started=True)

    async def stop(self, client="service"):
        if self.worker and self.worker.is_alive() and self.refresh().get("recording_status",{}).get("state")=="recording":
            await self.command(dict(action="stop",client=client))
            deadline=time.monotonic()+30
            while self.refresh().get("recording_status",{}).get("state") in ("recording","saving","checking"):
                if time.monotonic()>deadline:
                    raise RuntimeError("录制保存超时，请检查采集日志。")
                await asyncio.sleep(.025)
        async with self.lock:
            if self.worker and self.worker.is_alive():
                self.stop_event.set()
                await asyncio.to_thread(self.worker.join, 30)
                if self.worker.is_alive():
                    # No pgrep/pkill or hardware stop. Keep ownership for retry/diagnosis.
                    raise RuntimeError('owned session did not stop in 30 s; inspect its log')
            self.refresh()
            return dict(stopped=True, state=self.latest)

    async def command(self, event):
        event = dict(event, request_id=uuid4().hex)
        self.control(event)
        deadline=time.monotonic()+10
        while time.monotonic()<deadline:
            result=self.refresh().get('control_result') or {}
            if result.get('request_id')==event['request_id']:
                return result
            if not self.worker.is_alive():
                raise RuntimeError('会话已经退出，请检查日志后重新启动。')
            await asyncio.sleep(.01)
        raise RuntimeError('控制命令等待确认超时，请查询状态后再操作。')

    def control(self, event):
        if event.get('action') not in ('start','stop','note','view'):
            raise ValueError('expected start/stop/note/view')
        if not self.worker or not self.worker.is_alive():
            raise ValueError('start a simulation session first')
        self.controls.put_nowait(event)
        return dict(queued=True)

    async def export(self, identifier):
        path = self.episode(identifier)
        if json.loads((path/'meta.json').read_text())['status'] != 'complete':
            raise ValueError('cannot export an incomplete episode')
        executable = Path(site_value('collection.export_python') or workspace_root()/'envs/data-tools-py310/bin/python').expanduser()
        if not executable.is_file():
            raise ValueError('LeRobot export environment unavailable. Set collection.export_python to the existing data-tools Python; no packages were installed.')
        if self.export_process and self.export_process.returncode is None:
            raise ValueError('an export is already running')
        target = path.parent/(path.name+'-lerobot')
        self.export_process = await asyncio.create_subprocess_exec(str(executable),
            str(repository_root()/'scripts/export_lerobot.py'), str(path), '--output', str(target),
            stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)
        output, _ = await self.export_process.communicate()
        if self.export_process.returncode:
            raise ValueError('LeRobot export failed (check data-tools environment): '+output.decode(errors='replace')[-2000:])
        return dict(exported=True, output=str(target.relative_to(self.root)))


class ApiError(Exception):
    def __init__(self, status, code, message):
        self.status,self.code,self.message=status,code,message


def make_app(manager=None, *, enable_remote_control=False, video_handler=None, frame_provider=None, managed=True):
    from aiohttp import web, ClientSession, ClientTimeout, ClientError, WSMsgType
    manager=manager or SessionManager()
    root=Path(__file__).with_name('collection_web')
    cookie=secrets.token_urlsafe(32)
    remote_enabled=bool(enable_remote_control)
    safety={}
    stream=None
    frame_row={}
    client=None
    recent=deque(maxlen=10)
    revision=0
    audit=(manager.root/'api_audit.jsonl').open('a',buffering=1)
    recording_lock=asyncio.Lock()

    def snapshot():
        current=dict(manager.refresh(), **manager.active_options)
        if frame_provider:
            current.update(frame_provider())
        if frame_row:
            for key in ('overlay','gamepad_signal','gamepad_mask','sim_time_s','arm_clamps','view'):
                if key in frame_row:current[key]=frame_row[key]
        free=shutil.disk_usage(manager.root).free
        status=current.get('recording_status',{})
        saved=status.get('last_saved') or {}
        current.update(server_wall_time_ns=time.time_ns(), safety=safety, stream=current.get('stream') if frame_provider else stream, remote_control_enabled=remote_enabled,
            disk=dict(free_bytes=free,warning=free<DISK_MIN_BYTES),
            integrity=status.get('integrity') or dict(episode_id=None,state='pending',phase='idle',passed=None), recent_control_events=list(recent), revision=revision,
            session_id=manager.output.name if manager.output else None,
            episode_id=(manager.output.name+'~'+status['episode']) if manager.output and status.get('episode') else None)
        return current

    def event(endpoint, who, values):
        nonlocal revision
        revision+=1
        recent.append(dict(wall_time_ns=time.time_ns(),operation=endpoint,client=who,parameters=values))

    def response(value=None):
        state=snapshot()
        return web.json_response(dict(value or {},state=state,safety=state['safety']))

    @web.middleware
    async def contract(request,handler):
        is_api=request.path.startswith('/api/v1/')
        values=dict(request.query)
        who=request.headers.get('X-Gamepad-VLA-Client','').strip()
        result=None
        try:
            if request.method=='POST':
                raw=await request.read()
                try:values=json.loads(raw) if raw else {}
                except ValueError:values={'raw_body':raw.decode(errors='replace')}
            origin=request.headers.get('Origin')
            if origin and origin!=f'{request.scheme}://{request.host}':
                raise ApiError(403,'cross_origin','请求来源不属于本机页面，请在采集台本机调用接口。')
            if request.method=='POST':
                if request.content_type!='application/json' or not who:
                    raise ApiError(403,'client_header_required','写操作需要 JSON 和客户端标识，请使用 collection_ctl.py 或本机网页。')
                try:values=await request.json()
                except (ValueError,TypeError):raise ApiError(400,'invalid_json','JSON 格式错误，请修正参数后重试。')
                if not isinstance(values,dict):raise ApiError(400,'invalid_json','参数必须为 JSON 对象，请修正参数后重试。')
                request['values']=values
                request['client']=who
                is_web=secrets.compare_digest(request.cookies.get('collection_web_session',''),cookie)
                request['is_web']=is_web
                if request.path=='/api/v1/remote-control' and not is_web:
                    raise ApiError(403,'web_session_required','只有现场网页能切换远程控制，请让现场人员操作网页。')
                if not is_web and not remote_enabled:
                    raise ApiError(403,'remote_control_disabled','请现场人员在网页上开启远程控制。')
            result=await handler(request)
        except ApiError as exc:
            result=web.json_response(dict(error_code=exc.code,message_zh=exc.message,state=snapshot(),safety=safety),status=exc.status)
        except (ValueError,KeyError,TypeError,queue.Full) as exc:
            result=web.json_response(dict(error_code='invalid_request',message_zh=f'参数或数据无效，请核对后重试：{exc}',state=snapshot(),safety=safety),status=400)
        except RuntimeError as exc:
            result=web.json_response(dict(error_code='operation_failed',message_zh=f'操作失败，请查看服务日志后重试：{exc}',state=snapshot(),safety=safety),status=503)
        except web.HTTPException as exc:
            result=web.json_response(dict(error_code='not_found' if exc.status==404 else 'http_error',message_zh='端点或方法不存在，请按 API v1 文档核对请求。',state=snapshot(),safety=safety),status=exc.status)
        if is_api and request.path!='/api/v1/events':
            body=json.loads(result.text) if isinstance(result,web.Response) and result.content_type=='application/json' else {}
            audit.write(json.dumps(dict(wall_time_ns=time.time_ns(),endpoint=request.path,method=request.method,
                parameters=values,client=who or 'unspecified',result=dict(http_status=result.status,
                    error_code=body.get('error_code') if isinstance(body,dict) else None,
                    session_state=manager.refresh().get('session_state'),episode_id=snapshot()['episode_id'])),ensure_ascii=False)+'\n')
        return result

    async def index(request):
        result=web.FileResponse(root/'index.html')
        result.set_cookie('collection_web_session',cookie,httponly=True,samesite='Strict')
        return result

    async def config(request):
        files=site_value('collection.replay_files',[])
        default=dict(defaults(),**getattr(manager,"default_values",{}))
        if default['replay'] and default['replay'] not in files:files=[default['replay'],*files]
        return response(dict(managed=managed,defaults=default,replay_files=files,
            backends=['sapien','mujoco'],scene_variants=['full','free-space','gamepad-contact'],sources=['ros2','replay','udp'],
            estop_hint='~/gamepad_vla_estop/estop.sh（不能下电）'))

    async def status(request):
        return web.json_response(snapshot())

    async def start(request):
        if manager.worker and manager.worker.is_alive():
            raise ApiError(409,'session_active','仿真会话已经运行，请先停止当前会话。')
        v=dict(request['values'])
        if 'duration_s' in v:v['duration']=v.pop('duration_s')
        await manager.start(v)
        event('session/start',request['client'],v)
        return response()

    async def stop(request):
        await manager.stop(request['client'])
        event('session/stop',request['client'],{})
        return response()

    def running():
        s=manager.refresh()
        if not manager.worker or not manager.worker.is_alive() or s.get('session_state')!='running':
            raise ApiError(409,'session_not_running','仿真会话尚未就绪，请先启动会话并等待就绪。')
        return s

    async def recording_start(request):
        async with recording_lock:
            s=running()
            if manager.pending_start or s.get('recording_status',{}).get('state') in ('recording','saving','checking'):
                raise ApiError(409,'recording_active','录制已经开始或正在保存，请先查询当前录制状态。')
            v=request['values']
            if type(v.get('force',False)) is not bool:
                raise ApiError(400,'invalid_request','force 必须是布尔值，请修正后重试。')
            result=await manager.command(dict(action='start',client=request['client'],force=v.get('force',False)))
            if not result['accepted']:
                code=result['error_code']
                raise ApiError(409,code,'输入不健康：'+', '.join(result.get('unhealthy',[]))+'；请恢复输入后重试，必要时显式 force。')
            if v.get('note'):
                await manager.command(dict(action='note',client=request['client'],text=str(v['note'])))
            event('recording/start',request['client'],v)
            return response(dict(episode_id=snapshot()['episode_id']))

    async def recording_stop(request):
        async with recording_lock:
            s=manager.refresh()
            episode_id=snapshot()['episode_id']
            if manager.worker and manager.worker.is_alive() and s.get('recording_status',{}).get('state')=='recording':
                # Returns immediately; saving/checking runs in the controller process.
                manager.control(dict(action='stop',client=request['client']))
            event('recording/stop',request['client'],{})
            return response(dict(episode_id=episode_id))

    async def notes(request):
        running()
        text=request['values'].get('text')
        if not isinstance(text,str):raise ApiError(400,'invalid_request','备注 text 必须是文字，请修正后重试。')
        await manager.command(dict(action='note',client=request['client'],text=text))
        event('notes',request['client'],dict(text=text))
        return response()

    async def episodes(request):
        return web.json_response(await asyncio.to_thread(manager.episodes))

    async def episode(request):
        path=manager.episode(request.match_info['id'])
        meta=json.loads((path/'meta.json').read_text())
        check=path/'integrity.json'
        return response(dict(id=request.match_info['id'],meta=meta,
            integrity=read_integrity(path),voided=(path/'discarded.json').exists()))

    async def void(request):
        path=manager.episode(request.match_info['id'])
        if json.loads((path/'meta.json').read_text())['status']=='recording':
            raise ApiError(409,'episode_recording','该段仍在录制，请停止并保存后再作废。')
        value=discard_episode(path,request['values'].get('reason',''))
        event('episodes/void',request['client'],dict(id=request.match_info['id']))
        return response(value)

    async def export(request):
        identifier=request.match_info['id']
        event('episodes/export',request['client'],dict(id=identifier))
        return response(await manager.export(identifier))

    async def remote(request):
        nonlocal remote_enabled
        value=request['values'].get('enabled')
        if type(value) is not bool:raise ApiError(400,'invalid_request','enabled 必须是布尔值，请修正后重试。')
        remote_enabled=value
        event('remote-control',request['client'],dict(enabled=value))
        return response()

    async def events(request):
        result=web.StreamResponse(headers={'Content-Type':'text/event-stream','Cache-Control':'no-cache','X-Accel-Buffering':'no'})
        audit.write(json.dumps(dict(wall_time_ns=time.time_ns(),endpoint=request.path,method='GET',parameters={},
                                   client=request.headers.get('X-Gamepad-VLA-Client','web'),result=dict(http_status=200)))+'\n')
        await result.prepare(request)
        try:
            while True:
                data=json.dumps(snapshot(),ensure_ascii=False,separators=(',',':'))
                await asyncio.wait_for(result.write(('data: '+data+'\n\n').encode()),2)
                await asyncio.sleep(.05)
        except (ConnectionError,asyncio.TimeoutError,asyncio.CancelledError):pass
        return result

    async def video(request):
        if not manager.worker or not manager.worker.is_alive():raise web.HTTPServiceUnavailable(text='no active simulation')
        try:
            upstream=await client.ws_connect(f'http://127.0.0.1:{manager.port}/video')
        except (ClientError,OSError,asyncio.TimeoutError):return web.Response(status=503,text='stream unavailable')
        result=web.WebSocketResponse()
        await result.prepare(request)
        async def forward():
            async for message in upstream:
                if message.type==WSMsgType.BINARY:await result.send_bytes(message.data)
        async def closed():
            async for _ in result:pass
        tasks=[asyncio.create_task(forward()),asyncio.create_task(closed())]
        try:await asyncio.wait(tasks,return_when=asyncio.FIRST_COMPLETED)
        finally:
            for task in tasks:task.cancel()
            await asyncio.gather(*tasks,return_exceptions=True)
            await upstream.close();await result.close()
        return result

    async def view(request):
        running()
        value=request['values'].get('view')
        if value not in ('camera_3','operator'):raise ApiError(400,'invalid_request','视角必须是 camera_3 或 operator，请重新选择。')
        await manager.command(dict(action='view',view=value,client=request['client']))
        event('view',request['client'],dict(view=value))
        return response()

    async def inspect():
        nonlocal safety
        while True:
            safety=await asyncio.to_thread(detect_safety)
            manager.refresh()
            await asyncio.sleep(SAFETY_INTERVAL_S)

    async def measure_stream():
        nonlocal stream,frame_row
        while True:
            if manager.worker and manager.worker.is_alive() and manager.port:
                try:
                    async with client.get(f'http://127.0.0.1:{manager.port}/state',timeout=ClientTimeout(total=2)) as r:
                        row=await r.json()
                        stream=row.get('stream');frame_row=row
                except (ClientError,OSError,asyncio.TimeoutError):stream=None
            else:stream=None;frame_row={}
            await asyncio.sleep(.05)

    async def lifecycle(app):
        nonlocal client
        client=ClientSession()
        tasks=[asyncio.create_task(inspect())]
        if not frame_provider:tasks.append(asyncio.create_task(measure_stream()))
        yield
        for task in tasks:task.cancel()
        await asyncio.gather(*tasks,return_exceptions=True)
        if managed:await manager.stop()
        if manager.export_process and manager.export_process.returncode is None:
            await manager.export_process.wait()
        await client.close();audit.close()

    app=web.Application(middlewares=[contract])
    app.cleanup_ctx.append(lifecycle)
    app.router.add_get('/',index)
    app.router.add_get('/video',video_handler or video)
    app.router.add_static('/overlay',root/'overlay')
    app.router.add_static('/web',root.parent/'operator_web')
    for path,handler in [('config',config),('status',status),('events',events),('episodes',episodes),('episodes/{id}',episode)]:
        app.router.add_get('/api/v1/'+path,handler)
    for path,handler in [('session/start',start),('session/stop',stop),('recording/start',recording_start),('recording/stop',recording_stop),
                         ('notes',notes),('episodes/{id}/void',void),('episodes/{id}/export',export),('remote-control',remote),('view',view)]:
        app.router.add_post('/api/v1/'+path,handler)
    return app


def main():
    import argparse
    from aiohttp import web
    from gamepad_vla.site import configure_runtime
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host',default=site_value('operator.bind_host','127.0.0.1'))
    parser.add_argument('--port',type=int,default=site_value('operator.port',8780))
    parser.add_argument('--enable-remote-control',action='store_true')
    parser.add_argument('--source',choices=('ros2','udp','replay'))
    parser.add_argument('--spec',type=Path)
    parser.add_argument('--scene-variant',choices=('full','free-space','gamepad-contact'))
    parser.add_argument('--arm-startup',choices=('spec','first_command','none'))
    parser.add_argument('--output-root',type=Path)
    parser.add_argument('--ros-topic-prefix')
    args=parser.parse_args();require_loopback(args.host);configure_runtime()
    values={k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items() if k in ('source','spec','scene_variant','arm_startup','ros_topic_prefix') and v is not None}
    manager=SessionManager(args.output_root,default_values=values)
    web.run_app(make_app(manager,enable_remote_control=args.enable_remote_control),host=args.host,port=args.port,shutdown_timeout=2)
