"""Real aiohttp permission/audit/SSE flow with a fake completed-state source."""
import asyncio
import json
from pathlib import Path
from types import SimpleNamespace
import time
import pytest
pytest.importorskip('aiohttp')
from aiohttp import ClientSession, CookieJar
from aiohttp.test_utils import TestServer
from gamepad_vla.workstation.collection_server import SessionManager, make_app
from gamepad_vla.workstation.collection_snapshots import SceneSnapshots


def test_snapshot_permissions_shared_countdown_and_download(tmp_path):
    async def scenario():
        manager=SessionManager(tmp_path/'sessions');manager.output=manager.root/'session-test'
        manager.view_run=manager.output/'view';manager.view_run.mkdir(parents=True)
        (manager.view_run/'snapshot.mmap').touch()
        manager.worker=SimpleNamespace(is_alive=lambda:True)
        manager.refresh=lambda:dict(session_state='running',recording_status=dict(state='recording',episode='unchanged'))
        captures=[];started=asyncio.Event();release=asyncio.Event()
        def reader(run):return dict(applied_ns=time.monotonic_ns(),arm_q=[[0]*7]*2,hand_q=[[0]*21]*2,step=7)
        async def capture(run,folder,identifier):
            captures.append(identifier);started.set();await release.wait()
            for name in ('camera_3','aux_face','aux_front','aux_back','aux_left','aux_right','aux_overview'):
                (folder/(name+'.png')).write_bytes(b'fake PNG')
            return dict(camera_count=7)
        service=SceneSnapshots(tmp_path/'snapshots',reader=reader,capture=capture)
        app=make_app(manager,managed=False,snapshots=service,camera=SimpleNamespace(stop=lambda:None))
        server=TestServer(app);await server.start_server();url=str(server.make_url('')).rstrip('/')
        headers={'X-Gamepad-VLA-Client':'test','Content-Type':'application/json'}
        async with ClientSession() as remote,ClientSession(cookie_jar=CookieJar(unsafe=True)) as a,ClientSession(cookie_jar=CookieJar(unsafe=True)) as b:
            async def post(client,path,values):
                async with client.post(url+'/api/v1/'+path,json=values,headers=headers) as response:
                    return response.status,await response.json()
            assert (await post(remote,'snapshot',dict(delay_s=30)))[0]==403
            await a.get(url+'/');await b.get(url+'/')
            code,result=await post(a,'snapshot',dict(delay_s=30,label='reference'))
            assert code==200 and result['snapshot']['state']=='countdown'
            assert (await post(b,'snapshot',dict(delay_s=0)))[0]==409
            async with a.get(url+'/api/v1/events') as sa,b.get(url+'/api/v1/events') as sb:
                states=[]
                for stream in (sa,sb):
                    line=await asyncio.wait_for(stream.content.readline(),2)
                    states.append(json.loads(line.decode().removeprefix('data: ')))
                assert states[0]['scene_snapshot']['id']==states[1]['scene_snapshot']['id']
                assert states[0]['scene_snapshot']['deadline_wall_time_ns']==states[1]['scene_snapshot']['deadline_wall_time_ns']
            assert (await post(b,'snapshot/cancel',{}))[1]['snapshot']['state']=='cancelled'
            assert not captures
            # Existing remote-control switch governs these writes, too.
            assert (await post(a,'remote-control',dict(enabled=True)))[0]==200
            assert (await post(remote,'snapshot',dict(delay_s=0,label='all views')))[0]==200
            await asyncio.wait_for(started.wait(),2)
            assert (await post(a,'snapshot/cancel',{}))[0]==409
            release.set();await service.task
            assert service.status()['state']=='saved'
            audit=manager.root/'api_audit.jsonl';size=audit.stat().st_size
            async with remote.get(url+'/api/v1/snapshots') as response:
                rows=await response.json();assert len(rows)==1
            name=rows[0]['name']
            async with remote.get(url+'/api/v1/snapshots/'+name) as response:
                data=await response.json();assert len([f for f in data['files'] if f.endswith('.png')])==7
            async with remote.get(url+'/api/v1/snapshots/'+name+'/camera_3.png') as response:
                assert response.status==200 and await response.read()==b'fake PNG'
            assert audit.stat().st_size==size,'Successful reads must not append production logs'
            assert manager.refresh()['recording_status']==dict(state='recording',episode='unchanged')
            async with remote.post(url+'/api/v1/snapshot',json={'delay_s':0},headers=dict(headers,Origin='http://wrong.example')) as response:
                assert response.status==403
        await server.close()
    asyncio.run(scenario())


def test_auxiliary_websocket_proxy_forwards_view_demand(tmp_path):
    async def scenario():
        from aiohttp import web,WSMsgType
        async def upstream_video(request):
            ws=web.WebSocketResponse();await ws.prepare(request)
            async for message in ws:
                if message.type==WSMsgType.TEXT:await ws.send_bytes(message.data.encode())
            return ws
        upstream_app=web.Application();upstream_app.router.add_get('/aux-video',upstream_video)
        upstream=TestServer(upstream_app);await upstream.start_server()
        manager=SessionManager(tmp_path);manager.worker=SimpleNamespace(is_alive=lambda:True)
        manager.refresh=lambda:dict(session_state='running');manager.port=upstream.port
        app=make_app(manager,managed=False,camera=SimpleNamespace(stop=lambda:None),snapshots=SceneSnapshots(tmp_path/'snapshots'))
        outer=TestServer(app);await outer.start_server()
        try:
            async with ClientSession() as client:
                async with client.ws_connect(str(outer.make_url('/aux-video'))) as ws:
                    await ws.send_json(dict(views=['face','overview']))
                    message=await ws.receive(timeout=2)
                    assert message.type==WSMsgType.BINARY
                    assert json.loads(message.data)==dict(views=['face','overview'])
                async with client.get(str(outer.make_url('/auxiliary-panel.mjs'))) as response:
                    assert response.status==200 and 'auxiliaryPanel' in await response.text()
        finally:
            await outer.close();await upstream.close()
    asyncio.run(scenario())
