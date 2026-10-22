"""API v1 contract tests with the real recorder and a CPU observation source."""
import asyncio
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
pytest.importorskip('aiohttp')
from aiohttp import ClientSession, CookieJar
from aiohttp.test_utils import TestServer

from gamepad_vla.workstation.collection_server import SessionManager, make_app
from gamepad_vla.workstation.collection_controller import CollectionController, CollectionOptions
from gamepad_vla.workstation.collection_health import InputHealth
from gamepad_vla.workstation.collection import Session
from test_collection import observation


class Desk(SessionManager):
    def __init__(self,path):
        super().__init__(path)
        self.alive=False
        self.worker=SimpleNamespace(is_alive=lambda:self.alive)
        self.controller=None
    def refresh(self):
        return self.controller.status() if self.controller else self.latest
    async def start(self,v):
        self.output=self.root/'session-test';self.output.mkdir(exist_ok=True)
        options=CollectionOptions(v.get('backend','sapien'),v.get('source','replay'),self.output,None)
        self.controller=CollectionController(options)
        s=Session.__new__(Session)
        s.output=self.output;s.meta=dict(scene_spec_text='{}',fps=50,operator_notes=[])
        s.writer=None;s.last_sim_time_s=0.;s.episode_index=0;s.pending_notes=[]
        s.control_log=(self.output/'events.jsonl').open('a');s.recording_state='idle';s.last_saved=None;s.on_status=lambda:None
        s.commands=dict.fromkeys(('left_arm','right_arm','left_hand','right_hand'));s.input_sequences={}
        self.controller.session=s;self.controller.source=SimpleNamespace(boundary=lambda:([],7),health=InputHealth())
        self.controller.state='running';self.alive=True
    async def stop(self,client='service'):
        if self.alive:
            if self.controller.session.writer:await self.command(dict(action='stop',client=client))
            self.controller.session.close();self.controller.state='stopped';self.alive=False
        return {}
    async def command(self,event):
        self.controller.control(event)
        if event['action']=='start' and self.controller.session.writer:
            s=self.controller.session
            for _ in range(2):s.record(observation(s.last_sim_time_s+.02),[])
        return self.controller.control_result
    def control(self,event):
        self.controller.control(event)
    async def export(self,identifier):
        self.episode(identifier)
        return dict(exported=True,output='export-fixture')


def test_all_api_contracts_and_audit_force(tmp_path):
    async def scenario():
        m=Desk(tmp_path);server=TestServer(make_app(m));await server.start_server()
        url=str(server.make_url('')).rstrip('/')
        headers={'X-Gamepad-VLA-Client':'cli','Content-Type':'application/json'}
        async with ClientSession(cookie_jar=CookieJar(unsafe=True)) as browser,ClientSession() as remote:
            async def call(path,data=None,*,session=remote,h=headers):
                async with session.request('GET' if data is None else 'POST',url+'/api/v1/'+path,json=data,headers=h) as r:
                    return r.status,await r.json()
            assert (await call('status'))[1]['remote_control_enabled'] is False
            for path,data in [('session/start',{}),('session/stop',{}),('recording/start',{}),('recording/stop',{}),('notes',{'text':'n'}),('episodes/missing/void',{}),('episodes/missing/export',{})]:
                code,r=await call(path,data);assert code==403 and r['error_code']=='remote_control_disabled'
                assert all(k in r for k in ('error_code','message_zh','state','safety'))
            assert (await call('remote-control',{'enabled':True}))[0]==403
            page=await browser.get(url+'/')
            cookie=page.cookies['collection_web_session']
            assert cookie['httponly'] and cookie['samesite']=='Strict'
            web_headers=dict(headers,**{'X-Gamepad-VLA-Client':'web','Origin':url})
            assert (await call('remote-control',{'enabled':True},session=browser,h=web_headers))[0]==200
            assert (await call('session/stop',{},h={'Content-Type':'application/json'}))[0]==403
            assert (await call('session/stop',{},h=dict(headers,Origin='https://invalid.example')))[0]==403
            assert (await call('session/stop',{},h={'X-Gamepad-VLA-Client':'cli'}))[0]==200  # json= supplies required Content-Type
            async with remote.post(url+'/api/v1/session/stop',data='{}',headers={'X-Gamepad-VLA-Client':'cli','Content-Type':'text/plain'}) as invalid:
                assert invalid.status==403
            assert (await call('recording/start',{}))[0]==409
            assert (await call('session/start',dict(backend='sapien',source='ros2')))[0]==200
            assert (await call('session/start',{}))[0]==409
            code,r=await call('recording/start',{});assert code==409 and r['error_code']=='input_unhealthy'
            assert all(k in r['message_zh'] for k in ('left_hand','left_arm','right_hand','right_arm'))
            code,r=await call('recording/start',dict(force=True,note='forced test'));assert code==200
            identifier=r['episode_id'];assert identifier
            assert (await call('recording/start',{}))[0]==409
            assert (await call('notes',dict(text='note from CLI')))[0]==200
            assert (await call('episodes/'+identifier+'/void',{}))[0]==409
            assert (await call('recording/stop',{}))[0]==200
            assert (await call('recording/stop',{}))[0]==200
            for _ in range(100):
                code,ep=await call('episodes/'+identifier)
                if ep['integrity'] and ep['integrity']['state'] in ('passed','failed'):break
                await asyncio.sleep(.02)
            assert ep['meta']['forced'] and len(ep['meta']['recording_start_health'])==4
            assert ep['meta']['started_by']==ep['meta']['stopped_by']=='cli'
            assert ep['integrity']['passed']
            assert len((await call('episodes'))[1])==1
            assert (await call('episodes/'+identifier+'/void',dict(reason='test')))[1]['discarded']
            assert (await call('episodes/'+identifier+'/export',{}))[1]['exported']
            assert (await call('episodes/missing'))[0]==400
            assert (await call('episodes/missing/void',{}))[0]==400
            assert (await call('episodes/missing/export',{}))[0]==400
            assert (await call('notes',{}))[0]==400
            assert (await call('remote-control',dict(enabled='yes'),session=browser,h=web_headers))[0]==400
            async with remote.get(url+'/api/v1/events') as r:
                assert r.headers['Content-Type'].startswith('text/event-stream')
                assert (await r.content.readline()).startswith(b'data: ')
            assert (await call('session/stop',{}))[0]==200
            assert (await call('session/stop',{}))[0]==200
            assert (await call('notes',dict(text='n')))[0]==409
            assert (await call('remote-control',dict(enabled=False),session=browser,h=web_headers))[0]==200
            assert (await call('session/stop',{}))[0]==403
        await server.close()
        logs=[json.loads(line) for line in (tmp_path/'api_audit.jsonl').read_text().splitlines()]
        assert any(e['client']=='cli' and e['result']['http_status']==409 for e in logs)
        assert any(e['endpoint']=='/api/v1/remote-control' and e['client']=='web' for e in logs)
        assert all(k in logs[0] for k in ('wall_time_ns','endpoint','parameters','result','client'))
        server=TestServer(make_app(Desk(tmp_path)));await server.start_server()
        async with ClientSession() as c:
            async with c.get(server.make_url('/api/v1/status')) as r:assert not (await r.json())['remote_control_enabled']
        await server.close()
    asyncio.run(scenario())
