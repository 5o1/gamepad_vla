"""Camera proxy, file compatibility, ownership and browser binding contracts."""
import asyncio
import base64
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
from pathlib import Path
import re
import signal
import shutil
import subprocess
import sys
import threading
import time
from types import SimpleNamespace
import zipfile

import pytest

from gamepad_vla.workstation.camera_panel import CameraPanel, CameraConflict, camera_owners, stop_group

PNG = base64.b64decode('iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+jZ1kAAAAASUVORK5CYII=')
META = dict(serial='fixture', width=1280, height=720, K=[906.,0.,648.,0.,905.,362.,0.,0.,1.],
            D=[0.]*5, R=[1.,0.,0.,0.,1.,0.,0.,0.,1.], P=[0.]*12, age_s=.01,
            image_stamp=dict(sec=12,nanosec=34), captured_at_utc='2026-10-23T00:00:00+00:00')


@pytest.fixture
def preview():
    calls=[]
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*a):pass
        def do_GET(self):self.route('GET')
        def do_POST(self):self.route('POST')
        def route(self,method):
            calls.append((method,self.path))
            data=json.dumps(META).encode();kind='application/json'
            if self.path=='/preview.jpg':data=b'jpeg-fixture';kind='image/jpeg'
            elif self.path=='/snapshot':
                stream=io.BytesIO()
                with zipfile.ZipFile(stream,'w') as z:
                    z.writestr('image.png',PNG);z.writestr('camera.json',json.dumps(META))
                data=stream.getvalue();kind='application/zip'
            self.send_response(200);self.send_header('Content-Type',kind);self.end_headers();self.wfile.write(data)
    server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    yield server.server_port,calls
    server.shutdown();server.server_close();thread.join()


def panel(tmp_path,preview):
    p=CameraPanel(tmp_path,settings={'camera.camera_3':'fixture'},owner_scan=lambda serial:[])
    p.preview_port=preview[0];p.mode='preview';p.ensure_session()
    return p


def test_snapshot_preserves_old_lossless_png_and_metadata(tmp_path,preview):
    p=panel(tmp_path,preview)
    shot=p.snapshot('position_1 测试/../')
    path=Path(shot['path']);meta=json.loads(path.with_suffix('.json').read_text())
    assert path.parent.parent==tmp_path/'calibration'
    assert re.fullmatch(r'\d{8}-\d{6}',p.session)
    assert re.fullmatch(r'position_1_[A-Za-z0-9_-]*_\d{8}T\d{6}_\d{6}Z.png',path.name)
    assert path.read_bytes()==PNG
    assert all(meta[k]==v for k,v in META.items())
    assert meta['label']=='position_1 测试/../' and meta['image_file']==path.name
    assert meta['saved_locally_at_utc'] and p.photos()[0]['name']==path.stem
    assert p.photo(path.name)==path
    assert ('POST','/snapshot') in preview[1]
    for name in ('../photo.png','/tmp/photo.png','file.zip'):
        with pytest.raises(FileNotFoundError):p.photo(name)
    p.mode='recording'
    with pytest.raises(CameraConflict):p.snapshot('busy')


def test_occupancy_real_substitute_process(tmp_path):
    child=subprocess.Popen(['realsense2_camera_node','30'],executable='/bin/sleep')
    try:
        owners=camera_owners('fixture')
        assert child.pid in {p['pid'] for p in owners}
        p=CameraPanel(tmp_path,settings={'camera.camera_3':'fixture'})
        with pytest.raises(CameraConflict,match=f'PID {child.pid}'):
            p.start()
        assert child.poll() is None
    finally:
        child.terminate();child.wait(timeout=3)


EGO_STUB = '''import argparse,json,signal,threading
from pathlib import Path
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
p=argparse.ArgumentParser()
for k in ('host','port','output-dir','video-output-dir'):p.add_argument('--'+k,required=True)
p.add_argument('--no-open',action='store_true');a=p.parse_args()
root=Path(a.video_output_dir);root.mkdir(parents=True);state={'running':False,'recording':False,'last_recording_report':None,'unmodified':'fixture extra field'}
class H(BaseHTTPRequestHandler):
 def log_message(self,*a):pass
 def do_GET(self):self.send(state)
 def send(self,data):
  self.send_response(200);self.send_header('Content-Type','application/json');self.end_headers();self.wfile.write(json.dumps(data).encode())
 def do_POST(self):
  with (root/'calls.jsonl').open('a') as f:f.write(json.dumps(self.path)+'\\n')
  result=state
  if self.path=='/api/start':state['running']=True
  elif self.path=='/api/record/start':state.update(running=False,recording=True)
  elif self.path=='/api/record/stop':
   for n in ('ego.mp4','ego_depth_aligned.mkv','ego_depth.json','ego_frames.jsonl'):(root/n).write_text('{}')
   report={'quality':{'status':'warning','warnings':['fixture']},'unchanged':[1,2,3]}
   (root/'ego_recording_report.json').write_text(json.dumps(report))
   state.update(running=True,recording=False,last_recording_report=report)
   result={'kind':'rgbd_video','recording_report':report,'rgb_path':str(root/'ego.mp4')}
  elif self.path=='/api/stop':state.update(running=False,recording=False)
  self.send(result)
s=ThreadingHTTPServer((a.host,int(a.port)),H)
signal.signal(signal.SIGTERM,lambda *args:threading.Thread(target=s.shutdown).start())
s.serve_forever();s.server_close()
'''


@pytest.fixture
def ego_panel(tmp_path):
    script=tmp_path/'ego_web.py';script.write_text(EGO_STUB)
    p=CameraPanel(tmp_path/'data',settings={'camera.camera_3':'fixture',
        'camera.ego_web_script':str(script),'station.python':sys.executable},owner_scan=lambda serial:[])
    events=[]
    # Preview hardware alone is replaced; the real upstream proxy is a child server.
    def start():events.append('preview/start');p.mode='preview'
    def stop():events.append('preview/stop');p.mode='idle'
    p._start_preview=start;p._stop_preview=stop
    yield p,events
    p.stop()


@pytest.mark.parametrize('restore',[True,False])
def test_record_proxy_outputs_and_exclusive_handoff(ego_panel,restore):
    p,events=ego_panel
    if restore:p.start()
    result=p.record_start();child=p.ego
    assert result['recording'] and result['unmodified']=='fixture extra field'
    assert p.mode=='recording' and events[-1]=='preview/stop'
    assert p.record_state()==result
    with pytest.raises(CameraConflict):p.start()
    with pytest.raises(CameraConflict):p.record_start()
    result=p.record_stop()
    assert result['kind']=='rgbd_video' and result['recording_report']['unchanged']==[1,2,3]
    assert p.status()['recording_report']==result['recording_report']
    assert p.mode==('preview' if restore else 'idle')
    assert child.poll() is not None and p.ego is None
    path=p.data_root/'camera'/p.session/'videos'
    assert all((path/n).is_file() for n in ('ego.mp4','ego_depth_aligned.mkv','ego_depth.json','ego_frames.jsonl','ego_recording_report.json'))
    calls=[json.loads(l) for l in (path/'calls.jsonl').read_text().splitlines()]
    assert calls==['/api/start','/api/record/start','/api/record/stop','/api/stop']
    assert p.record_stop()==result  # idempotent


def test_stop_during_recording_does_not_restore_preview(ego_panel):
    p,events=ego_panel;p.start();p.record_start();child=p.ego
    p.stop()
    assert p.mode=='idle' and child.poll() is not None and events.count('preview/start')==1


def test_camera_routes_auth_proxy_audit_and_conflict(tmp_path,preview):
    pytest.importorskip('aiohttp')
    from aiohttp import ClientSession, CookieJar
    from aiohttp.test_utils import TestServer
    from gamepad_vla.workstation.collection_server import make_app,SessionManager
    async def run():
        p=panel(tmp_path/'data',preview)
        p.stop=lambda:dict(mode='idle')
        p.start=lambda:dict(mode='preview')
        p.record_start=lambda:dict(recording=True,extra='untouched')
        p.record_stop=lambda:dict(recording_report={'quality':{'status':'ok'}})
        p.record_state=lambda:dict(recording=False,extra='untouched')
        m=SessionManager(tmp_path/'episodes')
        server=TestServer(make_app(m,camera=p));await server.start_server()
        headers={'X-Gamepad-VLA-Client':'test'}
        try:
            async with ClientSession(cookie_jar=CookieJar(unsafe=True)) as browser,ClientSession() as remote:
                url=server.make_url
                for action in ('start','stop','snapshot','record/start','record/stop'):
                    r=await remote.post(url('/api/v1/camera/'+action),json={},headers=headers)
                    assert r.status==403
                await browser.get(url('/'))
                for action in ('start','snapshot','record/start','record/stop','stop'):
                    r=await browser.post(url('/api/v1/camera/'+action),json={'label':'board'},headers=headers)
                    assert r.status==200,(action,await r.text())
                    value=await r.json()
                    if action=='record/start':assert value==dict(recording=True,extra='untouched')
                r=await remote.get(url('/api/v1/camera/preview.jpg'));assert r.status==200 and await r.read()==b'jpeg-fixture'
                assert ('GET','/preview.jpg') in preview[1]
                for path in ('status','photos','record/state'):
                    r=await remote.get(url('/api/v1/camera/'+path));assert r.status==200
                for method,path in preview[1]:
                    if path=='/snapshot':assert method=='POST'
                r=await remote.get(url('/api/v1/camera/photos'));photo=(await r.json())[0]
                r=await remote.get(url('/api/v1/camera/photos/'+photo['name']+'.png'));assert await r.read()==PNG
                r=await remote.get(url('/api/v1/camera/photos/missing.png'));assert r.status==404
                r=await remote.get(url('/api/v1/camera/start'));assert r.status==405
                child=subprocess.Popen(['realsense2_camera_node','30'],executable='/bin/sleep')
                try:
                    p.start=lambda:CameraPanel(tmp_path,settings={'camera.camera_3':'fixture'}).check_available()
                    r=await browser.post(url('/api/v1/camera/start'),json={},headers=headers)
                    assert r.status==409 and str(child.pid) in (await r.json())['message_zh']
                    assert child.poll() is None
                finally:child.terminate();child.wait(timeout=3)
        finally:await server.close()
        rows=[json.loads(l) for l in (m.root/'api_audit.jsonl').read_text().splitlines()]
        assert any(r['endpoint']=='/api/v1/camera/snapshot' and r['parameters'].get('label')=='board' and r['result']['http_status']==200 for r in rows)
        assert any(r['result']['http_status']==409 and r['result']['error_code']=='camera_busy' for r in rows)
    asyncio.run(run())


@pytest.mark.skipif(shutil.which('node') is None,reason='browser binding test requires an existing Node executable')
def test_browser_buttons_use_post_and_registered_routes():
    root=Path(__file__).parents[1]
    # Execute exported action map and actual onclick handlers with a minimal DOM.
    js=root/'src/gamepad_vla/workstation/collection_web/camera-panel.mjs'
    script='''import {cameraActions,installCameraPanel} from MODULE_URL;
import assert from 'node:assert/strict';
const els=new Map();const get=id=>{if(!els.has(id))els.set(id,{value:'position_1',textContent:'',classList:{toggle(){}},replaceChildren(){},append(){}});return els.get(id)};
let calls=[];globalThis.setTimeout=()=>0;
const fetcher=async(path,options={})=>{calls.push({path,options});return {ok:true,json:async()=>path.endsWith('/photos')?[]:path.endsWith('/status')?{mode:'idle',width:1280,height:720}:{name:'fixture'}}};
installCameraPanel({getElementById:get,createElement:()=>({append(){}})},fetcher);
for(const [id,path] of Object.entries(cameraActions)){calls=[];await get(id).onclick();const call=calls.find(c=>c.path===path);assert(call);assert.equal(call.options.method,'POST');assert.equal(call.options.headers['Content-Type'],'application/json');assert.equal(call.options.headers['X-Gamepad-VLA-Client'],'web');if(id==='cameraSnapshot')assert.equal(JSON.parse(call.options.body).label,'position_1')}
get('cameraGrid').onchange();get('cameraCross').onchange();
console.log(JSON.stringify(cameraActions));
'''.replace('MODULE_URL',json.dumps(js.as_uri()))
    result=subprocess.run(['node','--input-type=module','-e',script],check=True,capture_output=True,text=True)
    actions=json.loads(result.stdout)
    page=(js.parent/'index.html').read_text()
    assert all(f'id="{id}"' in page for id in actions)
    assert '<script type="module" src="/camera-panel.mjs"></script>' in page
    assert not (root/'scripts/station'/('camera_'+'capture.py')).exists()


def test_cleanup_includes_upstream_detached_child_sessions(tmp_path):
    script=tmp_path/'parent.py';pidfile=tmp_path/'child.pid'
    script.write_text("import subprocess,time,sys\nfrom pathlib import Path\np=subprocess.Popen(['/bin/sleep','30'],start_new_session=True)\nPath(sys.argv[1]).write_text(str(p.pid))\ntime.sleep(30)\n")
    process=subprocess.Popen([sys.executable,str(script),str(pidfile)],start_new_session=True)
    try:
        deadline=time.monotonic()+5
        while not pidfile.exists():
            assert time.monotonic()<deadline
            time.sleep(.01)
        child=int(pidfile.read_text())
        stop_group(process,timeout=2)
        stat=Path(f'/proc/{child}/stat')
        assert not stat.exists() or stat.read_text().rsplit(')',1)[1].split()[0]=='Z'
        assert process.poll() is not None
    finally:
        if process.poll() is None:stop_group(process,timeout=2)
