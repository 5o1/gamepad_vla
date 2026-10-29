"""Station-only Task U acceptance; temporary replay server, no hardware commands.

Run with the existing station Python and runtime. --root must be a NEW directory.
The temporary service enables remote control at initial launch for this explicitly
requested replay acceptance; it never changes the production service's switch.
"""
import argparse
import json
import os
from pathlib import Path
import runpy
import shutil
import signal
import subprocess
import sys
import time

from gamepad_vla.site import repository_root
from gamepad_vla.workstation.runtime_logs import CappedProcessLog


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--port',type=int,default=17881)
    args=parser.parse_args()
    args.root.mkdir(parents=True,exist_ok=False)
    code=repository_root()
    ctl=runpy.run_path(str(code/'scripts/collection_ctl.py'))
    url=f'http://127.0.0.1:{args.port}'
    def request(path,data=None):
        status,value=ctl['request'](url,path,data)
        if status!=200:raise RuntimeError(f'{path}: {status} {value}')
        return value
    def until(predicate,timeout=120):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            value=predicate()
            if value:return value
            time.sleep(.1)
        raise TimeoutError('Acceptance condition did not arrive')
    def inventory(root):
        return {str(p.relative_to(root)):dict(bytes=p.stat().st_size,blocks=p.stat().st_blocks*512)
                for p in root.rglob('*') if p.is_file()}
    env=dict(os.environ,ROS_DOMAIN_ID='87',CUDA_VISIBLE_DEVICES='0')
    log=CappedProcessLog(args.root/'server.log')
    server=subprocess.Popen(['bash',str(code/'scripts/station/start_collection_server.sh'),
        '--host','127.0.0.1','--port',str(args.port),'--source','replay',
        '--spec','configs/workcell/station_g20_v4_4_raised_interim.json',
        '--scene-variant','gamepad-contact','--arm-startup','none',
        '--enable-remote-control','--output-root',str(args.root/'sessions')],
        cwd=code,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    result={}
    try:
        def ready():
            if server.poll() is not None:raise RuntimeError('Temporary service exited')
            try:return request('status')
            except OSError:return None
        before_status=until(ready)
        assert before_status['remote_control_enabled'] and before_status['session_state']=='idle'
        request('session/start',dict(backend='sapien',source='replay',scene_variant='gamepad-contact'))
        def running():
            s=request('status')
            if s.get('session_state')=='failed':raise RuntimeError(s.get('error'))
            return s if s.get('session_state')=='running' and (s.get('stream') or {}).get('frames',0)>30 else None
        status=until(running)
        session=Path(status['output'])
        before=inventory(args.root)
        begin=time.monotonic()
        for second in range(180):
            time.sleep(max(0,begin+second+1-time.monotonic()))
            status=request('status')
            assert status['session_state']=='running' and status['recording_status']['state']=='idle'
            if (second+1)%30==0:print(f'idle smoke {second+1}/180 s',flush=True)
        after=inventory(args.root)
        all_names=before.keys()|after.keys()
        changes={name:dict(bytes=after.get(name,{}).get('bytes',0)-before.get(name,{}).get('bytes',0),
            blocks=after.get(name,{}).get('blocks',0)-before.get(name,{}).get('blocks',0)) for name in all_names
            if before.get(name)!=after.get(name)}
        forbidden=['render_timing.jsonl','stream_timing.jsonl','physics_timing.jsonl','browser_timing.jsonl','operator.h264']
        assert not any((session/'view'/name).exists() for name in forbidden)
        result['idle']=dict(seconds=time.monotonic()-begin,
            growth_bytes=sum(v['bytes'] for v in changes.values()),
            growth_allocated_bytes=sum(v['blocks'] for v in changes.values()),changes=changes,
            session_startup_bytes=sum(v['bytes'] for n,v in before.items() if n.startswith('sessions/')),
            session_startup_allocated_bytes=sum(v['blocks'] for n,v in before.items() if n.startswith('sessions/')),
            stream=status['stream'],forbidden_files_present=[])
        assert result['idle']['growth_bytes']<16384
        request('recording/start',dict(note='Task U 30 s replay acceptance'))
        status=until(lambda:(s if (s:=request('status'))['recording_status']['state']=='recording' else None))
        identifier=status['episode_id']
        time.sleep(30)
        # Use the mandated idempotent CLI stop-and-wait, not an ad-hoc wait.
        stopped=subprocess.run([sys.executable,str(code/'scripts/collection_ctl.py'),
            '--url',url,'record-stop','--wait','--json'],check=True,capture_output=True,text=True)
        episode=json.loads(stopped.stdout)
        assert episode['id']==identifier and episode['integrity']['passed'] is True
        status=request('status')
        assert status['recording_status']['state'] not in ('recording','saving','checking')
        result['recording']=dict(rows=episode['meta']['rows'],integrity=episode['integrity'],
            duration_s=episode['meta']['recording_end_sim_time_s']-episode['meta']['recording_start_sim_time_s'])
        assert result['recording']['duration_s']>=29
        request('session/stop',{})
        audit=[json.loads(line) for line in (args.root/'sessions/api_audit.jsonl').read_text().splitlines()]
        assert all(e['method']=='POST' for e in audit)
        result['audit']=dict(rows=len(audit),successful_reads=0)
        print(json.dumps(result,indent=2),flush=True)
    finally:
        if server.poll() is None:
            # Stop and recheck via the same CLI even on acceptance failure.
            subprocess.run([sys.executable,str(code/'scripts/collection_ctl.py'),'--url',url,
                'record-stop','--wait','--json'],capture_output=True,timeout=140)
            try:request('session/stop',{})
            finally:
                os.killpg(server.pid,signal.SIGTERM)
                server.wait(timeout=45)
        log.close()
    # Only delete the directory that this invocation created, after all children exit.
    shutil.rmtree(args.root)
    result['cleaned']=not args.root.exists()
    print('TASK_U_RESULT '+json.dumps(result),flush=True)


if __name__=='__main__':main()
