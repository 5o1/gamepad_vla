#!/usr/bin/env python3
"""Local REST client for people and Codex; Python standard library only."""
import argparse
import json
from pathlib import Path
import sys
import time
from urllib.error import HTTPError, URLError
from urllib.parse import quote, urlparse
from urllib.request import Request, urlopen


def request(base,path,data=None):
    headers={'X-Gamepad-VLA-Client':'cli','Accept':'application/json'}
    if data is not None:headers['Content-Type']='application/json'
    req=Request(base.rstrip('/')+'/api/v1/'+path,headers=headers,
                data=None if data is None else json.dumps(data).encode())
    try:
        with urlopen(req,timeout=60) as response:return 200,json.load(response)
    except HTTPError as exc:
        try:return exc.code,json.load(exc)
        except ValueError:return exc.code,dict(error_code='http_error',message='Invalid server response. Check the service log.')


def parser():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--url',default='http://127.0.0.1:8780')
    p.add_argument('--json',action='store_true')
    sub=p.add_subparsers(dest='command',required=True)
    for name in ('status','session-stop','record-start','record-stop','note','episodes','void','export','session-start','snapshot','snapshot-cancel','snapshots','snapshot-get'):
        s=sub.add_parser(name)
        s.add_argument('--json',action='store_true',default=argparse.SUPPRESS)
        s.add_argument('--url',default=argparse.SUPPRESS)
        if name=='snapshot':s.add_argument('--delay-s',type=float);s.add_argument('--label',default='')
        if name=='snapshots':s.add_argument('name',nargs='?')
        if name=='snapshot-get':s.add_argument('name');s.add_argument('--output',type=Path,required=True)
        if name=='session-start':
            s.add_argument('--backend',choices=['sapien','mujoco'],required=True)
            s.add_argument('--scene-variant',choices=['full','free-space','gamepad-contact'],required=True)
            s.add_argument('--source',choices=['ros2','udp','replay'],required=True)
            s.add_argument('--replay')
            s.add_argument('--duration-s',type=float)
            s.add_argument('--diagnostics',action='store_true',help='bounded capture for this session only')
        if name=='record-start':s.add_argument('--note',default='');s.add_argument('--force',action='store_true')
        if name=='record-stop':s.add_argument('--wait',action='store_true');s.add_argument('--wait-timeout',type=float,default=120)
        if name=='note':s.add_argument('text')
        if name=='episodes':s.add_argument('id',nargs='?')
        if name in ('void','export'):s.add_argument('id')
        if name=='void':s.add_argument('--reason',required=True)
    return p


def main(argv=None):
    p=parser();a=p.parse_args(argv)
    host=urlparse(a.url).hostname
    import ipaddress
    try:local=host=='localhost' or ipaddress.ip_address(host).is_loopback
    except ValueError:local=False
    if not local:p.error('Only station-local loopback URLs are allowed')
    path='status';data=None
    if a.command=='session-start':
        path='session/start';data=dict(backend=a.backend,scene_variant=a.scene_variant,source=a.source,
                                      replay=a.replay,duration_s=a.duration_s,diagnostics=a.diagnostics)
    elif a.command=='session-stop':path='session/stop';data={}
    elif a.command=='record-start':path='recording/start';data=dict(note=a.note,force=a.force)
    elif a.command=='record-stop':path='recording/stop';data={}
    elif a.command=='note':path='notes';data=dict(text=a.text)
    elif a.command=='episodes':path='episodes'+('/'+quote(a.id,safe='') if a.id else '')
    elif a.command=='void':path='episodes/'+quote(a.id,safe='')+'/void';data=dict(reason=a.reason)
    elif a.command=='snapshot':path='snapshot';data=dict(delay_s=a.delay_s,label=a.label)
    elif a.command=='snapshot-cancel':path='snapshot/cancel';data={}
    elif a.command in ('snapshots','snapshot-get'):path='snapshots'+('/'+quote(a.name,safe='/') if a.name else '')
    elif a.command=='export':path='episodes/'+quote(a.id,safe='')+'/export';data={}
    try:
        status,result=request(a.url,path,data)
        if status==200 and a.command=='snapshot-get':
            try:
                a.output.mkdir(parents=True,exist_ok=True)
                for filename in result['files']:
                    if Path(filename).name!=filename:raise ValueError('Invalid snapshot filename')
                    if (a.output/filename).exists():raise FileExistsError(f'Refusing to overwrite {a.output/filename}')
                for filename in result['files']:
                    req=Request(a.url.rstrip('/')+'/api/v1/'+path+'/'+quote(filename,safe=''),headers={'X-Gamepad-VLA-Client':'cli'})
                    with urlopen(req,timeout=60) as response, (a.output/filename).open('xb') as output:
                        import shutil
                        shutil.copyfileobj(response,output)
                result=dict(result,output=str(a.output))
            except (OSError,ValueError) as exc:
                status=500;result=dict(error_code='snapshot_download_failed',message=f'Snapshot download failed: {exc}')
        if status==200 and a.command=='record-stop' and a.wait and result.get('episode_id'):
            target=result['episode_id']
            deadline=time.monotonic()+a.wait_timeout
            while True:
                code,episode=request(a.url,'episodes/'+quote(target,safe=''))
                if code!=200:status,result=code,episode;break
                check=episode.get('integrity') or {}
                active=episode.get('state',{}).get('integrity') or {}
                active_pending=active.get('episode_id')==target and active.get('state') in ('pending','running')
                if check.get('episode_id')==target and check.get('state') in ('passed','failed') and not active_pending:
                    result=episode
                    if check['state']=='failed':
                        status=422;result=dict(result,error_code='integrity_failed',message='Integrity check failed. Keep the original files and review the results.')
                    break
                if time.monotonic()>deadline:
                    status=504;result=dict(error_code='integrity_pending',message='Integrity check timed out. Passing is unconfirmed; query status or episodes.');break
                time.sleep(.05)
    except (OSError,URLError,TimeoutError) as exc:
        result=dict(error_code='service_unreachable',message='Service unreachable. Check that the service is running on the station.',detail=str(exc))
        print(json.dumps(result,ensure_ascii=False) if a.json else result['message'])
        return 3
    if a.json:print(json.dumps(result,ensure_ascii=False))
    elif status!=200:print(result.get('message','Operation failed. Check the service log.'))
    elif isinstance(result,list):
        if a.command=='snapshots':print('\n'.join(e['name'] for e in result) or 'No snapshots yet.')
        else:print('\n'.join(f"{e['id']} · {e.get('rows',0)} rows · {'Void' if e.get('discarded') else 'Valid'}" for e in result) or 'No episodes yet.')
    else:
        state=result.get('state',result);rec=state.get('recording_status',{})
        if a.command=='record-stop' and a.wait and result.get('integrity'):
            print('Stopped recording '+result.get('id','')+'; integrity: passed.')
        elif a.command=='snapshot-get':print('Snapshot downloaded: '+result['output'])
        elif a.command=='snapshots':print(result['name']+' · '+', '.join(result['files']))
        elif a.command in ('snapshot','snapshot-cancel'):
            snap=result.get('snapshot',{});print(f"Snapshot {snap.get('state','unknown')}"+(f" · {snap.get('remaining_s',0):.1f} s remaining" if snap.get('state')=='countdown' else ''))
        elif a.command=='export':print('Exported: '+result['output'])
        elif a.command=='void':print('Marked as void. Original data retained.')
        else:
            labels=dict(idle='Idle',recording='Recording',saving='Saving',checking='Checking',saved='Saved',failed='Failed')
            print(f"Session {state.get('session_state','—')}; {labels.get(rec.get('state'),'Check complete' if result.get('integrity') else '—')}; episode {result.get('id') or state.get('episode_id') or '—'}; Remote control {'On' if state.get('remote_control_enabled') else 'Off'}.")
    return 0 if status==200 else 2 if status in (403,409) else 1


if __name__=='__main__':sys.exit(main())
