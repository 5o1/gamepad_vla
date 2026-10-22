#!/usr/bin/env python3
"""Local REST client for people and Codex; Python standard library only."""
import argparse
import json
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
        except ValueError:return exc.code,dict(error_code='http_error',message_zh='服务返回异常，请查看服务日志。')


def parser():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--url',default='http://127.0.0.1:8780')
    p.add_argument('--json',action='store_true')
    sub=p.add_subparsers(dest='command',required=True)
    for name in ('status','session-stop','record-start','record-stop','note','episodes','void','export','session-start'):
        s=sub.add_parser(name)
        s.add_argument('--json',action='store_true',default=argparse.SUPPRESS)
        s.add_argument('--url',default=argparse.SUPPRESS)
        if name=='session-start':
            s.add_argument('--backend',choices=['sapien','mujoco'],required=True)
            s.add_argument('--scene-variant',choices=['full','free-space'],required=True)
            s.add_argument('--source',choices=['ros2','udp','replay'],required=True)
            s.add_argument('--replay')
            s.add_argument('--duration-s',type=float)
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
    if not local:p.error('仅允许采集台本机回环服务地址')
    path='status';data=None
    if a.command=='session-start':
        path='session/start';data=dict(backend=a.backend,scene_variant=a.scene_variant,source=a.source,
                                      replay=a.replay,duration_s=a.duration_s)
    elif a.command=='session-stop':path='session/stop';data={}
    elif a.command=='record-start':path='recording/start';data=dict(note=a.note,force=a.force)
    elif a.command=='record-stop':path='recording/stop';data={}
    elif a.command=='note':path='notes';data=dict(text=a.text)
    elif a.command=='episodes':path='episodes'+('/'+quote(a.id,safe='') if a.id else '')
    elif a.command=='void':path='episodes/'+quote(a.id,safe='')+'/void';data=dict(reason=a.reason)
    elif a.command=='export':path='episodes/'+quote(a.id,safe='')+'/export';data={}
    try:
        status,result=request(a.url,path,data)
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
                        status=422;result=dict(result,error_code='integrity_failed',message_zh='完整性检查失败，请保留原始文件并查看检查结果。')
                    break
                if time.monotonic()>deadline:
                    status=504;result=dict(error_code='integrity_pending',message_zh='本段完整性检查等待超时，未确认通过，请用 status 或 episodes 查询。');break
                time.sleep(.05)
    except (OSError,URLError,TimeoutError) as exc:
        result=dict(error_code='service_unreachable',message_zh='服务不可达，请确认采集台本机服务已启动。',detail=str(exc))
        print(json.dumps(result,ensure_ascii=False) if a.json else result['message_zh'])
        return 3
    if a.json:print(json.dumps(result,ensure_ascii=False))
    elif status!=200:print(result.get('message_zh','操作失败，请查看服务日志。'))
    elif isinstance(result,list):
        print('\n'.join(f"{e['id']} · {e.get('rows',0)} 行 · {'作废' if e.get('discarded') else '有效'}" for e in result) or '暂无 episode。')
    else:
        state=result.get('state',result);rec=state.get('recording_status',{})
        if a.command=='record-stop' and a.wait and result.get('integrity'):
            print('已停止录制 '+result.get('id','')+'，完整性检查通过。')
        elif a.command=='export':print('已导出：'+result['output'])
        elif a.command=='void':print('已标为作废，原始数据保留。')
        else:
            labels=dict(idle='未录制',recording='录制中',saving='保存中',checking='检查中',saved='已保存',failed='失败')
            print(f"会话 {state.get('session_state','—')}；{labels.get(rec.get('state'),'检查已完成' if result.get('integrity') else '—')}；episode {result.get('id') or state.get('episode_id') or '—'}；远程控制 {'开' if state.get('remote_control_enabled') else '关'}。")
    return 0 if status==200 else 2 if status in (403,409) else 1


if __name__=='__main__':sys.exit(main())
