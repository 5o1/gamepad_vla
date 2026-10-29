"""Production view captures are opt-in, bounded, and isolated to one session."""
import asyncio
import json
import subprocess
import sys
from types import SimpleNamespace

import pytest

from gamepad_vla.workstation.runtime_logs import DiagnosticFile, CappedProcessLog
from gamepad_vla.workstation.collection_server import SessionManager, IdleSession


def test_default_short_view_session_has_no_frame_files(tmp_path, monkeypatch):
    from gamepad_vla.workstation import operator_view as module
    class Latest:
        def __init__(self, *args): pass
        def write(self, value): pass
        def close(self): pass
    monkeypatch.setitem(sys.modules, 'xbox_gamepad_physics.teleop.stream.ipc',
                        SimpleNamespace(Latest=Latest, SNAPSHOT_CAPACITY=1, FRAME_CAPACITY=1))
    # A config cannot silently turn on diagnostics.
    monkeypatch.setattr(module, 'load_config', lambda _: dict(diagnostics=True))
    monkeypatch.setattr(module.subprocess, 'Popen', lambda *a, **kw:
        SimpleNamespace(pid=1, poll=lambda: None, wait=lambda **kw: None, returncode=0))
    backend=SimpleNamespace(apply=lambda _:None, observe=lambda:SimpleNamespace(gamepad_signal=SimpleNamespace(tolist=lambda:[])),
        robot=SimpleNamespace(get_qpos=lambda:SimpleNamespace(tolist=lambda:[])), right_robot=None,
        gamepad=SimpleNamespace(articulation=SimpleNamespace(get_qpos=lambda:SimpleNamespace(tolist=lambda:[]))),close=lambda:None)
    view=module.OperatorView(backend, None, tmp_path/'view', tmp_path/'spec')
    for tick in range(100):
        view.apply([SimpleNamespace(stamp=tick)])
        view.observe()
    view.close(cancelled=True)
    files={p.name for p in view.run.iterdir()}
    assert not files & {'physics_timing.jsonl','render_timing.jsonl','stream_timing.jsonl','browser_timing.jsonl','operator.h264'}
    assert json.loads((view.run/'config.json').read_text())['diagnostics'] is False


@pytest.mark.parametrize('limit', ['time','rows','bytes'])
def test_diagnostics_stop_at_limits_and_do_not_resume(tmp_path, limit):
    clock=[0.]
    path=tmp_path/'capture.jsonl'
    capture=DiagnosticFile(path,True,clock=lambda:clock[0],seconds=2,
        rows=2 if limit=='rows' else 100,max_bytes=4 if limit=='bytes' else 1000)
    capture.write(b'x\n')
    if limit=='time':clock[0]=2.
    for _ in range(10):capture.write(b'x\n')
    assert path.read_bytes()==(b'x\n' if limit=='time' else b'x\nx\n')
    assert not capture.enabled
    clock[0]=0.
    capture.write(b'more')
    assert path.stat().st_size<=4
    disabled=DiagnosticFile(tmp_path/'off')
    disabled.write(b'no')
    assert not (tmp_path/'off').exists()


def test_child_log_is_capped_but_child_can_finish(tmp_path):
    log=CappedProcessLog(tmp_path/'child.log',max_bytes=1024)
    child=subprocess.Popen([sys.executable,'-c',"import os; os.write(1,b'x'*200000)"],stdout=log,stderr=log)
    assert child.wait(timeout=10)==0
    log.close()
    assert (tmp_path/'child.log').stat().st_size==1024


def test_diagnostics_are_not_inherited_from_config_or_last_session(tmp_path):
    async def scenario():
        replay=tmp_path/'replay';replay.touch()
        manager=SessionManager(tmp_path,default_values=dict(backend='sapien',source='replay',
            scene_variant='full',duration=None,rate_hz=60,replay=str(replay),diagnostics=True))
        options=[]
        def process(*, target,args):
            options.append(args[0])
            return SimpleNamespace(start=lambda:None,is_alive=lambda:False)
        manager.ctx=SimpleNamespace(Queue=lambda **kw:None,Event=lambda:None,Process=process)
        await manager.start(dict(diagnostics=True))
        await manager.start({})
        assert [o.diagnostics for o in options]==[True,False]
        with pytest.raises(ValueError,match='boolean'):
            await manager.start(dict(diagnostics='true'))
    asyncio.run(scenario())


def test_idle_clock_resets_for_recording_connections_and_activity():
    now=[0.]
    idle=IdleSession(clock=lambda:now[0])
    state=dict(session_state='running',recording_status=dict(state='idle'))
    now[0]=7199;assert not idle.expired(state)
    now[0]=7200;assert idle.expired(state)
    idle.connections=1;assert not idle.expired(state)
    now[0]+=10000;assert not idle.expired(state)
    idle.connections=0;idle.touch()
    now[0]+=7199;assert not idle.expired(state)
    state['recording_status']['state']='recording'
    now[0]+=10000;assert not idle.expired(state)
    state['recording_status']['state']='idle'
    now[0]+=7199;assert not idle.expired(state)
    now[0]+=1;assert idle.expired(state)
    idle.touch();assert not idle.expired(state)
