"""Collection web data-quality, lifecycle and read-only safety contracts."""
import asyncio
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import queue
import subprocess
import time
from types import SimpleNamespace

import numpy as np
import pytest

from gamepad_vla.workstation.collection_controller import CollectionController, CollectionOptions
from gamepad_vla.workstation.collection_health import InputHealth, detect_safety, require_loopback
from gamepad_vla.workstation.collection_integrity import check_integrity, discard_episode
from gamepad_vla.workstation.episode_schema import EpisodeWriter, read_episode
from gamepad_vla.workstation.arm_mapping import ArmMapper
from test_collection import observation


def episode(path, *, events=()):
    writer = EpisodeWriter(path, dict(scene_spec_text='{}', fps=50, recording_start_sim_time_s=0,
        recording_end_sim_time_s=.04, received_input_messages=len(events),
        receive_start_sequence=events[0]['sequence'] if events else 0,
        receive_end_sequence=events[-1]['sequence']+1 if events else 0))
    writer.append_inputs(events)
    for t in (.02,.04):
        writer.append(observation(t), {}, {})
    writer.close()
    return path


@pytest.mark.parametrize('damage', ['none','truncated','corrupt','receiver_count','row_duration','sequence_gap','empty'])
def test_integrity_positive_and_negative_controls(tmp_path, damage):
    from gamepad_vla.workstation.collection_sources import command_event
    mapper = ArmMapper({s:np.zeros(7) for s in ('left','right')})
    events = [command_event(mapper.command('left',[0.]*7,0.), i) for i in range(3)]
    if damage=='sequence_gap':
        events[1]['sequence']=2;events[2]['sequence']=3
    path=episode(tmp_path/'ep',events=events)
    if damage in ('truncated','corrupt'):
        p=path/'steps.jsonl';data=p.read_bytes();p.write_bytes(data[:-25] if damage=='truncated' else b'broken\n'+data)
    if damage in ('receiver_count','row_duration','empty'):
        meta=json.loads((path/'meta.json').read_text())
        if damage=='receiver_count':meta['received_input_messages']+=1
        elif damage=='row_duration':meta['recording_end_sim_time_s']=10.
        else:meta['rows']=0;(path/'steps.jsonl').write_text('')
        (path/'meta.json').write_text(json.dumps(meta))
    result=check_integrity(path)
    assert result['passed'] is (damage=='none'),result


def test_discard_does_not_change_raw_data(tmp_path):
    path=episode(tmp_path/'ep')
    before={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in path.iterdir()}
    discard_episode(path,'bad trial')
    assert json.loads((path/'discarded.json').read_text())['discarded']
    assert before=={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in path.iterdir() if p.name!='discarded.json'}
    assert read_episode(path)[0]['status']=='complete'


def test_clamps_schema_v4_and_legacy(tmp_path):
    mapper=ArmMapper({s:np.zeros(7) for s in ('left','right')})
    c=mapper.command('right',[1000.]*7,1.)
    assert all(c.clamp_flags['absolute'])
    assert any(c.clamp_flags['startup'])
    writer=EpisodeWriter(tmp_path/'ep',dict(scene_spec_text='{}'))
    flags=dict(left=None,right=c.clamp_flags)
    writer.append(observation(),{}, {},arm_clamps=flags);writer.close()
    assert read_episode(tmp_path/'ep')[1][0]['arm_clamps']==flags
    p=tmp_path/'ep/meta.json';meta=json.loads(p.read_text())
    for version in (3,2):
        meta['schema_version']=version;p.write_text(json.dumps(meta))
        assert read_episode(p.parent)[0]['schema_version']==version
    meta['schema_version']=4;p.write_text(json.dumps(meta))
    row=json.loads((p.parent/'steps.jsonl').read_text());row['arm_clamps']['right']['absolute'][0]=1
    (p.parent/'steps.jsonl').write_text(json.dumps(row)+'\n')
    with pytest.raises(ValueError,match='clamp'):read_episode(p.parent)


def test_health_frequency_errors_and_stale():
    clock=[0.];health=InputHealth(lambda:clock[0])
    assert all(c['warning'] for c in health.snapshot().values())
    for i in range(200):
        clock[0]=i/100;health.receive('left_arm',error=i==5)
    c=health.snapshot()['left_arm']
    assert c['hz']==pytest.approx(100) and c['count']==200 and c['decode_errors']==1
    clock[0]+=.251
    assert health.snapshot()['left_arm']['warning']


def test_readonly_safety_real_testdouble_and_recovery():
    # The matching text is argv[0], not a shell command or hardware executable.
    before=detect_safety();before_pids={p['pid'] for p in before['matches']}
    child=subprocess.Popen(['linker_hand_ros2_sdk_testdouble','3'],executable='/bin/sleep')
    try:
        current=detect_safety()
        assert not current['safe'] and child.pid in {p['pid'] for p in current['matches']}
    finally:
        child.terminate();child.wait(timeout=3)
    assert {p['pid'] for p in detect_safety()['matches']}==before_pids


def test_safety_matches_rokae_and_scan_failure(tmp_path):
    p=tmp_path/'123';p.mkdir();(p/'cmdline').write_bytes(b'rokae_readonly_node\0')
    assert detect_safety(tmp_path)['matches'][0]['pid']==123
    assert not detect_safety(tmp_path/'missing')['safe']


@pytest.mark.parametrize('host',['0.0.0.0','192.0.2.1','::'])
def test_nonloopback_refused(host):
    with pytest.raises(ValueError):require_loopback(host)


def test_controller_receive_boundaries_and_stop(tmp_path):
    from gamepad_vla.workstation.collection import Session
    # Use the real lifecycle/recorder with the simulator replaced by observations.
    session=Session.__new__(Session)
    session.output=tmp_path;session.meta=dict(scene_spec_text='{}',fps=50,operator_notes=[])
    session.writer=None;session.last_sim_time_s=0.;session.episode_index=0;session.pending_notes=[]
    session.control_log=(tmp_path/'events.jsonl').open('w')
    session.recording_state='idle';session.last_saved=None;session.on_status=lambda:None
    session.commands=dict.fromkeys(('left_arm','right_arm','left_hand','right_hand'));session.input_sequences={}
    source=SimpleNamespace(boundary=lambda:([],7),health=InputHealth())
    controller=CollectionController(CollectionOptions('sapien','udp',tmp_path,1.))
    controller.session=session;controller.source=source
    controller.control(dict(action='start'))
    for t in (.02,.04):session.record(observation(t),[])
    controller.control(dict(action='stop'))
    assert session.recording_state=='saving'
    session.wait_saved()
    assert session.recording_state=='saved'
    meta,rows=read_episode(tmp_path/'episode-000000')
    assert meta['received_input_messages']==0 and meta['receive_start_sequence']==7
    assert check_integrity(tmp_path/'episode-000000')['passed']
    session.close()


def test_controller_cli_and_web_use_run_path():
    root=Path(__file__).parents[1]
    assert 'CollectionController(' in (root/'scripts/run_collection.py').read_text()
    from gamepad_vla.workstation.collection_controller import run_managed
    assert 'CollectionController' in run_managed.__code__.co_names


def test_manager_single_session_and_path_boundary(tmp_path, monkeypatch):
    from gamepad_vla.workstation.collection_server import SessionManager
    m=SessionManager(tmp_path)
    m.worker=SimpleNamespace(is_alive=lambda:True)
    with pytest.raises(ValueError,match='only one'):
        asyncio.run(m.start({}))
    with pytest.raises(ValueError,match='within'):
        m.episode('../outside')
    import gamepad_vla.workstation.collection_server as server
    monkeypatch.setattr(server, 'site_value', lambda *args: str(tmp_path/'missing-python'))
    with pytest.raises(ValueError,match='environment unavailable'):
        path=episode(tmp_path/'session-1/episode-000000')
        asyncio.run(m.export(str(path.relative_to(tmp_path))))


def test_controller_final_status_does_not_publish_to_closed_view(tmp_path,monkeypatch):
    import gamepad_vla.workstation.collection_controller as module
    closed=[False]
    class Source:
        decode_errors=0;health=InputHealth()
        def boundary(self):return [],0
        def drain_events(self):return []
        def poll(self,now):return []
        def close(self):pass
    class Backend:
        def reset(self):pass
        def apply(self,commands):pass
        def step(self,dt):pass
        def observe(self):return observation()
        def close(self):pass
    class Recorder:
        writer=None
        meta={}
        def __init__(self,*args,**kwargs):pass
        def record_inputs(self,events):pass
        def record(self,obs,commands,**kwargs):return obs.to_dict()
        def status(self):return dict(state='idle')
        def close(self,status):pass
    class View:
        def __init__(self,*args,**kwargs):pass
        def wait_ready(self,stop):pass
        def publish(self,row):assert not closed[0]
        def events(self):return []
        def close(self,**kwargs):closed[0]=True
    import gamepad_vla.workstation.operator_view as stream
    monkeypatch.setattr(stream,'OperatorView',View)
    monkeypatch.setattr(module,'load_spec',lambda path:SimpleNamespace(left_arm=SimpleNamespace(home_q=[0]*7),right_arm=SimpleNamespace(home_q=[0]*7)))
    monkeypatch.setattr(module,'configure_runtime',lambda:None)
    monkeypatch.setattr(module,'CollectionReplay',lambda *a,**kw:Source())
    monkeypatch.setattr(module,'create_backend',lambda *a,**kw:Backend())
    monkeypatch.setattr(module,'Session',Recorder)
    controller=CollectionController(CollectionOptions('sapien','replay',tmp_path,.02,replay=tmp_path/'fake',view=True,rate_hz=50))
    assert controller.run()['ticks']==1
    assert controller.state=='stopped' and closed[0]


def test_manager_stopped_means_child_has_exited(tmp_path):
    from gamepad_vla.workstation.collection_server import SessionManager
    m=SessionManager(tmp_path)
    alive=[True]
    m.worker=SimpleNamespace(is_alive=lambda:alive[0],exitcode=0)
    m.latest=dict(session_state='stopped')
    assert m.refresh()['session_state']=='stopping'
    alive[0]=False
    assert m.refresh()['session_state']=='stopped'


def test_stop_closes_only_owned_operator_children(tmp_path):
    from gamepad_vla.workstation.operator_view import OperatorView
    import io
    calls=[]
    view=OperatorView.__new__(OperatorView)
    view.run=tmp_path;view.collection=True;view.backend=None
    view.children=[SimpleNamespace(wait=lambda **kw:calls.append('wait'),returncode=0)]
    view.logs=[];view.timing=io.StringIO();view.snapshot=SimpleNamespace(close=lambda:calls.append('close'))
    view.close(cancelled=True)
    assert (tmp_path/'stop').exists() and calls==['wait','close']


def test_episode_list_uses_actual_active_measurements(tmp_path):
    from gamepad_vla.workstation.collection_server import SessionManager
    m=SessionManager(tmp_path)
    m.output=tmp_path/'session-live'
    path=m.output/'episode-000000'
    writer=EpisodeWriter(path,dict(scene_spec_text='{}',recording_start_sim_time_s=10.,fps=50))
    m.latest=dict(recording_status=dict(episode='episode-000000',rows=100,duration_s=2.,notes=[]))
    row=m.episodes()[0]
    assert row['rows']==100 and row['duration_s']==2.
    m.latest={}
    assert m.episodes()[0]['duration_s'] is None
    writer.close()


@pytest.mark.parametrize('argv,expected', [
    (['python','/tmp/linker_hand_ros2_sdk_testdouble/sleep.py'],True),
    (['/venv/bin/python3','-u','/opt/install/rokae_driver/lib/rokae_driver/node'],True),
    (['python3.10','-m','rokae_driver.node'],True),
    (['python','-m','linker_hand_ros2_sdk.node'],True),
    (['ros2','run','rokae_driver','node'],True),
    (['python3','/opt/ros/bin/ros2','launch','linker_hand_ros2_sdk','start.py'],True),
    (['/opt/install/rokae_driver/lib/rokae_driver/node'],True),
    (['bash','-c','sleep 600 # rokae_ linker_hand_ros2_sdk'],False),
    (['grep','rokae_'],False), (['rg','linker_hand_ros2_sdk'],False),
    (['/usr/bin/cursorsandbox','--','bash','-c','rokae_driver'],False),
    (['/tmp/not_rokae_driver/node','linker_hand_ros2_sdk'],False),
    (['/tmp/not.rokae_driver/node'],False),
    (['python','/tmp/not_linker_hand_ros2_sdk/node.py'],False),
    (['python','-c',"print('rokae_driver')"],False),
    (['python','-m'],False), ([],False),
])
def test_safety_program_identity(argv,expected):
    from gamepad_vla.workstation.collection_health import hardware_process_identity
    assert bool(hardware_process_identity(argv)) is expected


def test_color_conversion_preserves_clipping_and_rounding():
    pytest.importorskip('cv2')
    from gamepad_vla.workstation.collection_camera import color_to_rgb8
    picture=np.random.default_rng(3).uniform(-.1,1.1,(48,64,4)).astype(np.float32)
    picture.reshape(-1)[:8]=[-1,0,1,2,.5,.1,127.5/255,128.5/255]
    expected=np.rint(np.clip(picture[...,:3],0.,1.)*255).astype(np.uint8)
    assert np.array_equal(color_to_rgb8(picture),expected)


def test_two_recordings_never_reuse_previous_integrity(tmp_path):
    from concurrent.futures import Future
    from gamepad_vla.workstation.collection import Session
    from gamepad_vla.workstation.collection_integrity import check_and_store,store_integrity
    s=Session.__new__(Session)
    s.output=tmp_path/'session';s.output.mkdir();s.meta=dict(scene_spec_text='{}',fps=50)
    s.writer=None;s.last_sim_time_s=0.;s.episode_index=0;s.pending_notes=[]
    s.control_log=(s.output/'events.jsonl').open('w');s.recording_state='idle';s.last_saved=None;s.on_status=lambda:None
    s.commands=dict.fromkeys(('left_arm','right_arm','left_hand','right_hand'));s.input_sequences={};s.save_future=None
    class Pool:
        future=None;path=None
        def submit(self,function,path):self.future=Future();self.path=path;return self.future
        def shutdown(self,**kwargs):pass
    pool=Pool();s.save_pool=pool
    for index in (0,1):
        s.control(dict(action='start'))
        current=s.status()['integrity'];assert current['state']=='pending' and current['episode_id'].endswith(f'{index:06d}')
        for _ in range(2):s.record(observation(s.last_sim_time_s+.02),[])
        s.writer.meta.update(received_input_messages=0,receive_start_sequence=0,receive_end_sequence=0)
        s.control(dict(action='stop'))
        store_integrity(pool.path,dict(current,state='running'));s.last_integrity_poll=0
        status=s.status()
        assert status['state']=='checking' and status['integrity']['state']=='running'
        assert status['integrity']['episode_id']==current['episode_id'] and status['integrity']['passed'] is None
        if index==1:
            assert status['last_saved']['episode_id']==current['episode_id']
        pool.future.set_result(check_and_store(pool.path));s.update_save()
        assert s.status()['integrity']['state']=='passed'
    s.close()
