"""Standard-library client's refusal codes and integrity wait semantics."""
from pathlib import Path
import runpy
from urllib.error import URLError


def module():
    return runpy.run_path(Path(__file__).parents[1]/'scripts/collection_ctl.py')


def test_wait_includes_controller_save_completion(monkeypatch,capsys):
    m=module();calls=[]
    def request(base,path,data=None):
        calls.append(path)
        if path=='recording/stop':return 200,dict(episode_id='session~episode')
        saving=calls.count(path)==1
        return 200,dict(id='session~episode',integrity=dict(episode_id='session~episode',state='running' if saving else 'passed',passed=None if saving else True),state=dict(recording_status=dict(state='checking' if saving else 'saved')))
    monkeypatch.setitem(m['main'].__globals__,'request',request)
    monkeypatch.setattr(m['time'],'sleep',lambda _:None)
    assert m['main'](['record-stop','--wait'])==0
    assert calls.count('episodes/session~episode')==2
    assert 'integrity: passed' in capsys.readouterr().out


def test_refusal_and_unreachable_codes(monkeypatch,capsys):
    m=module()
    monkeypatch.setitem(m['main'].__globals__,'request',lambda *a:(403,dict(message='Ask the station operator to enable remote control.')))
    assert m['main'](['record-start'])==2
    def unavailable(*args):raise URLError('offline')
    monkeypatch.setitem(m['main'].__globals__,'request',unavailable)
    assert m['main'](['status'])==3
    assert 'Service unreachable' in capsys.readouterr().out


def test_wait_rejects_previous_episode_and_pending_states(monkeypatch,capsys):
    m=module();answers=iter([
        dict(episode_id='session~first',state='passed',passed=True),
        None,
        dict(episode_id='session~second',state='pending',passed=None),
        dict(episode_id='session~second',state='running',passed=None),
        dict(episode_id='session~second',state='passed',passed=True),
    ]);observed=[]
    def request(base,path,data=None):
        if path=='recording/stop':return 200,dict(episode_id='session~second')
        observed.append(path)
        return 200,dict(id='session~second',integrity=next(answers))
    monkeypatch.setitem(m['main'].__globals__,'request',request)
    monkeypatch.setattr(m['time'],'sleep',lambda _:None)
    assert m['main'](['record-stop','--wait'])==0
    assert observed==['episodes/session~second']*5
    assert 'session~second' in capsys.readouterr().out


def test_wait_timeout_is_failure_not_old_success(monkeypatch,capsys):
    m=module()
    def request(base,path,data=None):
        if path=='recording/stop':return 200,dict(episode_id='session~second')
        return 200,dict(integrity=dict(episode_id='session~first',state='passed',passed=True))
    monkeypatch.setitem(m['main'].__globals__,'request',request)
    assert m['main'](['record-stop','--wait','--wait-timeout','0'])==1
    assert 'timed out' in capsys.readouterr().out


def test_wait_never_reports_passed_while_same_episode_status_checks(monkeypatch,capsys):
    m=module();calls=[]
    def request(base,path,data=None):
        if path=='recording/stop':return 200,dict(episode_id='session~second')
        calls.append(path)
        state='running' if len(calls)==1 else 'passed'
        return 200,dict(id='session~second',integrity=dict(episode_id='session~second',state='passed',passed=True),
                        state=dict(integrity=dict(episode_id='session~second',state=state)))
    monkeypatch.setitem(m['main'].__globals__,'request',request)
    monkeypatch.setattr(m['time'],'sleep',lambda _:None)
    assert m['main'](['record-stop','--wait'])==0 and len(calls)==2


def test_snapshot_commands_use_shared_api(monkeypatch,capsys):
    m=module();calls=[]
    def request(base,path,data=None):
        calls.append((path,data))
        if path=='snapshots':return 200,[dict(name='2026-11-05/071500_reference')]
        if path.startswith('snapshots/'):return 200,dict(name='2026-11-05/071500_reference',files=['camera_3.png'])
        return 200,dict(snapshot=dict(state='countdown' if path=='snapshot' else 'cancelled',remaining_s=30))
    monkeypatch.setitem(m['main'].__globals__,'request',request)
    assert m['main'](['snapshot','--label','reference'])==0
    assert calls[-1]==('snapshot',dict(delay_s=None,label='reference'))
    assert m['main'](['snapshot','--delay-s','5'])==0 and calls[-1][1]['delay_s']==5
    assert m['main'](['snapshot-cancel'])==0 and calls[-1]==('snapshot/cancel',{})
    assert m['main'](['snapshots'])==0
    assert m['main'](['snapshots','2026-11-05/071500_reference'])==0
    assert calls[-1]==('snapshots/2026-11-05/071500_reference',None)


def test_snapshot_get_downloads_original_files(monkeypatch,tmp_path,capsys):
    import io
    m=module();received=[]
    monkeypatch.setitem(m['main'].__globals__,'request',lambda *args:(200,dict(name='2026-11-05/reference',files=['camera_3.png','meta.json'])))
    def open_url(request,timeout):
        received.append(request.full_url)
        return io.BytesIO(b'original bytes')
    monkeypatch.setitem(m['main'].__globals__,'urlopen',open_url)
    output=tmp_path/'reference'
    assert m['main'](['snapshot-get','2026-11-05/reference','--output',str(output)])==0
    assert (output/'camera_3.png').read_bytes()==(output/'meta.json').read_bytes()==b'original bytes'
    assert len(received)==2 and received[0].endswith('/snapshots/2026-11-05/reference/camera_3.png')
    assert m['main'](['snapshot-get','2026-11-05/reference','--output',str(output)])==1
    assert len(received)==2 and (output/'camera_3.png').read_bytes()==b'original bytes'
    assert 'Snapshot download failed' in capsys.readouterr().out
