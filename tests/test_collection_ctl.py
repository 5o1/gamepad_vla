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
