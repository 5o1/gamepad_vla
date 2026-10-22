"""Strict saved-episode validation and non-destructive discard sidecars."""
import json
from pathlib import Path
import time

from .episode_schema import read_episode, read_input_events


def check_integrity(path):
    path = Path(path)
    result = dict(episode_id=episode_id(path), state='running', passed=False, checked_wall_time_ns=time.time_ns(), checks={}, errors=[])
    try:
        meta, rows = read_episode(path)
        events = read_input_events(path, meta=meta)
        result['checks']['strict_readback'] = True
        result['rows'] = len(rows)
        if meta['status'] != 'complete' or not rows:
            raise ValueError('episode is incomplete or empty')
        seq = [e['sequence'] for e in events]
        if any(b != a+1 for a, b in zip(seq, seq[1:])):
            raise ValueError('input sequence has a gap')
        received = meta.get('received_input_messages')
        if received is None:
            raise ValueError('no independent receiver boundary count (legacy episode)')
        if received != len(events):
            raise ValueError('command journal count differs from receiver count')
        if events and (seq[0] != meta['receive_start_sequence'] or seq[-1]+1 != meta['receive_end_sequence']):
            raise ValueError('input sequence differs from receiver boundaries')
        result['checks']['receiver_count_and_sequences'] = True
        fps = float(meta['fps'])
        start = float(meta['recording_start_sim_time_s'])
        end = float(meta['recording_end_sim_time_s'])
        dt = max(float(meta.get('physics_timestep_s', 0)), 1e-6)
        expected = len(rows)/fps
        if abs(end-start-expected) > dt+1e-6:
            raise ValueError('row count and recorded duration differ')
        if abs(rows[-1]['sim_time_s']-end) > 1e-6 or abs(rows[0]['sim_time_s']-start-1/fps) > dt+1e-6:
            raise ValueError('row time boundaries differ')
        if any(abs(b['sim_time_s']-a['sim_time_s']-1/fps)>dt+1e-6 for a,b in zip(rows,rows[1:])):
            raise ValueError('row clock has a gap')
        result['duration_s'] = end-start
        result['input_messages'] = len(events)
        result['checks']['row_duration'] = True
        result['passed'] = True
    except (OSError, ValueError, KeyError, TypeError, ZeroDivisionError) as exc:
        result['errors'].append(str(exc))
    result['state'] = 'passed' if result['passed'] else 'failed'
    return result


def discard_episode(path, note=''):
    path = Path(path)
    if not (path/'meta.json').is_file():
        raise ValueError('episode not found')
    marker = dict(discarded=True, wall_time_ns=time.time_ns(), note=str(note))
    (path/'discarded.json').write_text(json.dumps(marker, ensure_ascii=False, indent=2)+'\n')
    return marker



def episode_id(path):
    path=Path(path)
    return path.parent.name+'~'+path.name


def pending_integrity(path, phase='saving'):
    return dict(episode_id=episode_id(path),state='pending',phase=phase,passed=None,checks={},errors=[])


def store_integrity(path, result):
    path=Path(path)
    temporary=path/'integrity.json.tmp'
    temporary.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    temporary.replace(path/'integrity.json')


def read_integrity(path):
    path=Path(path)
    file=path/'integrity.json'
    if not file.exists():return pending_integrity(path,'awaiting_check')
    result=json.loads(file.read_text())
    # Legacy sidecars remain readable; bind them to their actual directory.
    result.setdefault('episode_id',episode_id(path))
    result.setdefault('state','passed' if result.get('passed') is True else 'failed' if result.get('passed') is False else 'pending')
    return result


def check_and_store(path):
    """CPU worker entrypoint: never imports or constructs a simulator."""
    path=Path(path)
    running=dict(pending_integrity(path,'checking'),state='running')
    store_integrity(path,running)
    result=check_integrity(path)
    store_integrity(path,result)
    return result
