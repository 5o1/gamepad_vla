#!/usr/bin/env python3
"""Summarize Task J's fixed replay window on the GPU node (same monotonic clock)."""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from gamepad_vla.workstation.g20_mapping import G20_JOINT_NAMES
from gamepad_vla.workstation.recorder import read_episode


def rows(path):
    if not Path(path).is_file():
        raise ValueError(f'{path} is absent. Start the benchmark session with --diagnostics; production sessions do not capture timing.')
    return [json.loads(line) for line in Path(path).read_text().splitlines()]


def percentiles(values):
    a = np.asarray(values)
    return dict(n=len(a), p50=None if not len(a) else float(np.percentile(a, 50)),
                p95=None if not len(a) else float(np.percentile(a, 95)))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run', type=Path, required=True)
    p.add_argument('--episode', type=Path, required=True)
    p.add_argument('--sender-log', type=Path, required=True)
    p.add_argument('--workstation-log', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    sends = rows(a.sender_log)
    # Fixed before looking at outcomes: 10 s warmup, next 300 complete seconds.
    start = sends[0]['stamp'] + 10
    end = start + 300
    physics = rows(a.run/'physics_timing.jsonl')
    stream = rows(a.run/'stream_timing.jsonl')
    commands = [r for r in physics if r['kind']=='command' and start<=r['applied_ns']/1e9<end]
    frames = [r for r in stream if start<=r['encoded_ns']/1e9<end]
    by_id = {r['command_id']: r for r in commands}
    first = {}
    for r in stream:
        if r['command_id'] in by_id:
            first.setdefault(r['command_id'], r['encoded_ns'])
    fps = np.histogram([r['encoded_ns']/1e9 for r in frames], bins=start+np.arange(301))[0]
    stamp_to_send = {r['stamp']:r for r in sends}
    send_apply = [(r['applied_ns']/1e9-stamp_to_send[s]['stamp'])*1000
                  for r in commands for s in r['stamps'] if s in stamp_to_send]
    apply_encode = [(first[k]-by_id[k]['applied_ns'])/1e6 for k in first]
    meta, arrays = read_episode(a.episode)
    commanded = np.full_like(arrays['hand_q'], np.nan)
    stamps = np.full(len(commanded), np.nan)
    for r in rows(a.episode/'commands.jsonl'):
        if r['kind']=='hand' and r['side']=='left':
            tick = int(round(r['t']*meta['rates']['loop_hz']))
            commanded[tick] = [r['joint_positions'][n] for n in G20_JOINT_NAMES]
            stamps[tick] = r['stamp']
    root = ET.parse(meta['spec']['left_hand']['urdf']).getroot()
    limits = {j.get('name'):(float(j.find('limit').get('lower')),float(j.find('limit').get('upper')))
              for j in root.findall('joint') if j.find('limit') is not None}
    low = np.array([limits[n][0] for n in G20_JOINT_NAMES])
    high = np.array([limits[n][1] for n in G20_JOINT_NAMES])
    # Exclude any target or observed joint within .02 rad of a URDF stop.
    actual = arrays['hand_q']
    interior = (commanded>low+.02)&(commanded<high-.02)&(actual>low+.02)&(actual<high-.02)
    # Steady: target range <= .02 rad over preceding .1 s (7 ticks at 60 Hz).
    steady = np.zeros_like(interior)
    for i in range(6,len(commanded)):
        steady[i] = np.ptp(commanded[i-6:i+1],axis=0)<=.02
    window = (stamps>=start)&(stamps<end)
    eligible = interior & steady & window[:,None]
    errors = np.abs(actual-commanded)
    final = next(json.loads(line) for line in reversed(a.workstation_log.read_text().splitlines())
                 if line.startswith('{"ticks":'))
    result = dict(window=dict(start_monotonic_s=start,end_monotonic_s=end,duration_s=300),
        sender=dict(packets=len(sends), duration_s=sends[-1]['stamp']-sends[0]['stamp']),
        workstation=final, send_to_apply_ms=percentiles(send_apply),
        apply_to_first_encode_ms=percentiles(apply_encode),
        applied_commands=len(commands), encoded_commands=len(first),
        applied_commands_without_frame=len(commands)-len(first),
        frame_state_apply_to_encode_ms=percentiles([(r['encoded_ns']-r['applied_ns'])/1e6 for r in frames]),
        encoded_fps=dict(mean=float(fps.mean()),minimum=int(fps.min()),seconds=fps.tolist()),
        tracking=dict(total_joint_samples=int(window.sum()*21),
            limit_excluded=int((window[:,None]&~interior).sum()),
            interior_nonsteady=int((window[:,None]&interior&~steady).sum()),
            eligible=int(eligible.sum()),error_rad=percentiles(errors[eligible]),
            all_interior_error_rad=percentiles(errors[window[:,None]&interior]),
            per_joint={n:percentiles(errors[:,i][eligible[:,i]]) for i,n in enumerate(G20_JOINT_NAMES)}),
        arm_max_error_rad=float(np.max(np.abs(arrays['arm_q']-meta['spec']['left_arm']['home_q']))),
        episode=dict(path=str(a.episode),read_episode_rows=len(arrays['t']),finite_hand=bool(np.isfinite(actual).all())))
    result['passed'] = bool(result['sender']['duration_s']>=300 and final['decode_errors']==0
        and final['rejected_packets']==0 and len(frames)>0 and fps.min()>=30
        and eligible.sum()>0 and np.median(errors[eligible])<=.05
        and len(send_apply)>0 and len(apply_encode)>0 and result['episode']['finite_hand'])
    a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result))
    if not result['passed']:
        raise SystemExit(1)


if __name__=='__main__':
    main()
