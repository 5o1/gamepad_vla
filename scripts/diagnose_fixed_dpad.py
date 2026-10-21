#!/usr/bin/env python3
"""Record real thumb tracking, mimic error and contact identities during fixed holds."""
import argparse
import json
from pathlib import Path

import numpy as np

from gamepad_vla.tasks.fixed_gamepad import FixedGamepadPhysics, load_task, THUMB_JOINTS
from gamepad_vla.tasks.dpad_env import DPAD_INDICES
from gamepad_vla.workstation.g20_mapping import G20_JOINT_NAMES


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=Path('configs/tasks/fixed_dpad.json'))
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--hold-seconds',type=float,default=1.)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    config=load_task(args.config)
    physics=FixedGamepadPhysics(config,work_dir=args.output/'model')
    thumb=[G20_JOINT_NAMES.index(n) for n in THUMB_JOINTS]
    ip=G20_JOINT_NAMES.index('thumb_ip')
    summaries=[]
    try:
        for label in range(1,5):
            physics.reset()
            for _ in range(round(.5/config['control_timestep_s'])):
                physics.step(physics.ik.commands[0])
            rows=[]
            with (args.output/f'hold_{label}.jsonl').open('w') as stream:
                for step in range(round(args.hold_seconds/config['control_timestep_s'])):
                    contact=physics.step(physics.ik.commands[label])
                    q=physics.data.qpos[physics.qidx]
                    pairs=[]
                    for c in physics.data.contact:
                        ids=[int(physics.model.geom_bodyid[g]) for g in c.geom]
                        if set(ids)&physics.robot_bodies and set(ids)&physics.gamepad_bodies:
                            pairs.append([physics.model.body(i).name for i in ids])
                    row=dict(time_s=(step+1)*config['control_timestep_s'],dpad=np.asarray(physics.gamepad.gamepad_state().values)[DPAD_INDICES].tolist(),
                             thumb_target=physics.ik.commands[label].tolist(),thumb_q=q[thumb].tolist(),
                             thumb_mimic_error_rad=float(q[ip]-1.02*q[thumb[-1]]),
                             fingertip_error_m=float(np.linalg.norm(physics.fingertip()-physics.ik.goal_points_world[label])),
                             contact_pairs=pairs,**contact)
                    rows.append(row);stream.write(json.dumps(row)+'\n')
            steady=rows[len(rows)//2:]
            result=dict(label=label,mean_dpad=np.mean([r['dpad'] for r in steady],axis=0).tolist(),
                        mean_abs_tracking_error_rad=np.mean([np.abs(np.array(r['thumb_q'])-r['thumb_target']) for r in steady],axis=0).tolist(),
                        max_abs_mimic_error_rad=max(abs(r['thumb_mimic_error_rad']) for r in rows),
                        mean_tip_error_mm=float(np.mean([r['fingertip_error_m'] for r in steady])*1000),
                        contact_pairs=sorted({tuple(pair) for r in steady for pair in r['contact_pairs']}),
                        peak_contact_force_n=max(r['max_contact_force_n'] for r in rows))
            summaries.append(result)
            (args.output/'summary.json').write_text(json.dumps(summaries,indent=2)+'\n')
            print(json.dumps(result),flush=True)
    finally:
        physics.close()


if __name__=='__main__':
    main()
