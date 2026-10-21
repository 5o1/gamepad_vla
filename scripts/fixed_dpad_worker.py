#!/usr/bin/env python3
"""JSON-lines simulation worker: desired full reports in, physical full reports out."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np

from gamepad_vla.signals import sdl3_to_xinput
from gamepad_vla.targets import TargetSequence
from gamepad_vla.tasks.fixed_gamepad import load_task
from gamepad_vla.tasks.dpad_env import FixedDpadEnv


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--logic-fps',type=float,required=True)
    parser.add_argument('--ticks',type=int,required=True)
    parser.add_argument('--checkpoint',type=Path)
    parser.add_argument('--record-state',action='store_true')
    parser.add_argument('--sdl-y-down-positive',choices=['yes','no'],required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    config=load_task(args.config)
    count=round(1/(args.logic_fps*config['control_timestep_s']))
    if not np.isclose(count*config['control_timestep_s'],1/args.logic_fps):
        raise ValueError('game tick must contain an integer number of control steps')
    config['episode_seconds']=args.ticks/args.logic_fps
    sequence=TargetSequence(np.array([0.]),np.zeros((1,20),dtype=np.float32),'causal-game')
    env=FixedDpadEnv(config,sequence=sequence,work_dir=args.output/'model')
    model=None
    if args.checkpoint:
        import torch
        torch.set_num_threads(1)
        from stable_baselines3 import PPO
        model=PPO.load(args.checkpoint,device='cpu')
    try:
        observation,reset_info=env.reset(seed=0)
        print(json.dumps(dict(type='ready',control_steps_per_tick=count,reset=reset_info)),flush=True)
        for line in sys.stdin:
            request=json.loads(line)
            report=np.asarray(request['report'],dtype=np.float32)
            observation=env.set_target(report)
            rows=[]
            for _ in range(count):
                action=np.zeros(4) if model is None else model.predict(observation,deterministic=True)[0]
                observation,reward,_,_,info=env.step(action)
                row=dict(info,reward=reward,residual=np.asarray(action).tolist())
                if args.record_state:
                    row.update(qpos=env.physics.data.qpos.tolist(),qvel=env.physics.data.qvel.tolist(),
                               physics_time_s=float(env.physics.data.time))
                rows.append(row)
            actual=sdl3_to_xinput(rows[-1]['gamepad_signal'],
                sdl_y_down_positive=args.sdl_y_down_positive=='yes',digital_threshold=config['digital_threshold'])
            print(json.dumps(dict(type='result',report=actual.tolist(),control_rows=rows)),flush=True)
    finally:
        env.close()


if __name__=='__main__':
    main()
