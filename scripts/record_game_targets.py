#!/usr/bin/env python3
"""Record a real upstream game policy's full reports for physical-control replay.

Uses the upstream public checkpoint/environment/inference helpers. Runs a seeded
logical-time game, not a wall-clock hardware experiment. No upstream files change.
"""
import argparse
import hashlib
import importlib
import json
from pathlib import Path


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checkpoint',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--ticks',type=int,default=100)
    parser.add_argument('--seed',type=int,default=520000)
    args=parser.parse_args()
    import numpy as np
    import torch
    from dataclasses import asdict
    from resnake_gym.gamepad_runtime import make_env,stack_observations
    from resnake_gym.policy_checkpoint import load_policy_model
    from resnake_gym.policy import distribution_from_observation,reports_from_policy_action,sample_policy_action

    torch.set_num_threads(1)
    torch.manual_seed(args.seed)
    args.output.mkdir(parents=True,exist_ok=False)
    payload,config,model=load_policy_model(args.checkpoint,'cpu')
    env=make_env(config,deterministic_solver=True)
    observation,_=env.reset(seed=args.seed)
    hidden=None
    rows=[]
    metadata=dict(checkpoint_sha256=hashlib.sha256(args.checkpoint.read_bytes()).hexdigest(),
                  source_repository='https://github.com/5o1/resnake_gym',
                  seed=args.seed,logic_fps=config.logic_fps,config=asdict(config),
                  execution='logical-time upstream policy rollout; not wall-clock or hardware',
                  source_sha256={})
    for name in ('policy_checkpoint','gamepad_runtime','policy','envs.reward_gamepad','wrappers.gamepad_async'):
        path=Path(importlib.import_module('resnake_gym.'+name).__file__)
        metadata['source_sha256'][name]=hashlib.sha256(path.read_bytes()).hexdigest()
    metadata['config'].pop('resume',None)
    try:
        with torch.no_grad(), (args.output/'timeline.jsonl').open('w') as stream:
            for tick in range(args.ticks):
                distribution,_,hidden=distribution_from_observation(model,stack_observations([observation],'cpu'),hidden)
                report=reports_from_policy_action(distribution,sample_policy_action(distribution,stochastic=False),
                    continuous_buttons=getattr(config,'chunk_rho',None) is not None)[0].cpu().numpy()
                if report.shape != (1,20):
                    raise ValueError('recording currently requires a one-tick, full-20 report policy')
                row=dict(type='tick',clock_id='logical-game-replay',scheduled_ns=round(tick*1e9/config.logic_fps),
                         report=report[0].tolist())
                rows.append(row);stream.write(json.dumps(row)+'\n')
                observation,_,terminated,truncated,info=env.step(report)
                if terminated or truncated:
                    break
        metadata.update(completed_ticks=len(rows),terminated=bool(terminated),truncated=bool(truncated),
                        last_game_result={k:info[k] for k in ('score','won','logic_steps','termination_reason') if k in info},
                        timeline_sha256=hashlib.sha256((args.output/'timeline.jsonl').read_bytes()).hexdigest())
        (args.output/'provenance.json').write_text(json.dumps(metadata,indent=2)+'\n')
        values=np.array([r['report'] for r in rows])
        print(json.dumps(dict(ticks=len(rows),active_channels=np.flatnonzero(np.any(values!=0,axis=0)).tolist(),
                              timeline_sha256=metadata['timeline_sha256'],game=metadata['last_game_result'])),flush=True)
    finally:
        env.close()


if __name__=='__main__':
    main()
