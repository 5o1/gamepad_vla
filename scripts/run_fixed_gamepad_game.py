#!/usr/bin/env python3
"""Logical-time game/hand loop with separate upstream and simulation interpreters."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import time

from gamepad_vla.provenance import code_provenance


def board_state(game):
    state=game.unwrapped.state
    return dict(width=state.width,height=state.height,snake=state.snake,
                food=state.food,score=state.score,direction=state.direction)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--game-checkpoint',type=Path,required=True)
    parser.add_argument('--controller-checkpoint',type=Path)
    parser.add_argument('--controller-python',required=True)
    parser.add_argument('--config',type=Path,default=Path('configs/tasks/fixed_dpad_causal.json'))
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--ticks',type=int,default=100)
    parser.add_argument('--record-state',action='store_true')
    parser.add_argument('--seed',type=int,default=520000)
    args=parser.parse_args()
    import numpy as np
    import torch
    from resnake_gym.gamepad_runtime import make_env,stack_observations
    from resnake_gym.policy_checkpoint import load_policy_model
    from resnake_gym.policy import distribution_from_observation,reports_from_policy_action,sample_policy_action

    torch.set_num_threads(1)
    torch.manual_seed(args.seed)
    args.output.mkdir(parents=True,exist_ok=False)
    _,config,game_policy=load_policy_model(args.game_checkpoint,'cpu')
    game=make_env(config,deterministic_solver=True)
    observation,_=game.reset(seed=args.seed)
    command=[args.controller_python,str(Path(__file__).with_name('fixed_dpad_worker.py')),
             '--config',str(args.config.resolve()),'--output',str(args.output/'controller'),
             '--ticks',str(args.ticks),'--logic-fps',str(config.logic_fps),
             '--sdl-y-down-positive','no']
    if args.record_state:
        command+=['--record-state']
    if args.controller_checkpoint:
        command+=['--checkpoint',str(args.controller_checkpoint.resolve())]
    hidden=None
    metadata=dict(seed=args.seed,ticks_requested=args.ticks,logic_fps=config.logic_fps,
                  source=code_provenance(Path(__file__).resolve().parents[1]),
                  controller_config=json.loads(args.config.read_text()),
                  game_checkpoint_sha256=hashlib.sha256(args.game_checkpoint.read_bytes()).hexdigest(),
                  controller='ppo' if args.controller_checkpoint else 'ik',
                  timing='logical-time coupled simulation; not wall-clock execution',
                  future_targets='only latest received command is held; no future game decisions')
    if args.controller_checkpoint:
        metadata['controller_checkpoint_sha256']=hashlib.sha256(args.controller_checkpoint.read_bytes()).hexdigest()
    if args.record_state:
        metadata['initial_game_state']=board_state(game)
    worker=None
    try:
        with (args.output/'worker.log').open('w') as log, (args.output/'game_trace.jsonl').open('w') as trace, (args.output/'control_trace.jsonl').open('w') as controls:
            worker=subprocess.Popen(command,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,text=True,bufsize=1)
            ready=json.loads(worker.stdout.readline())
            metadata['controller_reset'] = ready.get('reset')
            if ready.get('type') != 'ready':
                raise RuntimeError('simulation worker did not become ready')
            started=time.perf_counter()
            with torch.no_grad():
                for tick in range(args.ticks):
                    distribution,_,hidden=distribution_from_observation(game_policy,stack_observations([observation],'cpu'),hidden)
                    requested=reports_from_policy_action(distribution,sample_policy_action(distribution,stochastic=False),
                        continuous_buttons=getattr(config,'chunk_rho',None) is not None)[0].cpu().numpy()
                    if requested.shape != (1,20):
                        raise ValueError('closed loop requires a one-tick full-20 game policy')
                    worker.stdin.write(json.dumps(dict(report=requested[0].tolist()))+'\n');worker.stdin.flush()
                    line=worker.stdout.readline()
                    if not line:
                        raise RuntimeError('simulation worker stopped; inspect worker.log')
                    physical=json.loads(line)
                    actual=np.asarray(physical['report'],dtype=np.float32)
                    observation,_,terminated,truncated,info=game.step(actual[None])
                    row=dict(tick=tick,requested=requested[0].tolist(),executed=actual.tolist(),
                             game={k:info[k] for k in ('score','won','logic_steps','termination_reason') if k in info})
                    if args.record_state:
                        row['game_state']=board_state(game)
                    trace.write(json.dumps(row)+'\n');trace.flush()
                    for control in physical['control_rows']:
                        controls.write(json.dumps(control)+'\n')
                    controls.flush()
                    if terminated or truncated:
                        break
            worker.stdin.close()
            if worker.wait()!=0:
                raise RuntimeError('simulation worker failed; inspect worker.log')
            metadata.update(ticks_completed=tick+1,terminated=bool(terminated),truncated=bool(truncated),
                            last_game_result=row['game'],wall_seconds=time.perf_counter()-started,
                            simulated_seconds=(tick+1)/config.logic_fps)
            (args.output/'summary.json').write_text(json.dumps(metadata,indent=2)+'\n')
            print(json.dumps(metadata),flush=True)
    finally:
        if worker is not None and worker.poll() is None:
            worker.terminate();worker.wait()
        game.close()


if __name__=='__main__':
    main()
