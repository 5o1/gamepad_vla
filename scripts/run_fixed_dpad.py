#!/usr/bin/env python3
"""Evaluate IK, train a residual PPO, or evaluate a saved policy (simulation only)."""
import argparse
import importlib.metadata
import json
from functools import partial
from pathlib import Path

import numpy as np

from gamepad_vla.provenance import code_provenance
from gamepad_vla.tasks.fixed_gamepad import load_task
from gamepad_vla.kinematics import SerialChain
from gamepad_vla.workstation.spec import pose_to_matrix
from gamepad_vla.tasks.dpad_env import FixedDpadEnv, score_rollout
from gamepad_vla.targets import load_timeline


def evaluate(env, output, seeds, model=None):
    scores = []
    for seed in seeds:
        observation, reset_info = env.reset(seed=seed)
        rows = []
        with (output/f'trace_seed{seed}.jsonl').open('w') as stream:
            while True:
                action = np.zeros(4) if model is None else model.predict(observation, deterministic=True)[0]
                observation, reward, terminated, truncated, info = env.step(action)
                row = dict(info, reward=reward, residual=np.asarray(action).tolist())
                rows.append(row)
                stream.write(json.dumps(row, allow_nan=False)+'\n')
                if terminated or truncated:
                    break
        scores.append(dict(seed=seed, **reset_info, **score_rollout(rows, env.config['digital_threshold'])))
        (output/'evaluation.json').write_text(json.dumps(scores,indent=2,allow_nan=False)+'\n')
        print(json.dumps(scores[-1]),flush=True)
    return scores


def make_training_env(config, sequence, output, rank):
    from stable_baselines3.common.monitor import Monitor
    env = FixedDpadEnv(config, sequence=sequence, work_dir=output/f'model_env{rank}')
    return Monitor(env, str(output/f'env{rank}.monitor.csv'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['ik','train','eval','inspect'])
    parser.add_argument('--config', type=Path, default=Path('configs/tasks/fixed_dpad_causal.json'))
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--timeline', type=Path)
    parser.add_argument('--seed',type=int,default=0)
    parser.add_argument('--eval-seeds',type=int,nargs='+',default=[100,101,102])
    parser.add_argument('--steps',type=int,default=100000)
    parser.add_argument('--checkpoint',type=Path)
    parser.add_argument('--device',default='cpu')
    parser.add_argument('--torch-threads',type=int,default=1)
    parser.add_argument('--num-envs',type=int,default=1)
    args = parser.parse_args()
    if args.num_envs < 1:
        parser.error("--num-envs must be positive")
    args.output.mkdir(parents=True,exist_ok=False)
    config = load_task(args.config)
    sequence = load_timeline(args.timeline) if args.timeline else None
    env = FixedDpadEnv(config,sequence=sequence,work_dir=args.output/'model')
    run = dict(mode=args.mode, seed=args.seed, config=config, num_envs=args.num_envs,
               requested_steps=args.steps,device=args.device,torch_threads=args.torch_threads,
               provenance=code_provenance(Path(__file__).resolve().parents[1]),
               environment={n:importlib.metadata.version(n) for n in ['mujoco','numpy','gymnasium','stable-baselines3','torch']},
               scope='Simulation contact control; estimated hand drives and asset contact parameters, no hardware validation.')
    (args.output/'run.json').write_text(json.dumps(run,indent=2)+'\n')
    try:
        if args.mode == 'inspect':
            env.reset(seed=args.seed)
            p = env.physics
            arm = SerialChain(p.workcell.left_arm.urdf, p.workcell.left_arm.tip_link)
            q = dict(zip(p.workcell.left_arm.joint_names, p.workcell.left_arm.home_q))
            predicted = (pose_to_matrix(p.workcell.left_arm.base_pose) @
                         arm.forward([q[n] for n in arm.active])[0] @
                         pose_to_matrix(p.workcell.left_hand.mount))
            conversion_error = float(np.max(np.abs(predicted-p.hand_pose())))
            if conversion_error > 1e-5:
                raise ValueError(f'arm welding changed the hand frame: {conversion_error}')
            result = dict(arm_conversion_max_abs_error=conversion_error, ik_error_m=env.physics.ik.errors,
                          hand_world=env.physics.hand_pose().tolist(),
                          gamepad_pose=env.physics.workcell.gamepad.root_pose,
                          qpos=env.physics.data.qpos.tolist(),
                          ncon=int(env.physics.data.ncon),
                          fingertip_world=env.physics.fingertip().tolist(),
                          gamepad_findings=[str(x) for x in env.physics.gamepad.findings])
            (args.output/'inspection.json').write_text(json.dumps(result,indent=2)+'\n')
            print(json.dumps(result),flush=True)
        elif args.mode == 'ik':
            evaluate(env,args.output,args.eval_seeds)
        else:
            import torch
            torch.set_num_threads(args.torch_threads)
            from stable_baselines3 import PPO
            if args.mode == 'eval':
                if not args.checkpoint:
                    parser.error('eval requires --checkpoint')
                model = PPO.load(args.checkpoint,env=env,device=args.device)
            else:
                from stable_baselines3.common.callbacks import CheckpointCallback
                from stable_baselines3.common.monitor import Monitor
                from stable_baselines3.common.env_checker import check_env
                check_env(env, warn=True)
                if args.num_envs == 1:
                    monitored = Monitor(env,str(args.output/'monitor.csv'))
                else:
                    from stable_baselines3.common.vec_env import SubprocVecEnv
                    monitored = SubprocVecEnv([partial(make_training_env,config,sequence,args.output,rank)
                                              for rank in range(args.num_envs)], start_method='spawn')
                model = (PPO.load(args.checkpoint,env=monitored,device=args.device) if args.checkpoint else
                         PPO('MlpPolicy',monitored,n_steps=512,batch_size=256,n_epochs=10,
                             learning_rate=3e-4,seed=args.seed,device=args.device,verbose=1,
                             policy_kwargs={'net_arch':dict(pi=[256,256],vf=[256,256])}))
                try:
                    model.learn(total_timesteps=args.steps,callback=CheckpointCallback(
                        save_freq=max(1,2048//args.num_envs),save_path=str(args.output/'checkpoints')),reset_num_timesteps=not bool(args.checkpoint))
                finally:
                    model.save(args.output/'policy_latest')
                    if args.num_envs > 1:
                        monitored.close()
                model.save(args.output/'policy')
                run['completed_training_steps'] = int(model.num_timesteps)
                (args.output/'run.json').write_text(json.dumps(run,indent=2)+'\n')
            evaluate(env,args.output,args.eval_seeds,model)
    finally:
        env.close()


if __name__ == '__main__':
    main()
