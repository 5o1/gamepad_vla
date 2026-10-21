#!/usr/bin/env python3
"""Recalculate archived D-pad aggregate and per-direction equality, without training."""
import argparse
import json
from pathlib import Path
import statistics


def audit(path):
    data=json.loads(Path(path).read_text())
    ik={e['seed']:e for e in data['ik_baseline']}
    means=[];comparisons=[]
    for train in data['training']:
        means.append(statistics.mean(e['f1'] for e in train['evaluations']))
        for e in train['evaluations']:
            baseline=ik[e['seed']]
            per={name:{k:e['per_direction'][name][k]==base[k] for k in ('tp','fp','fn','f1')}
                 for name,base in baseline['per_direction'].items()}
            comparisons.append(dict(training_seed=train['seed'],target_seed=e['seed'],per_direction_equal=per,
                                    f1_equal=e['f1']==baseline['f1'],
                                    physical_force_equal=e['max_contact_force_n']==baseline['max_contact_force_n']))
    return dict(ik_mean=statistics.mean(e['f1'] for e in ik.values()),ppo_seed_means=means,
                ppo_mean=statistics.mean(means),ppo_sample_std=statistics.stdev(means),
                per_direction_comparisons=comparisons,
                game_ticks=sum(e['ticks_completed'] for e in data['game_cases']),
                game_exact=sum(e['exact_reports'] for e in data['game_cases']),
                interpretation='Equal aggregate counts/F1 alone do not establish equal per-step digital outputs or physical trajectories.')


def audit_traces(ik_dir, ppo_dirs, seeds=(300,301,302)):
    rows=[]
    for training_seed,directory in enumerate(ppo_dirs):
        for seed in seeds:
            base=[json.loads(line) for line in (ik_dir/f'trace_seed{seed}.jsonl').read_text().splitlines()]
            learned=[json.loads(line) for line in (directory/f'trace_seed{seed}.jsonl').read_text().splitlines()]
            if len(base)!=len(learned):
                raise ValueError('paired trace lengths differ')
            same_target=all(a['target']==b['target'] and a['time_s']==b['time_s'] for a,b in zip(base,learned))
            if not same_target:raise ValueError('paired targets or times differ')
            rows.append(dict(training_seed=training_seed,target_seed=seed,steps=len(base),same_targets_and_times=same_target,
                binary_dpad_diff_steps=sum(any((x>=.3)!=(y>=.3) for x,y in zip(a['dpad'],b['dpad'])) for a,b in zip(base,learned)),
                raw32_diff_steps=sum(a['gamepad_signal']!=b['gamepad_signal'] for a,b in zip(base,learned)),
                contact_force_diff_steps=sum(a['max_contact_force_n']!=b['max_contact_force_n'] for a,b in zip(base,learned)),
                maximum_raw_dpad_difference=max(abs(x-y) for a,b in zip(base,learned) for x,y in zip(a['dpad'],b['dpad'])),
                nonzero_residual_steps=sum(any(x!=0 for x in b['residual']) for b in learned)))
    return rows


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input',type=Path,default=Path('docs/reports/2026-10-21-fixed-gamepad/fixed_phase_result.json'))
    p.add_argument('--output',type=Path)
    p.add_argument('--ik-traces',type=Path)
    p.add_argument('--ppo-traces',type=Path,nargs=3,metavar=('SEED0','SEED1','SEED2'))
    a=p.parse_args();result=audit(a.input)
    if a.ik_traces and a.ppo_traces:
        result['paired_traces']=audit_traces(a.ik_traces,a.ppo_traces)
    text=json.dumps(result,indent=2)+'\n'
    if a.output:
        a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(text)
    else:print(text,end='')


if __name__=='__main__':main()
