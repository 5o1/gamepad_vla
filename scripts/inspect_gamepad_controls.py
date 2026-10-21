#!/usr/bin/env python3
"""Extract motion ranges and static return loads from the pinned prepared asset."""
import argparse
import json
from pathlib import Path

from gamepad_vla.tasks.fixed_gamepad import load_task


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--task',type=Path,default=Path('configs/tasks/fixed_dpad_causal.json'))
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();cfg=load_task(args.task);root=Path(cfg['resource_root'])
    motion=json.loads((root/'common/xbox_one_s/motion/full_controls.json').read_text())
    physical=json.loads((root/'common/xbox_one_s/physical/full_controls.json').read_text())
    rows=[]
    for joint in motion['joints']:
        name=joint['name'];law={k:v.get('value') if isinstance(v,dict) else v for k,v in physical['return_laws'][name].items()}
        analog=('_stick_' in name and not name.endswith('click')) or name.endswith('hinge')
        fraction=.8 if analog else cfg['digital_threshold']
        q=joint['upper']*fraction
        force=min(law['effort_limit'],abs(law['stiffness']*(q-law['neutral_target'])))
        rows.append(dict(joint=joint,return_law=law,probe_fraction=fraction,probe_q=q,
                         static_return_load=force,load_unit='N' if joint['unit']=='m' else 'N m',
                         threshold_status='analog 80% feasibility target' if analog else 'task 0.3 trial threshold; not measured switch closure'))
    out=dict(profile='full_controls',signal_layout=motion['signal_policy']['layout'],
             signal_policy=motion['signal_policy'],bindings=motion['bindings'],controls=rows,
             scope='Source parameter extraction and zero-speed spring-load calculation; no contact simulation or hardware measurement.')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n')


if __name__=='__main__':main()
