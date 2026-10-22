#!/usr/bin/env python3
"""Replay the recovered fixed-placement success, including all native controls."""
from __future__ import annotations
import argparse,json,os,time
from pathlib import Path
from collections import Counter
import numpy as np
from k1_height_common import write_json
from k1b_search import Problem,cpu_seconds
from k1b_native import Native


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--root',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--task',type=Path,default=Path('configs/tasks/fixed_dpad_causal.json'))
    args=ap.parse_args();args.output.mkdir(parents=True,exist_ok=True);start=cpu_seconds()
    probe=json.loads((args.root/'common-pose-probe/result.json').read_text())
    successes=probe['common_successes']
    result=dict(source='common-pose-probe/result.json',source_attempts=len(probe['attempts']),source_successes=len(successes),candidates=[])
    for index,source in enumerate(successes):
        row=dict(source_pair_index=source['pair_index'],fixed_side=source['fixed_side'],pad4=source['pad4'],sides={})
        result['candidates'].append(row)
        for side in ('left','right'):
            p=Problem(args.task,side);n=Native(p,args.output/f'model-{index}-{side}');x=np.array(source['sides'][side]['x'])
            pure=n.evaluate(x);controls,states,diagnostics=n.controls(x)
            row['sides'][side]=dict(x=x.tolist(),pure=pure,controls=controls,states=states,diagnostics=diagnostics)
            result['cpu_seconds']=cpu_seconds()-start;write_json(args.output/'result.json',result)
            print(json.dumps(dict(stage='recovered_controls',candidate=index,side=side,controls=controls,pure= pure['passed'],cpu_seconds=result['cpu_seconds'])),flush=True)
    result['cpu_seconds']=cpu_seconds()-start;write_json(args.output/'result.json',result)

if __name__=='__main__':main()
