#!/usr/bin/env python3
"""Fix one native-valid side and its pad placement; optimize only the other side.

A finite diagnostic of common-placement search, never an impossibility proof.
All positive side checks use the unchanged Native.evaluate acceptance criteria.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

from gamepad_vla.feasibility import transform
from k1b_search import Problem, cpu_seconds
from k1b_native import Native
from k1_height_common import write_json


class CpuBudgetExceeded(Exception):
    pass


def main():
    a = argparse.ArgumentParser(description=__doc__)
    a.add_argument('--root',type=Path,required=True)
    a.add_argument('--task',type=Path,default=Path('configs/tasks/fixed_dpad_causal.json'))
    a.add_argument('--output',type=Path,required=True)
    a.add_argument('--cpu-limit',type=float,default=2000.)
    args = a.parse_args()
    start_cpu = cpu_seconds()
    args.output.mkdir(parents=True,exist_ok=True)
    data = {s:json.loads((args.root/f'native-{s}'/'native.json').read_text())['accepted']
            for s in ('left','right')}
    scale = np.array([.08,.08,.08,.3])
    pairs = sorted([(float(np.linalg.norm((np.array(l['x'])[:4]-np.array(r['x'])[:4])/scale)),li,ri)
                    for li,l in enumerate(data['left']) for ri,r in enumerate(data['right'])])[:4]
    result = dict(scope='Finite common-pose diagnostic; unchanged native acceptance. No failure implies global infeasibility.',
        parameters=dict(pairs=4,directions=2,grip_residual_multiplier=30,natural_residual_multiplier=30,
            trf_max_nfev=180,native_repair_max_nfev=40,cpu_limit_seconds=args.cpu_limit),
        nearest_pairs=[dict(distance=d,left_index=li,right_index=ri) for d,li,ri in pairs],
        attempts=[],common_successes=[])
    problems = {s:Problem(args.task,s) for s in ('left','right')}
    natives = {s:Native(problems[s],args.output/f'model-{s}') for s in ('left','right')}

    def save():
        result['cpu_seconds'] = cpu_seconds()-start_cpu
        write_json(args.output/'result.json',result)

    def budget():
        if cpu_seconds()-start_cpu>=args.cpu_limit:
            raise CpuBudgetExceeded

    try:
        for pair_index,(distance,li,ri) in enumerate(pairs):
            for fixed_side in ('left','right'):
                budget()
                moving_side = 'right' if fixed_side=='left' else 'left'
                indices = dict(left=li,right=ri)
                fixed_x = np.array(data[fixed_side][indices[fixed_side]]['x'])
                moving_x = np.array(data[moving_side][indices[moving_side]]['x'])
                moving_x[:4] = fixed_x[:4]
                p = problems[moving_side]
                native = natives[moving_side]
                fixed_check = natives[fixed_side].evaluate(fixed_x)
                initial = native.evaluate(moving_x)
                record = dict(pair_index=pair_index,distance=distance,fixed_side=fixed_side,
                    source_indices=indices,pad4=fixed_x[:4].tolist(),fixed_x=fixed_x.tolist(),
                    fixed_check=fixed_check,initial_x=moving_x.tolist(),initial=initial)
                result['attempts'].append(record)
                save()
                regularize = moving_x.copy()

                def unpack(z):
                    return np.r_[fixed_x[:4],z]

                def residual(z,repair=False):
                    budget()
                    x = unpack(z)
                    values = p.residual(x,regularize).copy()
                    values[:6] *= 30.
                    # Add the exact source six natural proposal residuals with
                    # 30x strength; the source weak copy remains in values.
                    pad,_,q,_,_ = p.unpack(x)
                    m = p.feature.measure(q,transform(p.pad_centre,pad))
                    up = m['upper_tilt_deg'][0]
                    pitch = m['forearm_pitch_deg'][0]
                    heading = m['forearm_heading_deg'][0]
                    natural = .05*30*np.array([max(0,(10.2-up)/10),max(0,(up-34.8)/15),
                        max(0,(-9.8-pitch)/20),max(0,(pitch-24.8)/15),max(0,(heading-29.8)/30),
                        max(0,-((m['elbow']-m['shoulder'])[0,0])/.1)])
                    return np.r_[values,natural,native.penalty(x)] if repair else np.r_[values,natural]

                xfinal = moving_x.copy()
                check = initial
                if not check['passed']:
                    fit = least_squares(residual,moving_x[4:],bounds=(p.lower[4:],p.upper[4:]),
                        method='trf',x_scale=np.r_[np.ones(7)*.5,np.ones(len(p.lo))*.5],
                        diff_step=1e-4,max_nfev=180,ftol=1e-8,xtol=1e-8,gtol=1e-8)
                    xfinal = unpack(fit.x)
                    check = native.evaluate(xfinal)
                    record['fit'] = dict(nfev=fit.nfev,cost=float(fit.cost),success=bool(fit.success),
                        message=fit.message,x=xfinal.tolist(),check=check)
                    save()
                    if not check['passed']:
                        repaired = least_squares(lambda z:residual(z,True),xfinal[4:],
                            bounds=(p.lower[4:],p.upper[4:]),method='trf',
                            x_scale=np.r_[np.ones(7)*.5,np.ones(len(p.lo))*.5],diff_step=1e-4,
                            max_nfev=40,ftol=1e-8,xtol=1e-8,gtol=1e-8)
                        repaired_x = unpack(repaired.x)
                        repaired_check = native.evaluate(repaired_x)
                        record['repair'] = dict(nfev=repaired.nfev,cost=float(repaired.cost),success=bool(repaired.success),
                            message=repaired.message,x=repaired_x.tolist(),check=repaired_check)
                        if repaired_check['passed'] or not check['passed']:
                            xfinal,check = repaired_x,repaired_check
                record['final_x'] = xfinal.tolist()
                record['final'] = check
                if fixed_check['passed'] and check['passed']:
                    sides = {fixed_side:dict(x=fixed_x.tolist(),check=fixed_check),
                             moving_side:dict(x=xfinal.tolist(),check=check)}
                    result['common_successes'].append(dict(pad4=fixed_x[:4].tolist(),sides=sides,
                        pair_index=pair_index,fixed_side=fixed_side))
                    print(json.dumps(dict(stage='common_pure_success',pair=pair_index,fixed_side=fixed_side,
                        success_count=len(result['common_successes']))),flush=True)
                save()
                print(json.dumps(dict(stage='probe_attempt',pair=pair_index,fixed_side=fixed_side,
                    passed=bool(check['passed']),reasons=check['reasons'],cpu_seconds=result['cpu_seconds'])),flush=True)
                if result['common_successes']:
                    return
    except CpuBudgetExceeded:
        result['stopped'] = 'CPU budget reached; pending trial remains incomplete'
        save()
        print(json.dumps(dict(stage='probe_cpu_stop',cpu_seconds=result['cpu_seconds'])),flush=True)
    save()


if __name__=='__main__': main()
