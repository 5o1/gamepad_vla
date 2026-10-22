#!/usr/bin/env python3
"""Paired finite-difference contact-polish diagnostic for Task S.

Compare relative K1-b steps with fixed absolute radian steps, holding the seed,
objective weights, limits and native acceptance fixed. No proposal is a pass.
SciPy 1.15.3 least_squares accepts a callable Jacobian; its diff_step is relative:
https://docs.scipy.org/doc/scipy-1.15.3/reference/generated/scipy.optimize.least_squares.html
"""
from __future__ import annotations
import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import json
import multiprocessing
import os
from pathlib import Path
import time

import numpy as np
from scipy.optimize import least_squares
from natural_grip_fixed import FixedProblem, FixedNative
from k1_height_common import write_json

PROBLEMS = {}
NATIVES = {}
OUTPUT = None
DEADLINE = None


def score(c):
    depth = max([max(0., -b['distance_m']) for b in c['violations']]+[0.])
    point = max(0., max(c['grip_point_error_m'])-.00025)
    natural = max(0., c['natural']['deviation']-1.)*.01
    missing = len({'ring', 'pinky'}-set(c['grips']))*.005
    return (not c['passed'], max(depth, point, natural, missing),
            depth+point+natural+missing, c['natural']['wrist_deviation_deg'])


def solve(item):
    side, seed_index, startx, step, native_feasibility = item
    p = PROBLEMS[side]
    if side not in NATIVES:
        NATIVES[side] = FixedNative(p, OUTPUT/'models'/str(os.getpid())/side)
    native = NATIVES[side]
    x0 = np.array(startx)
    initial = native.evaluate(x0)
    start = time.time()
    def residual(z):
        if time.time() >= DEADLINE:
            raise TimeoutError('Task S deadline')
        x = np.r_[p.fixed4, z]
        if native_feasibility:
            # Once near contact, remove competing approximate surface/normal
            # preferences. Penalize only violations of the actual hard gates,
            # with a 0.05 mm point and 0.15 degree natural interior margin.
            return native.feasibility_residual(x, x0)
        return np.r_[p.residual(x, x0), native.penalty(x)*3]
    def jacobian(z):
        base = residual(z)
        jac = np.empty((len(base), len(z)))
        for i in range(len(z)):
            zz = z.copy()
            h = step if z[i]+step < p.upper[4+i] else -step
            zz[i] += h
            jac[:, i] = (residual(zz)-base)/h
        return jac
    fit = least_squares(residual, x0[4:], bounds=(p.lower[4:], p.upper[4:]),
        jac=jacobian if step else '2-point', diff_step=1e-4, x_scale=.5,
        max_nfev=200 if native_feasibility else 120, ftol=1e-9, xtol=1e-9, gtol=1e-9)
    x = np.r_[p.fixed4, fit.x]
    check = native.evaluate(x)
    return dict(side=side, seed_index=seed_index, x=x.tolist(), check=check,
        initial_check=initial, absolute_step_rad=step, nfev=fit.nfev,
        native_feasibility=native_feasibility,
        optimizer_message=fit.message, cost=float(fit.cost), elapsed_seconds=time.time()-start)


def main():
    global PROBLEMS, OUTPUT, DEADLINE
    a = argparse.ArgumentParser(description=__doc__)
    a.add_argument('--task', type=Path, required=True)
    a.add_argument('--search', type=Path, required=True)
    a.add_argument('--output', type=Path, required=True)
    a.add_argument('--deadline', type=float, required=True)
    a.add_argument('--workers', type=int, default=2)
    a.add_argument('--per-side', type=int, default=2)
    a.add_argument('--side', choices=('left', 'right', 'both'), default='both')
    a.add_argument('--native-feasibility', action='store_true')
    args = a.parse_args()
    OUTPUT, DEADLINE = args.output, args.deadline
    OUTPUT.mkdir(parents=True, exist_ok=True)
    rows = [json.loads(f.read_text()) for f in (args.search/'checkpoints').glob('*.json')]
    items = []
    for side in ('left', 'right'):
        if args.side != 'both' and args.side != side:
            continue
        PROBLEMS[side] = FixedProblem(args.task, side)
        rr = []
        for row in rows:
            if row['side'] != side:
                continue
            rr.append(row)
            if row.get('repair'):
                rr.append(dict(side=side, seed_index=row['seed_index'], **row['repair']))
        rr.sort(key=lambda row: score(row['check']))
        seen = set()
        selected = []
        for row in rr:
            if row['seed_index'] not in seen:
                selected.append(row)
                seen.add(row['seed_index'])
            if len(selected) >= args.per_side:
                break
        for row in selected:
            for step in ((.0002, .001) if args.native_feasibility else (0., .0002, .001)):
                items.append((side, row['seed_index'], row['x'], step, args.native_feasibility))
    write_json(OUTPUT/'paired-starts.json', items)
    checkpoint = OUTPUT/'checkpoints'
    checkpoint.mkdir(exist_ok=True)
    with ProcessPoolExecutor(max_workers=args.workers,
            mp_context=multiprocessing.get_context('fork')) as pool:
        for future in as_completed([pool.submit(solve, item) for item in items]):
            row = future.result()
            path = checkpoint/f"{row['side']}-{row['seed_index']:04d}-{row['absolute_step_rad']:.4f}.json"
            write_json(path, row)
            print(json.dumps(dict(side=row['side'], seed=row['seed_index'], step=row['absolute_step_rad'],
                passed=row['check']['passed'], reasons=row['check']['reasons'],
                score=score(row['check']), nfev=row['nfev'])), flush=True)


if __name__ == '__main__':
    main()
