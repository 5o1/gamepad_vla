#!/usr/bin/env python3
"""One authorized 0.5 s, eight-worker CPU experiment; no parameter sweep.

Run inside the independent compute checkout, with BLAS/OpenMP limited to one
thread and CUDA_VISIBLE_DEVICES empty. Outputs stay in the compute workspace.
"""
import argparse
import json
from pathlib import Path
import time

import mujoco
import numpy as np

from gamepad_vla.workstation.collection_mujoco import CollectionMujocoBackend
from gamepad_vla.workstation.collection_sources import CollectionReplay
from gamepad_vla.workstation.spec import load_spec


def distribution(values):
    values = np.asarray(values, float)
    return dict(median=float(np.median(values)), p95=float(np.percentile(values, 95)),
                maximum=float(np.max(values)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--spec', type=Path, required=True)
    parser.add_argument('--replay', type=Path, required=True)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    baseline_meta = json.loads((args.baseline / 'meta.json').read_text())
    baseline = [json.loads(s) for s in (args.baseline / 'steps.jsonl').read_text().splitlines()][:15]
    if len(baseline) != 15:
        raise ValueError('baseline must contain at least 15 control rows')
    args.output.mkdir(parents=True, exist_ok=False)
    backend = CollectionMujocoBackend(load_spec(args.spec), work_dir=args.output / 'model', render=False)
    source = CollectionReplay(args.replay)
    rows, step_wall = [], []
    try:
        backend.reset()
        mujoco.mju_threadpool(backend.data, 8)
        started = time.perf_counter()
        with (args.output / 'observations.jsonl').open('x', buffering=1) as stream:
            for i in range(15):
                backend.apply(source.poll(i / 30))
                step_start = time.perf_counter()
                backend.step(1 / 30)
                step_wall.append(time.perf_counter() - step_start)
                row = backend.observe().to_dict()
                rows.append(row)
                stream.write(json.dumps(row) + '\n')
                print(json.dumps(dict(row=i + 1, sim_time_s=row['sim_time_s'],
                    step_wall_s=step_wall[-1], contacts=int(backend.data.ncon))), flush=True)
        elapsed = time.perf_counter() - started
        baseline_elapsed = (baseline[-1]['wall_time_ns'] - baseline_meta['created_wall_time_ns']) / 1e9
        result = dict(worker_threads=8, rows=15, simulation_time_s=rows[-1]['sim_time_s'],
            wall_s=elapsed, physics_step_wall_s=sum(step_wall), baseline_first_15_wall_s=baseline_elapsed,
            baseline_to_experiment_speed_ratio=baseline_elapsed / elapsed,
            decode_errors=source.decode_errors, contact_mismatches=sum(
                a['robot_contact'] != b['robot_contact'] for a, b in zip(rows, baseline)),
            contact_rows=sum(a['robot_contact'] for a in rows))
        for field in ('hand_q', 'arm_q'):
            result[field + '_difference_rad'] = distribution(np.abs(
                np.asarray([r[field] for r in rows]) - np.asarray([r[field] for r in baseline])))
        for field in ('palm_pose_world', 'fingertip_pose_world'):
            current = np.asarray([r[field] for r in rows])
            previous = np.asarray([r[field] for r in baseline])
            result[field + '_position_difference_m'] = distribution(np.linalg.norm(
                current[..., :3] - previous[..., :3], axis=-1))
            result[field + '_orientation_difference_rad'] = distribution(2 * np.arccos(np.clip(
                np.abs(np.sum(current[..., 3:] * previous[..., 3:], axis=-1)), 0, 1)))
        result['simulation_timestamp_max_difference_s'] = max(abs(a['sim_time_s'] - b['sim_time_s'])
                                                             for a, b in zip(rows, baseline))
        (args.output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result), flush=True)
    finally:
        source.close()
        backend.close()


if __name__ == '__main__':
    main()
