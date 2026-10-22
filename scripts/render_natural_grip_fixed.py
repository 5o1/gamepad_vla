#!/usr/bin/env python3
"""Capture fixed-grip candidates through the existing operator_view renderer.

The child owns the existing SAPIEN scene and appearance pipeline. This script
only publishes static joint states and saves its mmap RGB frames; it neither
steps physics nor starts the collection API, gateway, or hardware input.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidates', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--resources', type=Path)
    parser.add_argument('--spec', type=Path, default=Path(
        'configs/workcell/station_g20_v4_4_raised_interim.json'))
    parser.add_argument('--timeout', type=float, default=150.)
    args = parser.parse_args()

    import numpy as np
    from PIL import Image
    from gamepad_vla.site import configure_runtime
    from gamepad_vla.workstation.g20_mapping import G20_JOINT_NAMES, G20_MIMIC_JOINTS
    from gamepad_vla.workstation.operator_view import load_config
    from gamepad_vla.workstation.spec import load_spec, pose_to_matrix
    from xbox_gamepad_physics.teleop.stream.ipc import (
        Latest, SNAPSHOT_CAPACITY, FRAME_CAPACITY, unpack)

    configure_runtime()
    if args.resources:
        os.environ['XBOX_GAMEPAD_PHYSICS_RESOURCES'] = str(args.resources.resolve())
    from xbox_gamepad_physics.motion import load_motion_contract
    pad_joints = load_motion_contract('full_controls').joint_by_name
    spec = load_spec(args.spec)
    candidates = json.loads(args.candidates.read_text())['sides']
    args.output.mkdir(parents=True, exist_ok=True)
    run = Path(tempfile.mkdtemp(prefix='operator-', dir=args.output.resolve()))

    config = load_config(Path(__file__).resolve().parents[1] /
                         'configs/render/operator_view.yaml')
    intr = next(c.intrinsics for c in spec.cameras if c.name == 'camera_3')
    config.update(collection=True, view='camera_3', width=intr.width,
                  height=intr.height, resources=os.environ['XBOX_GAMEPAD_PHYSICS_RESOURCES'])
    config['pip'] = dict(enabled=False)
    config['overview'] = dict(enabled=False)
    # One world-fixed side view covers the arm bases, elbows and fixed pad.
    # operator_view expects these camera vectors in the gamepad root frame.
    frame = pose_to_matrix(spec.gamepad.root_pose)
    rotation = frame[:3, :3].T
    config['camera'] = dict(
        position_gamepad=(rotation @ (np.array([.43, 1.70, 1.32]) - frame[:3, 3])).tolist(),
        target_gamepad=(rotation @ (np.array([.40, .6252, 1.10]) - frame[:3, 3])).tolist(),
        up_gamepad=(rotation @ np.array([0., 0., 1.])).tolist(), fovy_degrees=48.)
    (run / 'config.json').write_text(json.dumps(config, indent=2) + '\n')
    (run / 'spec_path.txt').write_text(str(args.spec.resolve()))
    source = Latest(run / 'snapshot.mmap', SNAPSHOT_CAPACITY, True)
    frames = Latest(run / 'frame.mmap', FRAME_CAPACITY, True)
    proc = None
    manifest = dict(spec=str(args.spec.resolve()), candidates=str(args.candidates.resolve()),
                    renderer='gamepad_vla.workstation.operator_view', run=str(run), images={})
    try:
        env = dict(os.environ)
        env['PYTHONPATH'] = os.pathsep.join(str(Path(p or '.').resolve())
            for p in env.get('PYTHONPATH', '').split(os.pathsep))
        with (run / 'render.log').open('w') as log:
            proc = subprocess.Popen([sys.executable, '-u', '-m',
                'gamepad_vla.workstation.operator_view', 'render', '--run', str(run)],
                env=env, stdout=log, stderr=subprocess.STDOUT)
            last_frame = -1
            for side in ('left', 'right'):
                check = candidates[side]['check']
                arms = [list(getattr(spec, s + '_arm').home_q) for s in ('left', 'right')]
                hands = [list(getattr(spec, s + '_hand').home_q) for s in ('left', 'right')]
                index = ('left', 'right').index(side)
                arms[index] = check['arm_q']
                hand = dict(check['hand_q'])
                for name, (parent, multiplier) in G20_MIMIC_JOINTS[side].items():
                    hand[name] = hand[parent] * multiplier
                hands[index] = [hand[name] for name in G20_JOINT_NAMES]
                state = dict(arm_q=arms, hand_q=hands,
                    control_joint_positions={name: 0. for name in pad_joints},
                    command_id=0, command_applied_ns=None, candidate_side=side)
                source.write(json.dumps(state).encode())
                manifest['images'][side] = {}
                for view, label in (('camera_3', 'camera_3'), ('operator', 'side')):
                    temporary = run / 'view.tmp'
                    temporary.write_text(json.dumps(dict(view=view)))
                    temporary.replace(run / 'view.json')
                    deadline = time.monotonic() + args.timeout
                    while True:
                        if proc.poll() is not None:
                            raise RuntimeError(f'operator renderer exited ({proc.returncode}); see {run / "render.log"}')
                        row = frames.read()
                        if row:
                            meta, rgb = unpack(row[1])
                            if (meta.get('candidate_side') == side and meta.get('view') == view
                                    and meta['frame_id'] > last_frame):
                                image = np.frombuffer(rgb, np.uint8).reshape(meta['height'], meta['width'], 3)
                                destination = args.output / f'{side}_{label}.png'
                                Image.fromarray(image).save(destination)
                                manifest['images'][side][label] = dict(file=str(destination.resolve()),
                                    frame_id=meta['frame_id'], width=meta['width'], height=meta['height'])
                                last_frame = meta['frame_id']
                                break
                        if time.monotonic() >= deadline:
                            raise TimeoutError(f'no {side}/{view} frame; see {run / "render.log"}')
                        time.sleep(.05)
    finally:
        (run / 'stop').touch()
        if proc is not None:
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()
            manifest['renderer_returncode'] = proc.returncode
        source.close()
        frames.close()
        (args.output / 'render_manifest.json').write_text(
            json.dumps(manifest, indent=2, ensure_ascii=False) + '\n')
    print(json.dumps(manifest, ensure_ascii=False))


if __name__ == '__main__':
    main()
