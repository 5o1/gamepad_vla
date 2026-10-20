#!/usr/bin/env python3
"""Render a workcell through xbox_gamepad_physics's example runner."""

import argparse
import json
import os
from pathlib import Path
import sys

import yaml

from xbox_gamepad_physics.examples import EXPERIMENTS, lazy_experiment
from xbox_gamepad_physics.examples.runner import apply_overrides, load_config, run_experiment

from gamepad_vla.workstation.render_review import prepare_config
from gamepad_vla.site import configure_runtime


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", type=Path)
    parser.add_argument("--spec", type=Path,
                        default=Path(__file__).resolve().parents[1] / "configs/workcell/station_g20_v4_4.json",
                        help="Workcell JSON (default: adopted dual-0.8R v4.4 scene)")
    parser.add_argument("--set", dest="overrides", nargs="+", action="extend", default=[])
    parser.add_argument("--output", type=Path)
    parser.add_argument("--resources", type=Path, help="prepared resource directory")
    args = parser.parse_args(argv)
    configure_runtime()
    EXPERIMENTS.setdefault("workcell_review", lazy_experiment("gamepad_vla.workstation.render_review"))
    EXPERIMENTS.setdefault("workcell_twin_review", lazy_experiment("gamepad_vla.workstation.render_review"))
    try:
        cfg = prepare_config(apply_overrides(load_config(args.config), args.overrides), args.spec)
        if cfg["experiment"] not in EXPERIMENTS:
            raise ValueError(f"unknown experiment: {cfg['experiment']}")
    except (KeyError, ValueError, yaml.YAMLError) as error:
        parser.error(str(error))
    command = [sys.executable, *(sys.argv if argv is None else [str(Path(__file__).resolve()), *argv])]
    if args.resources is not None:
        from xbox_gamepad_physics.resources import RESOURCE_ENV, resource_root
        os.environ[RESOURCE_ENV] = str(args.resources.expanduser().resolve())
        resource_root.cache_clear()
    from xbox_gamepad_physics.errors import XboxGamepadPhysicsError
    # The runner creates its scene before SapienBackend receives it. Select the
    # native engine device here, before that scene can initialize a default GPU.
    device_guard = None
    device = os.environ.get("GAMEPAD_VLA_RENDER_DEVICE")
    if device:
        import sapien
        sapien.render.set_global_config(max_num_materials=50000, max_num_textures=50000)
        device_guard = sapien.render.SapienRenderer(sapien.Device(device))
    try:
        metrics = run_experiment(cfg, args.config, args.output, command)
    except (XboxGamepadPhysicsError, OSError) as error:
        print(f"example: {error}", file=sys.stderr)
        return 2
    print(json.dumps(metrics))
    return 0 if metrics.get("passed", True) else 1


if __name__ == "__main__":
    raise SystemExit(main())
