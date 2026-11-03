#!/usr/bin/env python3
"""Index v4 demonstrations or generate and physically validate augmentations."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from gamepad_vla.demogen.generate import GenerateConfig, identity_trajectory
from gamepad_vla.demogen.output import data_directory
from gamepad_vla.demogen.pipeline import build_library, dump_json, run
from gamepad_vla.demogen.segments import SliceConfig
from gamepad_vla.demogen.synthetic import from_candidates, physical_from_candidates
from gamepad_vla.demogen.validate import ValidationConfig, replay
from gamepad_vla.workstation.spec import load_spec


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    synthetic = sub.add_parser("synthetic", help="make a declared, unvalidated static-candidate fixture")
    synthetic.add_argument("--spec", type=Path, required=True)
    synthetic.add_argument("--candidates", type=Path, required=True)
    synthetic.add_argument("--repetitions", type=int, default=3)
    synthetic.add_argument("--fps", type=int, default=30)
    synthetic.add_argument("--physical", action="store_true", help="native feedback source instead of scripted signal labels")
    synthetic.add_argument("--name", required=True, help="new path below the configured data root's demogen/")
    for command in ("index", "identity", "generate"):
        p = sub.add_parser(command)
        p.add_argument("episodes", nargs="+", type=Path)
        p.add_argument("--side", choices=("left", "right"), default="left")
        p.add_argument("--config", type=Path, default=Path(__file__).resolve().parents[1] / "configs/demogen/default.json")
        p.add_argument("--name", required=True, help="new path below the configured data root's demogen/")
        if command == "identity":
            p.add_argument("--backend", choices=("auto", "sapien", "mujoco"), default="auto")
        if command == "generate":
            p.add_argument("--target-spec", type=Path, action="append", required=True)
            p.add_argument("--sequence", required=True, help="comma-separated exact labels from the index")
            p.add_argument("--count", type=int, default=1)
            p.add_argument("--seed", type=int, default=0)
            p.add_argument("--backend", choices=("auto", "sapien", "mujoco"), default="auto")
            p.add_argument("--resource-root", type=Path, help="existing asset resources for the selected backend")
    args = parser.parse_args()
    try:
        output = data_directory(args.name)
        if args.command == "synthetic":
            builder = physical_from_candidates if args.physical else from_candidates
            result = builder(load_spec(args.spec), args.candidates, output,
                             fps=args.fps, repetitions=args.repetitions)
            print(json.dumps(dict(output=str(output), synthetic=True, integrity=result)))
            return 0
        settings = json.loads(args.config.read_text())
        slicing = SliceConfig(**settings.get("slicing", {}))
        if args.command == "index":
            _, library = build_library(args.episodes, args.side, slicing)
            output.mkdir(parents=True, exist_ok=False)
            dump_json(output / "segments.json", [s.to_dict() for s in library])
            labels = sorted({s.control for s in library})
            print(json.dumps(dict(output=str(output), segments=len(library), controls=labels)))
            return 0
        if args.command == "identity":
            streams, library = build_library(args.episodes, args.side, slicing)
            output.mkdir(parents=True, exist_ok=False)
            results = []
            from gamepad_vla.demogen.pipeline import available_backend
            backend = available_backend() if args.backend == "auto" else args.backend
            for stream in streams:
                trajectory = identity_trajectory(stream, [s for s in library if s.source == stream.source])
                result, _, _ = replay(trajectory, backend_name=backend,
                                      config=ValidationConfig(**settings.get("validation", {})))
                results.append(dict(source=stream.source, validation=result))
            dump_json(output / "identity.json", results)
            print(json.dumps(dict(output=str(output), passed=all(r["validation"]["passed"] for r in results))))
            return 0 if all(r["validation"]["passed"] for r in results) else 2
        sequence = [x.strip() for x in args.sequence.split(",") if x.strip()]
        result = run(args.episodes, args.target_spec, sequence, output, side=args.side,
                     count=args.count, seed=args.seed, backend=args.backend, slicing=slicing,
                     generation=GenerateConfig(**settings.get("generation", {})),
                     validation=ValidationConfig(**settings.get("validation", {})),
                     resource_root=args.resource_root)
        print(json.dumps(dict(output=str(output), status=result["status"],
                              accepted=result["accepted"], rejected=result["rejected"])))
        return 0 if result["accepted"] else 2
    except (OSError, ValueError, RuntimeError) as exc:
        print(type(exc).__name__+": "+str(exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
