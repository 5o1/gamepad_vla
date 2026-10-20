#!/usr/bin/env python3
"""Recompute archived extrinsics in a new output directory and compare numbers."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import numpy as np

from gamepad_vla.site import workspace_root


def verify_solver_sources(solver, expected):
    for name, digest in expected.items():
        if hashlib.sha256((solver / "calib_board" / name).read_bytes()).hexdigest() != digest:
            raise ValueError(f"solver source differs from archived input: {name}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    repo = Path(__file__).resolve().parents[1]
    parser.add_argument("--archive", type=Path, default=repo / "references/calibration/2026-10-20-ego/measured")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--solver", type=Path)
    args = parser.parse_args()
    solver = (args.solver or workspace_root() / "external/calib_board").resolve()
    if not (solver / "calib_board/calibrate_extrinsics.py").is_file():
        parser.error("fetch the pinned calibration resource first")
    context = json.loads((args.archive / "capture_context.json").read_text())
    verify_solver_sources(solver, context["solver_source_hashes"])
    output = args.output.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=False)
    for name in ("capture_context.json", "board-placements.json", "board_spec.json", "intrinsics.json", "solve_four_holes.py"):
        shutil.copyfile(args.archive / name, output / name)
    shutil.copytree(args.archive / "images", output / "images")
    env = dict(os.environ, CUDA_VISIBLE_DEVICES="", PYTHONPATH=str(solver))
    subprocess.run([sys.executable, str(output / "solve_four_holes.py")], cwd=output, env=env, check=True)
    expected = json.loads((args.archive / "extrinsics_table.json").read_text())
    actual = json.loads((output / "extrinsics_table.json").read_text())
    delta = float(np.abs(np.asarray(actual["T_world_camera_optical"]) - expected["T_world_camera_optical"]).max())
    project_expected = json.loads((args.archive / "extrinsics_project_world.json").read_text())
    project_actual = json.loads((output / "extrinsics_project_world.json").read_text())
    project_delta = float(np.abs(np.asarray(project_actual["T_project_world_camera_optical"]) -
                                project_expected["T_project_world_camera_optical"]).max())
    metrics = {k: abs(actual[k] - expected[k]) for k in ("reprojection_median_px", "reprojection_p95_px")}
    passed = max(delta, project_delta) < 1e-6 and all(v < 1e-4 for v in metrics.values()) and actual["total_corners"] == 60
    result = {"passed": passed, "matrix_max_abs_difference": max(delta, project_delta), "reprojection_metric_differences_px": metrics,
              "solver_sources_match_archive": True, "archive_modified": False,
              "scope": "numerical reproduction, not new physical calibration"}
    (output / "reproduction.json").write_text(json.dumps(result, indent=2) + "\n")
    (output / "SHA256SUMS").write_text("".join(
        hashlib.sha256(p.read_bytes()).hexdigest() + "  " + str(p.relative_to(output)) + "\n"
        for p in sorted(output.rglob("*")) if p.is_file() and p.name != "SHA256SUMS"))
    print(json.dumps(result))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
