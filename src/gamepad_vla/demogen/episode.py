"""Strict v4 input and single-hand views; no new recording format."""
from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np

from gamepad_vla.workstation.collection_integrity import check_integrity
from gamepad_vla.workstation.episode_schema import FINGERS, SIDES, read_episode
from gamepad_vla.workstation.paths import resolve_workspace_path
from gamepad_vla.workstation.spec import spec_from_dict


@dataclass
class HandStream:
    source: str
    side: str
    time: np.ndarray
    arm_q: np.ndarray
    hand_q: np.ndarray
    signal: np.ndarray
    mask: np.ndarray
    palm_world: np.ndarray
    tips_world: np.ndarray
    meta: dict
    spec: object
    rows: list


def scene_from_text(text):
    data = json.loads(text)
    # Collection records effective collision policy beside the geometric spec.
    data.pop("collection_variant", None)
    for side in SIDES:
        for kind in ("arm", "hand"):
            key = side + "_" + kind
            if key in data:
                data[key]["urdf"] = resolve_workspace_path(data[key]["urdf"])
    for fixture in data.get("fixtures", []):
        if fixture.get("mesh"):
            fixture["mesh"] = resolve_workspace_path(fixture["mesh"])
    return spec_from_dict(data)


def load_demonstration(path):
    """Read v4 plus independent receiver counts, time boundaries and sidecars.

    Missing integrity evidence is an error. Synthetic examples use a declared
    zero-event receiver and generated clock, never fabricated ROS messages.
    """
    path = Path(path)
    meta, rows = read_episode(path)
    if meta["schema_version"] != 4:
        raise ValueError("demogen requires collection schema v4")
    if (path / "discarded.json").exists():
        raise ValueError("source episode is discarded")
    sidecar = path / "integrity.json"
    if sidecar.exists():
        saved = json.loads(sidecar.read_text())
        if saved.get("state") != "passed" or saved.get("passed") is not True:
            raise ValueError("source integrity sidecar has not passed")
    integrity = check_integrity(path)
    if not integrity["passed"]:
        raise ValueError("source integrity failed: " + "; ".join(integrity["errors"]))
    if meta.get("side_order") != list(SIDES):
        raise ValueError("source side order must be left, right")
    if meta.get("finger_order") != list(FINGERS):
        raise ValueError("source finger order differs from collection v4")
    clocks = [meta[k] for k in ("fps", "recording_start_sim_time_s", "recording_end_sim_time_s")]
    if not np.isfinite(clocks).all() or meta["fps"] <= 0:
        raise ValueError("source recording clocks and fps must be finite")
    spec = scene_from_text(meta["scene_spec_text"])
    if not spec.gamepad.fixed_root:
        raise ValueError("moving gamepad needs per-frame root poses; v4 does not record them")
    result = {}
    for index, side in enumerate(SIDES):
        result[side] = HandStream(
            str(path.resolve()), side, np.array([r["sim_time_s"] for r in rows]),
            np.array([r["arm_q"][index] for r in rows]),
            np.array([r["hand_q"][index] for r in rows]),
            np.array([r["gamepad_signal"] for r in rows]),
            np.array([r["gamepad_mask"] for r in rows], bool),
            np.array([r["palm_pose_world"][index] for r in rows]),
            np.array([r["fingertip_pose_world"][index] for r in rows]),
            meta, spec, rows)
    return result
