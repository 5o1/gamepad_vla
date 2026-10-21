#!/usr/bin/env python3
"""L1/L2 acceptance; all free-space samples, no steady-state filtering."""
import argparse
import json
from pathlib import Path
import numpy as np
from gamepad_vla.workstation.episode_schema import read_episode
from gamepad_vla.workstation.g20_mapping import G20_JOINT_NAMES


def stats(values):
    a = np.asarray(values).ravel()
    return dict(n=len(a), median=None if len(a) == 0 else float(np.median(a)),
                p95=None if len(a) == 0 else float(np.percentile(a, 95)),
                maximum=None if len(a) == 0 else float(np.max(a)))


def l1(path_a, path_b):
    ma, a = read_episode(path_a)
    mb, b = read_episode(path_b)
    if len(a) != len(b) or not a:
        raise ValueError("L1 needs identical nonempty sampling grids")
    schema_equal = all(set(x) == set(y) and x["commands"] == y["commands"] for x, y in zip(a, b))
    schema_equal &= ma["scene_spec_sha256"] == mb["scene_spec_sha256"]
    schema_equal &= ma["fingertip_offsets_m"] == mb["fingertip_offsets_m"]
    if ma.get("scene_variant", {}).get("name") != "free-space" or mb.get("scene_variant", {}).get("name") != "free-space":
        raise ValueError("reviewer ruling 2: L1 acceptance requires the free-space variant")
    time_delta = np.array([abs(x["sim_time_s"] - y["sim_time_s"]) for x, y in zip(a, b)])
    free = np.array([not (x["robot_contact"] or y["robot_contact"]) for x, y in zip(a, b)])
    hand = np.abs(np.array([r["hand_q"] for r in a]) - np.array([r["hand_q"] for r in b]))
    arm = np.abs(np.array([r["arm_q"] for r in a]) - np.array([r["arm_q"] for r in b]))
    palm = np.linalg.norm(np.array([r["palm_pose_world"] for r in a])[..., :3] - np.array([r["palm_pose_world"] for r in b])[..., :3], axis=-1)
    tips = np.linalg.norm(np.array([r["fingertip_pose_world"] for r in a])[..., :3] - np.array([r["fingertip_pose_world"] for r in b])[..., :3], axis=-1)
    result = dict(gate="L1", schema_equal=bool(schema_equal), rows=len(a),
                  duration_s=min(a[-1]["sim_time_s"] - ma["recording_start_sim_time_s"], b[-1]["sim_time_s"] - mb["recording_start_sim_time_s"]),
                  free_rows=int(free.sum()), contact_rows=int((~free).sum()), time_alignment_s=stats(time_delta))
    for label, mask in (("free", free), ("contact", ~free)):
        result[label] = dict(hand_rad=stats(hand[mask]), arm_rad=stats(arm[mask]),
                             all_joint_rad=stats(np.c_[hand[mask].reshape(-1, 42), arm[mask].reshape(-1, 14)]),
                             palm_m=stats(palm[mask]), fingertip_m=stats(tips[mask]),
                             all_position_m=stats(np.c_[palm[mask], tips[mask].reshape(-1, 10)]))
    result["passed"] = bool(schema_equal and result["duration_s"] >= 60 - 1e-6 and free.any() and time_delta.max() <= .00101
                            and result["free"]["all_joint_rad"]["median"] <= .02
                            and result["free"]["palm_m"]["median"] <= .002
                            and result["free"]["fingertip_m"]["median"] <= .002)
    return result


def l2(path):
    meta, rows = read_episode(path)
    if meta.get("scene_variant", {}).get("name") != "free-space":
        raise ValueError("reviewer ruling 2: L2 acceptance requires the free-space variant")
    free = np.array([not r["robot_contact"] for r in rows])
    hand, arm = [], []
    for r in rows:
        c = r["commands"]
        hcmd = np.array([[c[s + "_hand"]["joint_positions"][j] for j in G20_JOINT_NAMES] for s in ("left", "right")])
        acmd = np.array([c[s + "_arm"]["positions"] for s in ("left", "right")])
        hand.append(np.abs(np.asarray(r["hand_q"]) - hcmd))
        arm.append(np.abs(np.asarray(r["arm_q"]) - acmd))
    result = dict(gate="L2", backend=meta["backend_name"], rows=len(rows), free_rows=int(free.sum()),
                  duration_s=rows[-1]["sim_time_s"] - meta["recording_start_sim_time_s"],
                  hand_rad=stats(np.array(hand)[free]), arm_rad=stats(np.array(arm)[free]))
    run_result = json.loads((Path(path).parent / "result.json").read_text())
    result["decode_errors"] = run_result["decode_errors"]
    result["passed"] = bool(result["duration_s"] >= 300 - 1e-6 and result["decode_errors"] == 0 and free.any()
                            and result["hand_rad"]["median"] <= .05 and result["arm_rad"]["median"] <= .02)
    return result


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("gate", choices=("L1", "L2"))
    p.add_argument("episode", type=Path)
    p.add_argument("--other", type=Path)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    if a.gate == "L1" and a.other is None:
        p.error("L1 requires --other")
    result = l1(a.episode, a.other) if a.gate == "L1" else l2(a.episode)
    a.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result))
    raise SystemExit(0 if result["passed"] else 1)
