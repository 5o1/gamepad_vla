#!/usr/bin/env python3
"""Descriptive full-scene comparison; no acceptance thresholds or pass flag."""
import argparse
import json
from pathlib import Path
import numpy as np
from gamepad_vla.workstation.episode_schema import read_episode
from gamepad_vla.workstation.g20_mapping import G20_JOINT_NAMES


def stats(value):
    a = np.asarray(value).ravel()
    return dict(n=len(a), median=None if not len(a) else float(np.median(a)),
                p95=None if not len(a) else float(np.percentile(a,95)),
                maximum=None if not len(a) else float(np.max(a)))


def load(path):
    meta, rows = read_episode(path)
    hand, arm = [], []
    for row in rows:
        c = row["commands"]
        hand.append(np.abs(np.array(row["hand_q"]) - [[c[s+"_hand"]["joint_positions"][j] for j in G20_JOINT_NAMES] for s in ("left","right")]))
        arm.append(np.abs(np.array(row["arm_q"]) - [c[s+"_arm"]["positions"] for s in ("left","right")]))
    contact = np.array([r["robot_contact"] for r in rows])
    summary = dict(backend=meta["backend_name"], rows=len(rows), duration_s=rows[-1]["sim_time_s"]-meta["recording_start_sim_time_s"],
                   contact_rows=int(contact.sum()), contact_fraction=float(contact.mean()),
                   tracking_all=dict(hand_rad=stats(hand), arm_rad=stats(arm)),
                   tracking_contact=dict(hand_rad=stats(np.asarray(hand)[contact]), arm_rad=stats(np.asarray(arm)[contact])),
                   tracking_free=dict(hand_rad=stats(np.asarray(hand)[~contact]), arm_rad=stats(np.asarray(arm)[~contact])))
    return rows, summary


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("sapien", type=Path)
    p.add_argument("mujoco", type=Path)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    sa, ss = load(a.sapien)
    mj, ms = load(a.mujoco)
    n = min(len(sa), len(mj))
    sa, mj = sa[:n], mj[:n]
    if any(x["commands"] != y["commands"] or abs(x["sim_time_s"]-y["sim_time_s"])>.00101 for x,y in zip(sa,mj)):
        raise ValueError("descriptive comparison still requires aligned identical input")
    contact = np.array([x["robot_contact"] or y["robot_contact"] for x,y in zip(sa,mj)])
    delta = {}
    for name in ("hand_q", "arm_q"):
        d = np.abs(np.array([r[name] for r in sa])-np.array([r[name] for r in mj]))
        delta[name+"_rad"] = stats(d[contact])
    for name in ("palm_pose_world", "fingertip_pose_world"):
        d = np.linalg.norm(np.array([r[name] for r in sa])[...,:3]-np.array([r[name] for r in mj])[...,:3],axis=-1)
        delta[name+"_m"] = stats(d[contact])
    result = dict(mode="report_only", scene="full-v4.4", sapien=ss, mujoco=ms,
                  matched_rows=n, matched_contact_rows=int(contact.sum()),
                  matched_until_sim_s=min(sa[-1]["sim_time_s"],mj[-1]["sim_time_s"]),
                  contact_differences=delta, geometry_note="August data and current v4.4 geometry do not match")
    a.output.write_text(json.dumps(result, indent=2)+"\n")
    print(json.dumps(result))
