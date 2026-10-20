#!/usr/bin/env python3
"""Prepare two uniquely named instances of the archived upstream 0.8R asset.

This is a static review configuration, not a calibrated robot control profile.
Run on the compute node; source meshes and generated URDFs remain there.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from gamepad_vla.kinematics import SerialChain


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--derived", type=Path, required=True)
    parser.add_argument("--spec", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    spec = json.loads(args.spec.read_text())
    original = "AR5_5_08R_W4C4A6"
    source = args.source / "arm.urdf"
    reference = SerialChain(source, original + "_tcp")
    args.derived.mkdir(parents=True, exist_ok=True)
    report = {"source_urdf": str(source),
              "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
              "scope": "static preview; existing mounts and home angles assumed",
              "physical_pose_verified": False, "sides": {}}
    for side in ("left", "right"):
        prefix = f"AR5_{side}_5_08R_W4C4A6"
        target = args.derived / f"{side}_arm.urdf"
        root = ET.parse(source).getroot()
        for element in root.iter():
            for key in ("name", "link", "joint"):
                value = element.get(key)
                if value and value.startswith(original):
                    element.set(key, value.replace(original, prefix, 1))
        for mesh in root.iter("mesh"):
            mesh.set("filename", os.path.relpath(
                (args.source / mesh.get("filename")).resolve(), target.parent.resolve()))
        ET.ElementTree(root).write(target, encoding="utf-8", xml_declaration=True)
        arm = spec[side + "_arm"]
        arm.update(urdf=str(target), base_link=prefix + "_base",
                   tip_link=prefix + "_tcp",
                   joint_names=[prefix + f"_joint_{i}" for i in range(1, 8)])
        chain = SerialChain(target, arm["tip_link"])
        q = np.asarray(arm["home_q"])
        lower, upper = chain.active_limits
        assert np.all((q >= lower) & (q <= upper)), (side, q, lower, upper)
        samples = np.stack([np.zeros(7), q, q * 0.5])
        error = float(np.max(np.abs(chain.forward(samples) - reference.forward(samples))))
        assert error < 1e-12
        report["sides"][side] = {
            "urdf": str(target), "sha256": hashlib.sha256(target.read_bytes()).hexdigest(),
            "rename_fk_max_error": error, "home_within_limits": True,
            "home_tcp_in_base": chain.forward(q)[0].tolist(),
            "base_pose": arm["base_pose"], "home_q": arm["home_q"]}
    spec["name"] = "station-g20-08r-static-preview"
    spec["notes"] = [
        "Static review of two upstream AR5_5_08R_W4C4A6 arms, commit e6bcf44ff0203534e14500cb836f57e9f9492292.",
        "Both arms use the R variant. Unique link and joint names; original mesh geometry and kinematics retained.",
        "Upstream joint_1 rpy is zero; no additional station-candidate 90-degree rotation applied.",
        "Base mounting poses, home joint angles and hand mounts inherited from v4.3, not independently verified for this model.",
        "Existing home/IK/clearance certificates do not apply to this preview. No physics stepping or hardware commands.",
        "Archived camera extrinsics and retained driver intrinsics unchanged from v4.3.",
    ]
    spec["appearance"] = str((args.spec.parent / spec["appearance"]).resolve())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(spec, indent=2) + "\n")
    (args.output.parent / "asset_adaptation.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report))


if __name__ == "__main__":
    main()
