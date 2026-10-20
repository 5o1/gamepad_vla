#!/usr/bin/env python3
"""Validate active workcell mesh references and joint limits without a GPU."""
import argparse
import json
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from gamepad_vla.site import workspace_root
from gamepad_vla.workstation.appearance import load_appearance, match_visual_split
from gamepad_vla.workstation.sapien_backend import compose_robot_urdf
from gamepad_vla.workstation.spec import load_spec


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spec", type=Path, default=Path(__file__).resolve().parents[1] / "configs/workcell/station_g20_v4_4.json")
    args = parser.parse_args()
    spec = load_spec(args.spec)
    appearance = load_appearance(spec.source_dir / spec.appearance)
    temporary = workspace_root() / "tmp"
    temporary.mkdir(parents=True, exist_ok=True)
    referenced = set()
    with tempfile.TemporaryDirectory(dir=temporary) as directory:
        for side in ("left", "right"):
            path = compose_robot_urdf(spec, Path(directory) / (side + ".urdf"), side)
            root = ET.parse(path).getroot()
            arm = getattr(spec, side + "_arm")
            for name, value in zip(arm.joint_names, arm.home_q):
                limit = root.find(f"joint[@name='{name}']/limit")
                if not float(limit.get("lower")) <= value <= float(limit.get("upper")):
                    raise ValueError(f"home is outside limits: {name}")
            for link in root.findall("link"):
                for mesh in link.findall(".//mesh"):
                    filename = mesh.get("filename")
                    referenced.add(Path(filename))
                    split = match_visual_split(appearance, link.get("name"), filename)
                    if split:
                        referenced.update(Path(p["mesh"]) for p in split["parts"])
    referenced.update(Path(f.mesh) for f in spec.fixtures if f.mesh)
    missing = [str(p) for p in referenced if not p.is_file()]
    if missing:
        raise FileNotFoundError("Missing meshes:\n" + "\n".join(sorted(missing)))
    print(json.dumps({"spec": spec.name, "referenced_mesh_files": len(referenced), "all_meshes_present": True,
                      "home_joint_limits_valid": True, "physics_or_hardware_validation": False}))


if __name__ == "__main__":
    main()
