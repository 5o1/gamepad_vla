#!/usr/bin/env python3
"""Prepare the current workcell from the public, pinned resource downloads."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

from gamepad_vla.site import workspace_root


def normalized_xml_hash(root):
    root = ET.fromstring(ET.tostring(root))
    root.attrib.pop("name", None)
    for element in root.iter():
        if element.text is not None and not element.text.strip():
            element.text = None
        if element.tail is not None and not element.tail.strip():
            element.tail = None
    return hashlib.sha256(ET.tostring(root)).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    repo = Path(__file__).resolve().parents[1]
    parser.add_argument("--spec", type=Path, default=repo / "configs/workcell/station_g20_v4_4.json")
    args = parser.parse_args()
    workspace = workspace_root()
    manifest = json.loads((repo / "references/external_resources.json").read_text())
    resource = next(r for r in manifest["resources"] if r["id"] == "g20")
    report = {}
    for side in ("left", "right"):
        source = workspace / resource["destination"] / "L25" / side
        root = ET.parse(source / f"linkerhand_l25_{side}.urdf").getroot()
        digest = normalized_xml_hash(root)
        if digest != resource["normalized_xml_sha256"][side]:
            raise ValueError(f"{side} hand geometry/inertia/joints differ from the validated station model")
        target = workspace / "assets/linkerhand_station_20261014/linkerhand_g20_l20_l25_urdf_mesh" / ("g20_" + side)
        target.mkdir(parents=True, exist_ok=True)
        root.set("name", "linkerhand_g20_" + side)
        ET.ElementTree(root).write(target / f"linkerhand_g20_{side}.urdf", encoding="utf-8", xml_declaration=True)
        shutil.copytree(source / "meshes", target / "meshes", dirs_exist_ok=True)
        shutil.copyfile(workspace / resource["destination"] / "LICENSE", target / "LICENSE")
        report[side] = {"normalized_xml_sha256": digest, "matches_station_model": True}
    (workspace / "assets/g20_public_mapping.json").write_text(json.dumps(report, indent=2) + "\n")
    derived = workspace / "assets/derived/ar5_08r_e6bcf44f"
    subprocess.run([sys.executable, str(repo / "scripts/prepare_ar5_08r_preview.py"),
                    "--source", str(workspace / "assets/ar5_08r_linker_sim_e6bcf44f"),
                    "--derived", str(derived), "--spec", str(args.spec),
                    "--output", str(derived / "generated_preview.json")], check=True, stdout=subprocess.DEVNULL)
    subprocess.run([sys.executable, str(repo / "scripts/build_visual_split.py"), "--spec", str(args.spec)], check=True)
    print("Prepared both 0.8R arms, both G20 hands and the current hand visual partitions")


if __name__ == "__main__":
    main()
