#!/usr/bin/env python3
"""Reproduce the active G20 visual partitions from source meshes and appearance rules."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import trimesh

from gamepad_vla.workstation.appearance import load_appearance
from gamepad_vla.workstation.spec import load_spec
from gamepad_vla.workstation.sapien_backend import compose_robot_urdf
from gamepad_vla.site import workspace_root
import xml.etree.ElementTree as ET


def clip_surface(mesh, normal, origin, include_coplanar):
    triangles = mesh.triangles
    distances = (triangles - origin) @ normal
    inside = distances >= -1e-10
    full = inside.all(1)
    if not include_coplanar:
        full &= ~(np.abs(distances) < 1e-10).all(1)
    pieces = [triangles[full]]
    partial = inside.any(1) & ~inside.all(1)
    for triangle, distance in zip(triangles[partial], distances[partial]):
        polygon = []
        for i in range(3):
            a, b = triangle[i - 1], triangle[i]
            da, db = distance[i - 1], distance[i]
            if (da >= 0) != (db >= 0):
                polygon.append(a + (b - a) * (da / (da - db)))
            if db >= 0:
                polygon.append(b)
        for i in range(1, len(polygon) - 1):
            pieces.append(np.array([[polygon[0], polygon[i], polygon[i + 1]]]))
    points = np.concatenate(pieces).reshape(-1, 3)
    return trimesh.Trimesh(vertices=points, faces=np.arange(len(points)).reshape(-1, 3), process=False)


def hand_parts(mesh, link):
    if link.endswith("hand_base_link"):
        normal, origin = np.array([1., 0., 0.]), np.zeros(3)
        return (clip_surface(mesh, normal, origin, False), clip_surface(mesh, -normal, origin, True))
    mask = np.zeros(len(mesh.faces), dtype=bool)
    components = trimesh.graph.connected_components(mesh.face_adjacency, nodes=np.arange(len(mesh.faces)), min_len=1)
    for faces in components:
        points = mesh.vertices[mesh.faces[faces]].reshape(-1, 3)
        if len(faces) > 7000 and points[:, 2].min() > 0:
            mask[faces] = True
    if mask.all() or not mask.any():
        raise ValueError(f"visual partition does not match the expected G20 source: {link}")
    return tuple(trimesh.Trimesh(vertices=mesh.vertices, faces=mesh.faces[m], process=False) for m in (mask, ~mask))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spec", type=Path, default=Path(__file__).resolve().parents[1] / "configs/workcell/station_g20_v4_4.json")
    args = parser.parse_args()
    spec = load_spec(args.spec)
    appearance = load_appearance(spec.source_dir / spec.appearance)
    output = workspace_root() / "assets/derived/visual_split_v1"
    output.mkdir(parents=True, exist_ok=True)
    active = {}
    for side in ("left", "right"):
        path = compose_robot_urdf(spec, output / (side + "_source.urdf"), side)
        for link in ET.parse(path).getroot().findall("link"):
            for mesh in link.findall("visual/geometry/mesh"):
                active[(link.get("name"), str(Path(mesh.get("filename")).resolve()))] = True
    records = []
    for rule in appearance.get("visual_splits", []):
        source = Path(rule["source_mesh"]).resolve()
        if (rule["link"], str(source)) not in active:
            continue
        if rule["link"].startswith("AR5_"):
            raise ValueError("This generator reproduces the current 0.8R/G20 profile, not historical 0.7 arm partitions")
        mesh = trimesh.load(source, process=True)
        parts = hand_parts(mesh, rule["link"])
        area_error = abs(sum(p.area for p in parts) - mesh.area) / mesh.area
        if area_error >= 1e-6:
            raise ValueError(f"incomplete surface partition: {rule['link']}")
        files = []
        for part, definition in zip(parts, rule["parts"]):
            path = Path(definition["mesh"])
            path.parent.mkdir(parents=True, exist_ok=True)
            part.export(path)
            files.append({"file": path.name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()})
        records.append({"link": rule["link"], "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
                        "relative_area_error": float(area_error), "outputs": files})
    (output / "manifest.json").write_text(json.dumps(records, indent=2) + "\n")
    print(json.dumps({"meshes": len(records), "visual_parts": sum(len(r["outputs"]) for r in records)}))


if __name__ == "__main__":
    main()
