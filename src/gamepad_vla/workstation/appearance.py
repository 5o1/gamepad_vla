"""Backend-neutral linear RGB PBR material rules; first matching rule wins."""
import fnmatch
import json
from pathlib import Path

import numpy as np
from gamepad_vla.workstation.paths import resolve_workspace_path


def load_appearance(path):
    data = json.loads(Path(path).read_text())
    if set(data) not in ({"name", "color_space", "materials"}, {"name", "color_space", "materials", "visual_splits"}) or data["color_space"] != "linear_rgb":
        raise ValueError("appearance requires name, color_space=linear_rgb and materials")
    if not isinstance(data["name"], str) or not isinstance(data["materials"], list):
        raise ValueError("invalid appearance name/materials")
    names = set()
    for entry in data["materials"]:
        if set(entry) != {"name", "match", "base_color", "metallic", "roughness", "status", "source"}:
            raise ValueError("invalid material keys")
        if entry["name"] in names:
            raise ValueError("material names must be unique")
        names.add(entry["name"])
        match = entry["match"]
        if set(match) != {"link_glob", "mesh_glob"}:
            raise ValueError("invalid material match")
        if not all(isinstance(v, list) and all(isinstance(p, str) for p in v) for v in match.values()):
            raise ValueError("material match rules must be lists of strings")
        color = np.asarray(entry["base_color"], float)
        if color.shape != (4,) or not np.isfinite(color).all() or np.any((color < 0) | (color > 1)):
            raise ValueError("base_color must contain four finite values in [0, 1]")
        for key in ("metallic", "roughness"):
            if not isinstance(entry[key], (float, int)) or not np.isfinite(entry[key]) or not 0 <= entry[key] <= 1:
                raise ValueError(key + " must be in [0, 1]")
        if entry["status"] not in {"measured", "design", "assumed", "estimated"} or not isinstance(entry["source"], str):
            raise ValueError("material requires status and source")
    for split in data.get("visual_splits", []):
        if set(split) != {"link", "source_mesh", "parts"} or not split["parts"]:
            raise ValueError("invalid visual split")
        split["source_mesh"] = resolve_workspace_path(split["source_mesh"])
        for part in split["parts"]:
            if set(part) != {"mesh", "material"} or part["material"] not in names:
                raise ValueError("visual split requires mesh and known material")
            part["mesh"] = resolve_workspace_path(part["mesh"])
    return data


def match_material(table, link, mesh=""):
    for entry in table["materials"]:
        rules = entry["match"]
        if any(fnmatch.fnmatchcase(link, p) for p in rules["link_glob"]) or any(
                fnmatch.fnmatchcase(mesh, p) for p in rules["mesh_glob"]):
            return entry
    return None


def match_visual_split(table, link, mesh):
    """Exact link/source pairing prevents replacing another visual on the link."""
    return next((s for s in table.get("visual_splits", [])
                 if s["link"] == link and Path(s["source_mesh"]).resolve() == Path(mesh).resolve()), None)
