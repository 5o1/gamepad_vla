"""Read-only URDF geometry, in controller-local millimetres.

The upstream export has collision meshes only. Those meshes are also used for
the preview, with per-control colours; no replacement controller is invented.
"""
from __future__ import annotations

import itertools
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_URDF = ROOT.parent / "xbox_gamepad_physics/build/resources/robotwin/full_controls/mobility.urdf"
SHELL_NAMES = {"shell", "shell_front_plate"}
COLORS = {
    "shell": "#d7dbe0", "shell_front_plate": "#b7bec8",
    "a": "#48b269", "b": "#e75b5d", "x": "#4295dc", "y": "#e8bc45",
    "guide": "#eceff4", "dpad": "#404952",
    "left_stick": "#424b56", "right_stick": "#424b56",
    "left_shoulder": "#6a7480", "right_shoulder": "#6a7480",
    "left_trigger": "#505b68", "right_trigger": "#505b68",
}


def rotation(axis, angle):
    axis = np.asarray(axis, float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c+x*x*(1-c), x*y*(1-c)-z*s, x*z*(1-c)+y*s],
                     [y*x*(1-c)+z*s, c+y*y*(1-c), y*z*(1-c)-x*s],
                     [z*x*(1-c)-y*s, z*y*(1-c)+x*s, c+z*z*(1-c)]])


def origin_transform(element):
    t = np.eye(4)
    if element is None:
        return t
    t[:3, 3] = np.fromstring(element.get("xyz", "0 0 0"), sep=" ") * 1000
    r, p, y = np.fromstring(element.get("rpy", "0 0 0"), sep=" ")
    t[:3, :3] = rotation([0, 0, 1], y) @ rotation([0, 1, 0], p) @ rotation([1, 0, 0], r)
    return t


class ControllerAsset:
    def __init__(self, urdf=DEFAULT_URDF):
        self.urdf = Path(urdf)
        self.tree = ET.parse(self.urdf).getroot()
        self.joints = {}
        self.child_joint = {}
        for element in self.tree.findall("joint"):
            limit = element.find("limit")
            axis = element.find("axis")
            data = {"name": element.get("name"), "type": element.get("type"),
                    "parent": element.find("parent").get("link"),
                    "child": element.find("child").get("link"),
                    "origin": origin_transform(element.find("origin")),
                    "axis": np.fromstring(axis.get("xyz", "1 0 0"), sep=" ") if axis is not None else np.array([1., 0, 0]),
                    "limits": (float(limit.get("lower", 0)), float(limit.get("upper", 0))) if limit is not None else (0, 0)}
            self.joints[data["name"]] = data
            self.child_joint[data["child"]] = data
        self.geometry = []
        for link in self.tree.findall("link"):
            for collision in link.findall("collision"):
                mesh = collision.find("geometry/mesh")
                if mesh is None:
                    continue
                path = self.urdf.parent / mesh.get("filename")
                loaded = trimesh.load(path, force="mesh", process=True)
                loaded.apply_scale(np.fromstring(mesh.get("scale", "1 1 1"), sep=" ") * 1000)
                self.geometry.append((collision.get("name", link.get("name")), link.get("name"),
                                      origin_transform(collision.find("origin")), loaded, path))

    def link_transform(self, link, state=None):
        state = state or {}
        if link not in self.child_joint:
            return np.eye(4)
        j = self.child_joint[link]
        motion = np.eye(4)
        value = state.get(j["name"], 0.)
        if j["type"] in ("revolute", "continuous"):
            motion[:3, :3] = rotation(j["axis"], value)
        elif j["type"] == "prismatic":
            motion[:3, 3] = j["axis"] * value * 1000
        return self.link_transform(j["parent"], state) @ j["origin"] @ motion

    def meshes(self, state=None):
        result = {}
        for name, link, origin, mesh, _ in self.geometry:
            copy = mesh.copy()
            copy.apply_transform(self.link_transform(link, state) @ origin)
            result[name] = copy
        return result

    def components(self, names=None):
        """Preserve OBJ hull groups; do not merge shared hull-boundary vertices."""
        result = []
        for name, link, origin, _, path in self.geometry:
            if names is not None and name not in names:
                continue
            vertices, groups, faces = [], [], []
            for line in path.read_text().splitlines():
                fields = line.split()
                if not fields:
                    continue
                if fields[0] == "v":
                    vertices.append([float(v) * 1000 for v in fields[1:4]])
                elif fields[0] == "o":
                    if faces:
                        groups.append(faces)
                        faces = []
                elif fields[0] == "f":
                    faces.append([int(v.split("/")[0]) - 1 for v in fields[1:4]])
            if faces:
                groups.append(faces)
            for faces in groups:
                mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=True)
                mesh.remove_unreferenced_vertices()
                mesh.apply_transform(self.link_transform(link) @ origin)
                result.append((name, mesh))
        return result

    def ancestors(self, link):
        joints = []
        while link in self.child_joint:
            j = self.child_joint[link]
            joints.append(j)
            link = j["parent"]
        return joints

    def sweep_envelopes(self, samples=9):
        """Convex outer approximations of complete URDF joint-limit sweeps.

        Independent stick/dpad axes include their rectangular-limit corners.
        Each revolute interval is sampled; 0.35 mm additional clearance is used
        by the checker to cover the small chord error at the chosen resolution.
        """
        result = {}
        for name, link, origin, mesh, _ in self.geometry:
            if name in SHELL_NAMES:
                continue
            joints = self.ancestors(link)
            states = []
            grids = []
            for j in joints:
                lo, hi = j["limits"]
                count = samples if j["type"] in ("revolute", "continuous") else 2
                grids.append(np.linspace(lo, hi, count))
            for values in itertools.product(*grids):
                state = {j["name"]: q for j, q in zip(joints, values)}
                t = self.link_transform(link, state) @ origin
                states.append(trimesh.transform_points(mesh.vertices, t))
            result[name] = trimesh.convex.convex_hull(np.vstack(states))
        return result

    def sweep_sampling_bounds(self,samples=9):
        """Analytic upper bound on arc/chord deviation, summed over axes.

        Each joint's radius is bounded by the geometry radius plus descendant
        origin distances and maximum prismatic travel. Rigid rotations preserve
        that bound. Arc/chord sagitta is rho*(1-cos(delta_angle/2)).
        """
        result={}
        for name,link,origin,mesh,_ in self.geometry:
            if name in SHELL_NAMES:continue
            rho=float(np.linalg.norm(trimesh.transform_points(mesh.vertices,origin),axis=1).max())
            error=0.
            for joint in self.ancestors(link):
                lo,hi=joint["limits"]
                if joint["type"] in ("revolute","continuous"):
                    error+=rho*(1-math.cos((hi-lo)/(samples-1)/2))
                elif joint["type"]=="prismatic":
                    rho+=max(abs(lo),abs(hi))*1000
                rho+=float(np.linalg.norm(joint["origin"][:3,3]))
            result[name]=error
        return result

    def shell(self):
        return trimesh.util.concatenate([m for n, m in self.meshes().items() if n in SHELL_NAMES])

    def shell_section(self, x=0.):
        """Return yz polylines from actual asset triangles at pad-local x."""
        section = self.shell().section(plane_origin=[x, 0, 0], plane_normal=[1, 0, 0])
        return [] if section is None else [line[:, 1:3] for line in section.discrete]

    def summary(self):
        meshes = self.meshes()
        return {"urdf": str(self.urdf), "units": "mm", "mesh_source": "URDF collision/coacd",
                "bounds_mm": {name: m.bounds.tolist() for name, m in meshes.items()},
                "joint_limits": {name: {"type": j["type"], "lower": j["limits"][0], "upper": j["limits"][1]}
                                 for name, j in self.joints.items()},
                "shell_center_sections_yz_mm": {str(x): [a.tolist() for a in self.shell_section(x)] for x in (-12, 0, 12)}}


if __name__ == "__main__":
    import json
    print(json.dumps(ControllerAsset().summary(), indent=2))
