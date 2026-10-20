"""Batched forward kinematics for one serial URDF chain, numpy only.

Used for kinematic feasibility checks (e.g. thumb reach over the D-pad) that do
not need a physics engine. Supports revolute/prismatic/fixed joints and
first-order ``<mimic>`` joints whose source lies on the same chain.
"""

import xml.etree.ElementTree as ET

import numpy as np


def rpy_to_matrix(rpy):
    """URDF fixed-axis roll-pitch-yaw: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def axis_angle_to_matrix(axis, angles):
    """Rotations about a unit ``axis`` by each angle; returns ``(N, 3, 3)``."""
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    s, c = np.sin(angles)[:, None, None], np.cos(angles)[:, None, None]
    return np.eye(3) + s * k + (1 - c) * (k @ k)


def _origin(joint):
    element = joint.find("origin")
    transform = np.eye(4)
    if element is not None:
        transform[:3, :3] = rpy_to_matrix([float(v) for v in element.get("rpy", "0 0 0").split()])
        transform[:3, 3] = [float(v) for v in element.get("xyz", "0 0 0").split()]
    return transform


class SerialChain:
    """The joints from the URDF root link down to ``tip_link``.

    ``active`` lists the independent (non-fixed, non-mimic) joints in root-to-tip
    order; ``forward`` takes their positions and fills mimic joints as
    ``multiplier * source + offset``.
    """

    def __init__(self, urdf_path, tip_link):
        by_child = {j.find("child").get("link"): j
                    for j in ET.parse(urdf_path).getroot().findall("joint")}
        joints, link = [], tip_link
        while link in by_child:
            joint = by_child[link]
            joints.append(joint)
            link = joint.find("parent").get("link")
        if not joints:
            raise ValueError(f"no joints lead to {tip_link!r}")
        self.root_link, self.tip_link = link, tip_link
        self.joints = []
        for joint in reversed(joints):
            kind = joint.get("type")
            axis = joint.find("axis")
            limit = joint.find("limit")
            mimic = joint.find("mimic")
            self.joints.append(dict(
                name=joint.get("name"),
                child=joint.find("child").get("link"),
                type=kind,
                origin=_origin(joint),
                axis=np.array([float(v) for v in (axis.get("xyz") if axis is not None else "1 0 0").split()]),
                limits=None if limit is None or kind == "fixed"
                else (float(limit.get("lower")), float(limit.get("upper"))),
                mimic=None if mimic is None else (mimic.get("joint"),
                                                  float(mimic.get("multiplier", 1.0)),
                                                  float(mimic.get("offset", 0.0))),
            ))
        names = [j["name"] for j in self.joints]
        for joint in self.joints:
            if joint["mimic"] and joint["mimic"][0] not in names:
                raise ValueError(f"mimic source of {joint['name']} is not on the chain")
        self.active = [j["name"] for j in self.joints if j["type"] != "fixed" and not j["mimic"]]

    @property
    def active_limits(self):
        """``(lower, upper)`` arrays for the active joints."""
        limits = {j["name"]: j["limits"] for j in self.joints}
        return tuple(np.array(v) for v in zip(*(limits[n] for n in self.active)))

    def joint_positions(self, q):
        """Expand ``(N, len(active))`` to a dict of every movable joint's positions."""
        q = np.atleast_2d(np.asarray(q, dtype=float))
        if q.shape[1] != len(self.active):
            raise ValueError(f"expected {len(self.active)} active joints, got {q.shape[1]}")
        values = dict(zip(self.active, q.T))
        for joint in self.joints:
            if joint["mimic"]:
                source, multiplier, offset = joint["mimic"]
                values[joint["name"]] = multiplier * values[source] + offset
        return values

    def forward(self, q):
        """Tip-link poses in the root-link frame, shape ``(N, 4, 4)``."""
        return self.link_poses(q)[self.tip_link]

    def link_poses(self, q):
        """Poses of every link below the root, ``{link: (N, 4, 4)}`` in the root frame."""
        values = self.joint_positions(q)
        n = len(next(iter(values.values())))
        pose = np.broadcast_to(np.eye(4), (n, 4, 4)).copy()
        poses = {}
        for joint in self.joints:
            pose = pose @ joint["origin"]
            if joint["type"] in ("revolute", "continuous"):
                motion = np.broadcast_to(np.eye(4), (n, 4, 4)).copy()
                motion[:, :3, :3] = axis_angle_to_matrix(joint["axis"], values[joint["name"]])
                pose = pose @ motion
            elif joint["type"] == "prismatic":
                motion = np.broadcast_to(np.eye(4), (n, 4, 4)).copy()
                axis = joint["axis"] / np.linalg.norm(joint["axis"])
                motion[:, :3, 3] = values[joint["name"]][:, None] * axis
                pose = pose @ motion
            elif joint["type"] != "fixed":
                raise ValueError(f"unsupported joint type {joint['type']!r}")
            poses[joint["child"]] = pose
        return poses
