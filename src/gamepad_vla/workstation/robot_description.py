"""Backend-neutral composition of the workcell arm and hand URDF chains."""

import math
import xml.etree.ElementTree as ET
from dataclasses import replace
from pathlib import Path

from gamepad_vla.workstation.spec import pose_to_matrix


def matrix_to_rpy(rotation):
    pitch = math.asin(max(-1.0, min(1.0, -rotation[2, 0])))
    roll = math.atan2(rotation[2, 1], rotation[2, 2])
    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    return roll, pitch, yaw


def _absolute_meshes(element, base_dir):
    for mesh in element.iter("mesh"):
        filename = mesh.get("filename")
        if filename and not filename.startswith(("/", "package://")):
            mesh.set("filename", str((base_dir / filename).resolve()))


def compose_robot_urdf(spec, path, side="left"):
    """Write one URDF: the spec's arm chain (base..tip) with the G20 hand fixed at the mount."""
    if side == "right":
        spec = replace(spec, left_arm=spec.right_arm, left_hand=spec.right_hand)
    arm_urdf, hand_urdf = Path(spec.left_arm.urdf), Path(spec.left_hand.urdf)
    arm_root = ET.parse(arm_urdf).getroot()
    by_child = {j.find("child").get("link"): j for j in arm_root.findall("joint")}
    chain, link = [], spec.left_arm.tip_link
    while link != spec.left_arm.base_link:
        if link not in by_child:
            raise ValueError(f"{spec.left_arm.tip_link} does not descend from {spec.left_arm.base_link}")
        chain.append(by_child[link])
        link = by_child[link].find("parent").get("link")
    chain.reverse()
    names = [j.get("name") for j in chain if j.get("type") != "fixed"]
    if names != list(spec.left_arm.joint_names):
        raise ValueError(f"arm chain joints {names} differ from spec {spec.left_arm.joint_names}")
    keep = {spec.left_arm.base_link} | {j.find("child").get("link") for j in chain}

    robot = ET.Element("robot", name="workcell_left_arm_g20")
    for element in arm_root.findall("link"):
        if element.get("name") in keep:
            _absolute_meshes(element, arm_urdf.parent)
            robot.append(element)
    for joint in chain:
        robot.append(joint)
    hand_root = ET.parse(hand_urdf).getroot()
    if side == "right":
        # Keep station hand names internally consistent, distinct from the left
        # articulation for segmentation and contact evidence.
        for element in hand_root.iter():
            for attribute in ("name", "link", "joint"):
                if element.tag in {"link", "joint", "parent", "child", "mimic"} and attribute in element.attrib:
                    element.set(attribute, "right_" + element.get(attribute))
    for element in list(hand_root.findall("link")) + list(hand_root.findall("joint")):
        _absolute_meshes(element, hand_urdf.parent)
        robot.append(element)
    mount = pose_to_matrix(spec.left_hand.mount)
    joint = ET.SubElement(robot, "joint", name="workcell_hand_mount", type="fixed")
    ET.SubElement(joint, "parent", link=spec.left_arm.tip_link)
    ET.SubElement(joint, "child", link=("right_" if side == "right" else "") + spec.left_hand.root_link)
    ET.SubElement(joint, "origin", xyz=" ".join(f"{v:.9g}" for v in mount[:3, 3]),
                  rpy=" ".join(f"{v:.9g}" for v in matrix_to_rpy(mount[:3, :3])))
    Path(path).write_text(ET.tostring(robot, encoding="unicode"))
    return Path(path)
