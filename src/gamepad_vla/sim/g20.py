"""LinkerHand G20 URDF-to-USD conversion and PhysX mimic repair for Isaac Lab.

Adapted from the collection station's Isaac Lab project
(``assist_teleop_local_policy/source/linkerhand_g20_isaaclab/.../assets.py``,
snapshot at the compute node workspace://external/station_20261014).
Converter settings reproduce that project's ``assets/usd/right/config.yaml`` so
the left hand matches the deployed right-hand asset. See THIRD_PARTY_NOTICES.md
for provenance and permissions.
"""

import math
import xml.etree.ElementTree as ET
from pathlib import Path

G20_JOINT_NAMES = (
    "thumb_cmc_roll", "thumb_cmc_yaw", "thumb_cmc_pitch", "thumb_mcp", "thumb_ip",
    "index_mcp_roll", "index_mcp_pitch", "index_pip", "index_dip",
    "middle_mcp_roll", "middle_mcp_pitch", "middle_pip", "middle_dip",
    "ring_mcp_roll", "ring_mcp_pitch", "ring_pip", "ring_dip",
    "pinky_mcp_roll", "pinky_mcp_pitch", "pinky_pip", "pinky_dip",
)
G20_MIMIC_JOINT_NAMES = ("thumb_ip", "index_dip", "middle_dip", "ring_dip", "pinky_dip")


def urdf_joint_table(urdf_path):
    """Return ``{name: (lower, upper, mimic_source, multiplier)}`` for movable joints."""
    table = {}
    for joint in ET.parse(urdf_path).getroot().findall("joint"):
        if joint.get("type") == "fixed":
            continue
        limit, mimic = joint.find("limit"), joint.find("mimic")
        table[joint.get("name")] = (
            float(limit.get("lower")),
            float(limit.get("upper")),
            None if mimic is None else mimic.get("joint"),
            None if mimic is None else float(mimic.get("multiplier", 1.0)),
        )
    if set(table) != set(G20_JOINT_NAMES):
        raise ValueError(f"{urdf_path} does not have the G20 joint set")
    return table


def convert_urdf(urdf_path, usd_dir):
    """Convert a G20 URDF with the station's converter settings; needs a running Kit app."""
    from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg

    urdf_path, usd_dir = Path(urdf_path), Path(usd_dir)
    cfg = UrdfConverterCfg(
        asset_path=str(urdf_path),
        usd_dir=str(usd_dir),
        usd_file_name=urdf_path.with_suffix(".usd").name,
        force_usd_conversion=True,
        make_instanceable=True,
        fix_base=True,
        merge_fixed_joints=False,
        # Isaac's option name is inverted: True keeps <mimic> as PhysxMimicJointAPI.
        convert_mimic_joints_to_normal_joints=True,
        collider_type="convex_hull",
        self_collision=False,
        joint_drive=UrdfConverterCfg.JointDriveCfg(
            drive_type="force",
            target_type="position",
            gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=20.0, damping=0.6),
        ),
    )
    return Path(UrdfConverter(cfg).usd_path)


def repair_mimic_constraints(usd_path, joint_table):
    """Set mimic reference axes, make the coupling hard and restore URDF limits."""
    from pxr import Usd

    stage = Usd.Stage.Open(str(usd_path))
    if stage is None:
        raise RuntimeError(f"cannot open {usd_path}")
    repaired = 0
    for prim in stage.Traverse():
        if prim.GetName() not in G20_MIMIC_JOINT_NAMES:
            continue
        schemas = [s for s in prim.GetAppliedSchemas() if s.startswith("PhysxMimicJointAPI:")]
        if not schemas:
            continue
        axis = schemas[0].split(":", 1)[1]
        targets = prim.GetRelationship(f"physxMimicJoint:{axis}:referenceJoint").GetTargets()
        if len(targets) != 1:
            raise RuntimeError(f"invalid mimic reference for {prim.GetPath()}: {targets}")
        reference_axis = stage.GetPrimAtPath(targets[0]).GetAttribute("physics:axis").Get()
        if reference_axis is None:
            raise RuntimeError(f"missing reference axis for {targets[0]}")
        prim.GetAttribute(f"physxMimicJoint:{axis}:referenceJointAxis").Set(f"rot{reference_axis}")
        # Zero natural frequency and damping ratio select a hard constraint.
        prim.GetAttribute(f"physxMimicJoint:{axis}:naturalFrequency").Set(0.0)
        prim.GetAttribute(f"physxMimicJoint:{axis}:dampingRatio").Set(0.0)
        lower, upper, _, _ = joint_table[prim.GetName()]
        prim.GetAttribute("physics:lowerLimit").Set(math.degrees(lower))
        prim.GetAttribute("physics:upperLimit").Set(math.degrees(upper))
        repaired += 1
    if repaired != len(G20_MIMIC_JOINT_NAMES):
        raise RuntimeError(f"expected {len(G20_MIMIC_JOINT_NAMES)} mimic joints, repaired {repaired}")
    stage.GetRootLayer().Save()
    return repaired
