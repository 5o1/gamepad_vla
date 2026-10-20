"""Load both generated URDFs in SAPIEN; compare engine FK, no hardware IO."""

import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import sapien

folder = Path(sys.argv[1])
scene = sapien.Scene()
loader = scene.create_urdf_loader()
loader.fix_root_link = True
left_path = folder / "left/linkerhand_lhg20_left.urdf"
right_path = folder / "right_candidate/linkerhand_lhg20_right_mirrored.urdf"
left = loader.load(str(left_path))
right = loader.load(str(right_path))
assert left is not None and right is not None
root = ET.parse(left_path).getroot()
joints = root.findall("joint")
rng = np.random.default_rng(12)
reflection = np.diag([1.0, -1.0, 1.0, 1.0])
error = 0.0
for _ in range(100):
    q = {
        j.get("name"): rng.uniform(
            float(j.find("limit").get("lower")), float(j.find("limit").get("upper"))
        )
        for j in joints
        if j.find("mimic") is None
    }
    for j in joints:
        m = j.find("mimic")
        if m is not None:
            q[j.get("name")] = q[m.get("joint")] * float(
                m.get("multiplier", 1)
            ) + float(m.get("offset", 0))
    for hand in (left, right):
        hand.set_qpos(np.array([q[j.name] for j in hand.get_active_joints()]))
    lp = {l.name: l.pose.to_transformation_matrix() for l in left.get_links()}
    rp = {l.name: l.pose.to_transformation_matrix() for l in right.get_links()}
    for name in lp:
        error = max(
            error, float(np.max(np.abs(rp[name] - reflection @ lp[name] @ reflection)))
        )
assert error < 1e-4, error
report = dict(
    sapien_version=sapien.__version__,
    loaded_both=True,
    left_dof=left.dof,
    right_dof=right.dof,
    random_poses=100,
    maximum_engine_fk_error=error,
    scope="URDF collision loading and posed FK only; not dynamic/contact validation",
    mimic="Explicitly assigned coupled q for FK test; dynamic coupling NOT verified",
)
(folder / "sapien_audit.json").write_text(json.dumps(report, indent=2))
print(json.dumps(report, indent=2))
