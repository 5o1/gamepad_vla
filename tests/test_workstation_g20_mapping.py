import numpy as np

from gamepad_vla.sim.g20 import G20_JOINT_NAMES as CANONICAL_NAMES
from gamepad_vla.workstation import g20_mapping as mapping


def test_joint_order_and_quantized_round_trip():
    assert tuple(mapping.G20_JOINT_NAMES) == CANONICAL_NAMES
    rng = np.random.default_rng(7)
    for side in ("left", "right"):
        independent = [name for name in mapping.G20_JOINT_NAMES
                       if name not in mapping.G20_MIMIC_JOINTS[side]]
        joints = {name: rng.uniform(*mapping.G20_JOINT_LIMITS[side][name]) for name in independent}
        full = dict(zip(mapping.G20_JOINT_NAMES, mapping.apply_mimic_joint_positions(
            side, [joints.get(name, 0.0) for name in mapping.G20_JOINT_NAMES])))
        motor = [round(value) for value in mapping.joint_positions_to_motor_positions(side, full)]
        recovered = dict(zip(mapping.G20_JOINT_NAMES,
                             mapping.motor_positions_to_joint_positions(side, motor)))
        for name in independent:
            lower, upper = mapping.G20_JOINT_LIMITS[side][name]
            assert abs(recovered[name] - full[name]) <= (upper - lower) / 255 + 1e-12


def test_motor_endpoints_follow_side_directions():
    for side in ("left", "right"):
        endpoints = {
            0: mapping.motor_positions_to_joint_positions(side, [0] * 20),
            255: mapping.motor_positions_to_joint_positions(side, [255] * 20),
        }
        for motor_index, joint_index in enumerate(mapping.G20_URDF_INDEX[side]):
            if joint_index is None:
                continue
            name = mapping.G20_JOINT_NAMES[joint_index]
            lower, upper = mapping.G20_JOINT_LIMITS[side][name]
            command_lower = mapping.G20_COMMAND_LOWER[side][motor_index]
            command_upper = mapping.G20_COMMAND_UPPER[side][motor_index]
            assert endpoints[command_lower][joint_index] == lower
            assert endpoints[command_upper][joint_index] == upper


def test_side_specific_thumb_mimic_multiplier():
    values = [0.0] * 21
    values[mapping.G20_JOINT_NAMES.index("thumb_mcp")] = 0.5
    left = mapping.apply_mimic_joint_positions("left", values)
    right = mapping.apply_mimic_joint_positions("right", values)
    thumb_ip = mapping.G20_JOINT_NAMES.index("thumb_ip")
    assert left[thumb_ip] == 0.51
    assert right[thumb_ip] == 0.515
