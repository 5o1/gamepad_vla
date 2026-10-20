"""LinkerHand G20 motor/joint mapping from the collection station.

Ported without table or math changes from ``assist_teleop_local_policy``
``manus_mapping.py``, snapshot ``station_20261014`` at the compute node
``workspace://external/``. Reuse was authorized on 2026-10-14.
"""

from collections.abc import Mapping, Sequence

from gamepad_vla.sim.g20 import G20_JOINT_NAMES as _CANONICAL_JOINT_NAMES


G20_MOTOR_COUNT = 20
G20_JOINT_NAMES = [
    "thumb_cmc_roll", "thumb_cmc_yaw", "thumb_cmc_pitch", "thumb_mcp", "thumb_ip",
    "index_mcp_roll", "index_mcp_pitch", "index_pip", "index_dip",
    "middle_mcp_roll", "middle_mcp_pitch", "middle_pip", "middle_dip",
    "ring_mcp_roll", "ring_mcp_pitch", "ring_pip", "ring_dip",
    "pinky_mcp_roll", "pinky_mcp_pitch", "pinky_pip", "pinky_dip",
]
if tuple(G20_JOINT_NAMES) != _CANONICAL_JOINT_NAMES:
    raise RuntimeError("station G20 joint order differs from gamepad_vla.sim.g20")

G20_DEFAULT_MOTOR_POSITION = [
    255.0, 255.0, 255.0, 255.0, 255.0, 255.0, 193.0, 148.0, 105.0, 42.0,
    245.0, 255.0, 255.0, 255.0, 255.0, 255.0, 255.0, 255.0, 255.0, 255.0,
]
LOGICAL_SLOT_FROM_JOINT = {
    "index_mcp_roll": 0, "index_mcp_pitch": 1, "index_pip": 2, "index_dip": 3,
    "pinky_mcp_roll": 4, "pinky_mcp_pitch": 5, "pinky_pip": 6, "pinky_dip": 7,
    "middle_mcp_roll": 8, "middle_mcp_pitch": 9, "middle_pip": 10, "middle_dip": 11,
    "ring_mcp_roll": 12, "ring_mcp_pitch": 13, "ring_pip": 14, "ring_dip": 15,
    "thumb_cmc_roll": 16, "thumb_cmc_yaw": 17, "thumb_cmc_pitch": 18,
    "thumb_mcp": 19, "thumb_ip": 20,
}
JOINT_FROM_LOGICAL_SLOT = {slot: name for name, slot in LOGICAL_SLOT_FROM_JOINT.items()}
G20_COMMAND_LOWER = {
    "right": [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, None, None, None, None, 255, 255, 255, 255, 255],
    "left": [255, 255, 255, 255, 255, 0, 0, 0, 0, 0, 255, None, None, None, None, 255, 255, 255, 255, 255],
}
G20_COMMAND_UPPER = {
    "right": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, None, None, None, None, 0, 0, 0, 0, 0],
    "left": [0, 0, 0, 0, 0, 255, 255, 255, 255, 255, 0, None, None, None, None, 0, 0, 0, 0, 0],
}
G20_COMMAND_SOURCE_INDEX = {
    "right": [18, 1, 9, 13, 5, 16, 0, 8, 12, 4, 17, None, None, None, None, 19, 2, 10, 14, 6],
    "left": [18, 1, 9, 13, 5, 16, 0, 8, 12, 4, 17, None, None, None, None, 19, 2, 10, 14, 6],
}
G20_URDF_INDEX = {
    "right": [2, 6, 10, 14, 18, 0, 5, 9, 13, 17, 1, None, None, None, None, 3, 7, 11, 15, 19],
    "left": [2, 6, 10, 14, 18, 0, 5, 9, 13, 17, 1, None, None, None, None, 3, 7, 11, 15, 19],
}
G20_JOINT_LIMITS = {
    "right": {
        "thumb_cmc_roll": (0.0, 1.39), "thumb_cmc_yaw": (0.0, 1.57),
        "thumb_cmc_pitch": (0.0, 0.83), "thumb_mcp": (0.0, 1.25), "thumb_ip": (0.0, 1.29),
        "index_mcp_roll": (-0.23, 0.23), "index_mcp_pitch": (0.0, 1.22),
        "index_pip": (0.0, 1.75), "index_dip": (0.0, 1.55),
        "middle_mcp_roll": (-0.23, 0.23), "middle_mcp_pitch": (0.0, 1.22),
        "middle_pip": (0.0, 1.75), "middle_dip": (0.0, 1.55),
        "ring_mcp_roll": (-0.23, 0.23), "ring_mcp_pitch": (0.0, 1.22),
        "ring_pip": (0.0, 1.75), "ring_dip": (0.0, 1.55),
        "pinky_mcp_roll": (-0.23, 0.23), "pinky_mcp_pitch": (0.0, 1.22),
        "pinky_pip": (0.0, 1.75), "pinky_dip": (0.0, 1.55),
    },
    "left": {
        "thumb_cmc_roll": (0.0, 1.4), "thumb_cmc_yaw": (0.0, 1.57),
        "thumb_cmc_pitch": (0.0, 0.84), "thumb_mcp": (0.0, 1.26), "thumb_ip": (0.0, 1.29),
        "index_mcp_roll": (-0.23, 0.23), "index_mcp_pitch": (0.0, 1.22),
        "index_pip": (0.0, 1.74), "index_dip": (0.0, 1.55),
        "middle_mcp_roll": (-0.23, 0.23), "middle_mcp_pitch": (0.0, 1.22),
        "middle_pip": (0.0, 1.74), "middle_dip": (0.0, 1.55),
        "ring_mcp_roll": (-0.23, 0.23), "ring_mcp_pitch": (0.0, 1.22),
        "ring_pip": (0.0, 1.74), "ring_dip": (0.0, 1.55),
        "pinky_mcp_roll": (-0.23, 0.23), "pinky_mcp_pitch": (0.0, 1.22),
        "pinky_pip": (0.0, 1.74), "pinky_dip": (0.0, 1.55),
    },
}
G20_MIMIC_JOINTS = {
    "right": {"thumb_ip": ("thumb_mcp", 1.03), "index_dip": ("index_pip", 0.89),
              "middle_dip": ("middle_pip", 0.89), "ring_dip": ("ring_pip", 0.89),
              "pinky_dip": ("pinky_pip", 0.89)},
    "left": {"thumb_ip": ("thumb_mcp", 1.02), "index_dip": ("index_pip", 0.89),
             "middle_dip": ("middle_pip", 0.89), "ring_dip": ("ring_pip", 0.89),
             "pinky_dip": ("pinky_pip", 0.89)},
}


def motor_positions_to_joint_positions(hand: str, motor_positions: Sequence[float],
                                       joint_names: Sequence[str] = G20_JOINT_NAMES) -> list[float]:
    """Convert 20 LinkerHand SDK motor values into URDF joint radians."""
    _validate_hand(hand)
    if len(motor_positions) < G20_MOTOR_COUNT:
        raise ValueError(f"G20 motor command requires {G20_MOTOR_COUNT} values, got {len(motor_positions)}")
    logical_qpos = [0.0] * len(LOGICAL_SLOT_FROM_JOINT)
    for motor_index in range(G20_MOTOR_COUNT):
        source_index = G20_COMMAND_SOURCE_INDEX[hand][motor_index]
        urdf_index = G20_URDF_INDEX[hand][motor_index]
        cmd_lower = G20_COMMAND_LOWER[hand][motor_index]
        cmd_upper = G20_COMMAND_UPPER[hand][motor_index]
        if source_index is None or urdf_index is None or cmd_lower is None or cmd_upper is None:
            continue
        joint_name = G20_JOINT_NAMES[urdf_index]
        lower, upper = G20_JOINT_LIMITS[hand][joint_name]
        command = _clamp(float(motor_positions[motor_index]), min(cmd_lower, cmd_upper), max(cmd_lower, cmd_upper))
        logical_qpos[source_index] = _scale(command, float(cmd_lower), float(cmd_upper), lower, upper)
    _apply_mimic(hand, logical_qpos)
    return [_clamp(logical_qpos[LOGICAL_SLOT_FROM_JOINT[name]], *G20_JOINT_LIMITS[hand][name])
            for name in joint_names]


def apply_mimic_joint_positions(hand: str, joint_positions: Sequence[float],
                                joint_names: Sequence[str] = G20_JOINT_NAMES) -> list[float]:
    """Fill mimic joint positions for legacy software-coupled modes."""
    _validate_hand(hand)
    by_name = {name: float(value) for name, value in zip(joint_names, joint_positions)}
    for mimic_name, (source_name, multiplier) in G20_MIMIC_JOINTS[hand].items():
        by_name[mimic_name] = _clamp(by_name.get(source_name, 0.0) * multiplier,
                                     *G20_JOINT_LIMITS[hand][mimic_name])
    return [_clamp(by_name.get(name, 0.0), *G20_JOINT_LIMITS[hand][name]) for name in joint_names]


def joint_positions_to_motor_positions(hand: str, joint_positions: Sequence[float] | Mapping[str, float],
                                       joint_names: Sequence[str] = G20_JOINT_NAMES) -> list[float]:
    """Convert URDF joint radians back into 20 LinkerHand SDK motor values."""
    _validate_hand(hand)
    if isinstance(joint_positions, Mapping):
        by_name = {name: float(value) for name, value in joint_positions.items()}
    else:
        by_name = {name: float(value) for name, value in zip(joint_names, joint_positions)}
    motor_positions = list(G20_DEFAULT_MOTOR_POSITION)
    for motor_index in range(G20_MOTOR_COUNT):
        source_index = G20_COMMAND_SOURCE_INDEX[hand][motor_index]
        cmd_lower = G20_COMMAND_LOWER[hand][motor_index]
        cmd_upper = G20_COMMAND_UPPER[hand][motor_index]
        if source_index is None or cmd_lower is None or cmd_upper is None:
            continue
        joint_name = JOINT_FROM_LOGICAL_SLOT[source_index]
        lower, upper = G20_JOINT_LIMITS[hand][joint_name]
        qpos = _clamp(by_name.get(joint_name, 0.0), lower, upper)
        motor_positions[motor_index] = _clamp(
            _scale(qpos, lower, upper, float(cmd_lower), float(cmd_upper)),
            min(cmd_lower, cmd_upper), max(cmd_lower, cmd_upper))
    return motor_positions


def _apply_mimic(hand, logical_qpos):
    for mimic_name, (source_name, multiplier) in G20_MIMIC_JOINTS[hand].items():
        mimic_slot = LOGICAL_SLOT_FROM_JOINT[mimic_name]
        source_slot = LOGICAL_SLOT_FROM_JOINT[source_name]
        logical_qpos[mimic_slot] = _clamp(logical_qpos[source_slot] * multiplier,
                                           *G20_JOINT_LIMITS[hand][mimic_name])


def _scale(value, in_lower, in_upper, out_lower, out_upper):
    if in_upper == in_lower:
        return out_lower
    return (value - in_lower) * (out_upper - out_lower) / (in_upper - in_lower) + out_lower


def _clamp(value, lower, upper):
    return max(lower, min(upper, value))


def _validate_hand(hand):
    if hand not in {"left", "right"}:
        raise ValueError(f"hand must be 'left' or 'right', got {hand!r}")
