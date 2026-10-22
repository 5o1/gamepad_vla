"""Normalized workstation commands in joint radians."""

import math
from dataclasses import dataclass

from gamepad_vla.workstation import g20_mapping
from gamepad_vla.workstation.protocol import ArmCommandPacket, HandCommandPacket


@dataclass(frozen=True)
class HandCommand:
    side: str
    joint_positions: dict[str, float]
    stamp: float
    motor_positions: tuple[float, ...] | None = None


@dataclass(frozen=True)
class ArmCommand:
    side: str
    positions: tuple[float, ...]
    stamp: float
    raw_master_deg: tuple[float, ...] | None = None
    clamp_flags: dict | None = None


def _finite(value, label):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{label} must be finite")
    return value


def hand_command_from_packet(packet):
    """Convert a station hand packet to all 21 G20 joint positions."""
    if not isinstance(packet, HandCommandPacket):
        raise TypeError("packet must be a HandCommandPacket")
    if packet.format == "motor":
        values = g20_mapping.motor_positions_to_joint_positions(packet.hand, packet.position)
    elif packet.format == "joint-radians":
        if len(packet.name) != len(packet.position):
            raise ValueError("joint-radians name and position lengths differ")
        by_name = dict(zip(packet.name, packet.position))
        independent = [name for name in g20_mapping.G20_JOINT_NAMES
                       if name not in g20_mapping.G20_MIMIC_JOINTS[packet.hand]]
        missing = [name for name in independent if name not in by_name]
        if missing:
            raise ValueError(f"joint-radians packet is missing independent joints: {missing}")
        independent_values = [by_name.get(name, 0.0) for name in g20_mapping.G20_JOINT_NAMES]
        values = g20_mapping.apply_mimic_joint_positions(
            packet.hand, independent_values, g20_mapping.G20_JOINT_NAMES)
    else:
        raise ValueError(f"unsupported hand command format {packet.format!r}")
    return HandCommand(packet.hand, dict(zip(g20_mapping.G20_JOINT_NAMES, values)),
                       _finite(packet.stamp, "stamp"),
                       tuple(packet.position) if packet.format == "motor" else None)


def arm_command_from_packet(packet, mapper=None):
    """Convert an arm packet to the internal seven-radian command."""
    if not isinstance(packet, ArmCommandPacket):
        raise TypeError("packet must be an ArmCommandPacket")
    if packet.side not in {"left", "right"}:
        raise ValueError("arm side must be 'left' or 'right'")
    if len(packet.position) != 7:
        raise ValueError("arm command must have exactly 7 positions")
    if packet.representation == "master-deg":
        if mapper is None:
            raise ValueError("master-deg requires an arm mapper with explicit startup pose")
        return mapper.command(packet.side, packet.position, packet.stamp)
    return ArmCommand(packet.side, tuple(_finite(value, "arm position") for value in packet.position),
                      _finite(packet.stamp, "stamp"))
