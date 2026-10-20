"""Compact UDP JSON packets compatible with the station G20 relay schema."""

import json
import math
from dataclasses import asdict, dataclass, field


@dataclass(frozen=True)
class HandCommandPacket:
    hand: str
    format: str
    name: list[str]
    position: list[float]
    velocity: list[float]
    effort: list[float]
    stamp: float
    type: str = field(default="command", init=False)


@dataclass(frozen=True)
class GloveTipsPacket:
    hand: str
    palm: list[float]
    thumb: list[float]
    index: list[float]
    stamp: float
    type: str = field(default="glove_tips", init=False)


@dataclass(frozen=True)
class HandStatePacket:
    hand: str
    joint_names: list[str]
    joint_positions: list[float]
    motor_positions: list[float]
    type: str = field(default="state", init=False)


@dataclass(frozen=True)
class ArmCommandPacket:
    side: str
    position: list[float]
    stamp: float
    type: str = field(default="arm_command", init=False)


@dataclass(frozen=True)
class ArmStatePacket:
    side: str
    position: list[float]
    stamp: float
    type: str = field(default="arm_state", init=False)


PACKET_TYPES = (HandCommandPacket, GloveTipsPacket, HandStatePacket, ArmCommandPacket, ArmStatePacket)


def _object(data, keys, packet_type):
    if not isinstance(data, dict):
        raise ValueError("packet must be a JSON object")
    unknown = set(data) - set(keys)
    missing = set(keys) - set(data)
    if unknown:
        raise ValueError(f"{packet_type} packet has unknown keys: {sorted(unknown)}")
    if missing:
        raise ValueError(f"{packet_type} packet is missing keys: {sorted(missing)}")


def _side(value, label):
    if not isinstance(value, str) or value not in {"left", "right"}:
        raise ValueError(f"{label} must be 'left' or 'right'")
    return value


def _number(value, label):
    if isinstance(value, bool):
        raise ValueError(f"{label} must be a finite number")
    try:
        value = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be a finite number") from exc
    if not math.isfinite(value):
        raise ValueError(f"{label} must be a finite number")
    return value


def _numbers(value, label, length=None, minimum=None):
    if not isinstance(value, list):
        raise ValueError(f"{label} must be a list")
    if length is not None and len(value) != length:
        raise ValueError(f"{label} must have exactly {length} values")
    if minimum is not None and len(value) < minimum:
        raise ValueError(f"{label} must have at least {minimum} values")
    return [_number(item, f"{label}[{index}]") for index, item in enumerate(value)]


def _strings(value, label):
    if not isinstance(value, list) or not all(isinstance(item, str) for item in value):
        raise ValueError(f"{label} must be a list of strings")
    return list(value)


def _normalize(packet):
    if isinstance(packet, HandCommandPacket):
        if not isinstance(packet.format, str) or packet.format not in {"motor", "joint-radians"}:
            raise ValueError("command format must be 'motor' or 'joint-radians'")
        positions = _numbers(packet.position, "position", minimum=20 if packet.format == "motor" else None)
        if packet.format == "motor":
            positions = [float(min(max(round(value), 0), 255)) for value in positions[:20]]
        return HandCommandPacket(
            _side(packet.hand, "hand"), packet.format, _strings(packet.name, "name"), positions,
            _numbers(packet.velocity, "velocity"), _numbers(packet.effort, "effort"),
            _number(packet.stamp, "stamp"),
        )
    if isinstance(packet, GloveTipsPacket):
        return GloveTipsPacket(
            _side(packet.hand, "hand"), _numbers(packet.palm, "palm", 3),
            _numbers(packet.thumb, "thumb", 3), _numbers(packet.index, "index", 3),
            _number(packet.stamp, "stamp"),
        )
    if isinstance(packet, HandStatePacket):
        names = _strings(packet.joint_names, "joint_names")
        positions = _numbers(packet.joint_positions, "joint_positions")
        if len(names) != len(positions):
            raise ValueError("joint_names and joint_positions must have equal lengths")
        return HandStatePacket(_side(packet.hand, "hand"), names, positions,
                               _numbers(packet.motor_positions, "motor_positions"))
    if isinstance(packet, (ArmCommandPacket, ArmStatePacket)):
        cls = type(packet)
        return cls(_side(packet.side, "side"), _numbers(packet.position, "position", 7),
                   _number(packet.stamp, "stamp"))
    raise TypeError(f"unsupported packet type {type(packet).__name__}")


def decode(data):
    """Decode and validate one UTF-8 JSON datagram."""
    try:
        payload = json.loads(data.decode("utf-8"))
    except (AttributeError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError("datagram is not valid UTF-8 JSON") from exc
    if not isinstance(payload, dict) or not isinstance(payload.get("type"), str):
        raise ValueError("packet must have a string type")
    packet_type = payload["type"]
    if packet_type == "command":
        keys = ("type", "hand", "format", "name", "position", "velocity", "effort", "stamp")
        _object(payload, keys, packet_type)
        packet = HandCommandPacket(payload["hand"], payload["format"], payload["name"],
                                   payload["position"], payload["velocity"], payload["effort"],
                                   payload["stamp"])
    elif packet_type == "glove_tips":
        keys = ("type", "hand", "palm", "thumb", "index", "stamp")
        _object(payload, keys, packet_type)
        packet = GloveTipsPacket(payload["hand"], payload["palm"], payload["thumb"],
                                 payload["index"], payload["stamp"])
    elif packet_type == "state":
        keys = ("type", "hand", "joint_names", "joint_positions", "motor_positions")
        _object(payload, keys, packet_type)
        packet = HandStatePacket(payload["hand"], payload["joint_names"],
                                 payload["joint_positions"], payload["motor_positions"])
    elif packet_type in {"arm_command", "arm_state"}:
        keys = ("type", "side", "position", "stamp")
        _object(payload, keys, packet_type)
        cls = ArmCommandPacket if packet_type == "arm_command" else ArmStatePacket
        packet = cls(payload["side"], payload["position"], payload["stamp"])
    else:
        raise ValueError(f"unknown packet type {packet_type!r}")
    return _normalize(packet)


def encode(packet):
    """Validate and encode one packet using the relay's compact JSON form."""
    packet = _normalize(packet)
    payload = asdict(packet)
    payload = {"type": payload.pop("type"), **payload}
    return json.dumps(payload, separators=(",", ":")).encode("utf-8")
