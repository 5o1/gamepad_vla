import json

import pytest

from gamepad_vla.workstation import g20_mapping
from gamepad_vla.workstation.commands import arm_command_from_packet, hand_command_from_packet
from gamepad_vla.workstation.protocol import (
    ArmCommandPacket,
    ArmStatePacket,
    GloveTipsPacket,
    HandCommandPacket,
    HandStatePacket,
    decode,
    encode,
)


def test_decodes_station_relay_motor_packet_exactly():
    payload = {
        "type": "command", "hand": "left", "format": "motor", "name": [],
        "position": [-1.2, 2.5, *range(2, 22)], "velocity": [], "effort": [], "stamp": 4.5,
    }
    data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
    packet = decode(data)
    assert packet.position == [0.0, 2.0, *[float(value) for value in range(2, 20)]]


@pytest.mark.parametrize("packet", [
    HandCommandPacket("right", "joint-radians", ["j"], [0.2], [], [], 1.0),
    GloveTipsPacket("left", [0, 0, 0], [1, 2, 3], [4, 5, 6], 2.0),
    HandStatePacket("right", ["j"], [0.1], [255] * 20),
    ArmCommandPacket("left", [0] * 7, 3.0),
    ArmStatePacket("right", [1] * 7, 4.0),
])
def test_packet_round_trip(packet):
    assert decode(encode(packet)) == packet
    assert encode(packet).startswith(b'{"type":')


@pytest.mark.parametrize("data,message", [
    (b"not-json", "valid UTF-8 JSON"),
    (b'{"type":"mystery"}', "unknown packet"),
    (b'{"type":"arm_command","side":"left","position":[0],"stamp":0}', "exactly 7"),
    (b'{"type":"command","hand":"left","format":"motor","name":[],"position":[],"velocity":[],"effort":[],"stamp":0}', "at least 20"),
    (b'{"type":"state","hand":"left","joint_names":["a"],"joint_positions":[],"motor_positions":[]}', "equal lengths"),
])
def test_rejects_invalid_packets(data, message):
    with pytest.raises(ValueError, match=message):
        decode(data)


def test_command_conversion_requires_independent_joints_and_recomputes_mimics():
    independent = [name for name in g20_mapping.G20_JOINT_NAMES
                   if name not in g20_mapping.G20_MIMIC_JOINTS["left"]]
    positions = [0.2] * len(independent)
    packet = HandCommandPacket("left", "joint-radians", independent, positions, [], [], 8.0)
    command = hand_command_from_packet(packet)
    assert len(command.joint_positions) == 21
    assert command.joint_positions["thumb_ip"] == pytest.approx(0.204)
    with pytest.raises(ValueError, match="missing independent"):
        hand_command_from_packet(HandCommandPacket("left", "joint-radians", independent[:-1],
                                                   positions[:-1], [], [], 8.0))
    arm = arm_command_from_packet(ArmCommandPacket("right", list(range(7)), 9.0))
    assert arm.positions == tuple(float(value) for value in range(7))
