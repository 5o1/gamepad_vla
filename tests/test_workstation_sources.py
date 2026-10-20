import json
import socket
import pytest

from gamepad_vla.workstation import g20_mapping
from gamepad_vla.workstation.commands import ArmCommand, HandCommand
from gamepad_vla.workstation.protocol import ArmCommandPacket, HandCommandPacket, encode
from gamepad_vla.workstation.sources import ReplaySource, ScriptedSource, UdpSource


def _hand(stamp=1.0):
    return HandCommand("left", {name: 0.1 for name in g20_mapping.G20_JOINT_NAMES}, stamp)


def test_udp_drains_newest_holds_and_expires():
    source = UdpSource("127.0.0.1", 0, stale_after_s=0.2)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        for stamp, value in [(1.0, 255), (2.0, 0)]:
            packet = HandCommandPacket("left", "motor", [], [value] * 20, [], [], stamp)
            sender.sendto(encode(packet), source.bound_address)
        sender.sendto(b"bad", source.bound_address)
        commands = source.poll(10.0)
        assert len(commands) == 1 and commands[0].stamp == 2.0
        assert source.decode_errors == 1
        assert source.poll(10.19) == commands
        assert source.poll(10.21) == []
    finally:
        sender.close()
        source.close()


def test_scripted_zero_order_hold_per_kind_and_side():
    hand = _hand()
    arm = ArmCommand("left", (0.0,) * 7, 2.0)
    source = ScriptedSource([(0.0, hand), (0.1, arm)])
    assert source.poll(5.0) == [hand]
    assert source.poll(5.05) == [hand]
    assert source.poll(5.1) == [hand, arm]


def test_scripted_allows_simultaneous_different_command_types():
    hand = _hand()
    arm = ArmCommand("left", (0.0,) * 7, 2.0)
    assert ScriptedSource([(0.0, hand), (0.0, arm)]).poll(1.0) == [hand, arm]


def test_replay_emits_each_recorded_event_once(tmp_path):
    rows = [
        {"t": 0.0, "kind": "hand", "side": "left",
         "joint_positions": _hand().joint_positions, "stamp": 1.0},
        {"t": 0.2, "kind": "arm", "side": "right", "positions": [0] * 7, "stamp": 2.0},
    ]
    (tmp_path / "commands.jsonl").write_text("".join(json.dumps(row) + "\n" for row in rows))
    source = ReplaySource(tmp_path)
    assert source.poll(10.0) == [_hand()]
    assert source.poll(10.1) == []
    assert source.poll(10.2) == [ArmCommand("right", (0.0,) * 7, 2.0)]


@pytest.mark.parametrize("allowed,accepted", [(None, True), (["127.0.0.1"], True),
                                              (["198.51.100.34"], False),
                                              (["198.51.100.34", "127.0.0.1"], True)])
def test_udp_allowlist_counts_rejections_before_decoding(allowed, accepted):
    source = UdpSource("127.0.0.1", 0, stale_after_s=.2, allow_source=allowed)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
        try:
            sender.sendto(encode(HandCommandPacket("left", "motor", [], [128]*20, [], [], 1)),
                          source.bound_address)
            sender.sendto(b"invalid JSON", source.bound_address)
            assert bool(source.poll(10)) == accepted
            assert source.received_packets == (2 if accepted else 0)
            assert source.rejected_packets == (0 if accepted else 2)
            assert source.decode_errors == (1 if accepted else 0)
            assert source.poll(10.21) == []
        finally:
            source.close()


def test_udp_invalid_allowlist_does_not_open_socket():
    with pytest.raises(ValueError):
        UdpSource("127.0.0.1", 0, stale_after_s=.2, allow_source=["not-an-ip"])
