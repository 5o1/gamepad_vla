import sys
import types

import pytest

from gamepad_vla.workstation.backends import FanoutBackend, NullBackend, RealRos2Backend
from gamepad_vla.workstation.backends_registry import BACKENDS
from gamepad_vla.workstation.commands import ArmCommand, HandCommand
from gamepad_vla.workstation.g20_mapping import G20_JOINT_NAMES


def _hand():
    return HandCommand("left", {name: 0.0 for name in G20_JOINT_NAMES}, 1.0)


class FailingBackend(NullBackend):
    name = "failing"

    def __init__(self):
        super().__init__()
        self.held = 0

    def apply(self, commands):
        raise RuntimeError("apply failed")

    def hold(self):
        self.held += 1


class HeldNull(NullBackend):
    def __init__(self):
        super().__init__()
        self.held = 0

    def hold(self):
        self.held += 1


def test_null_and_fanout_observations_and_failure_hold():
    primary, other = NullBackend(), NullBackend()
    primary.apply([_hand()])
    observation = FanoutBackend(primary, other).observe()
    assert observation.backend == "null" and observation.extra["null"].backend == "null"
    held, failing = HeldNull(), FailingBackend()
    with pytest.raises(RuntimeError, match="apply failed"):
        FanoutBackend(held, failing).apply([_hand()])
    assert held.held == 1 and failing.held == 1


class JointState:
    def __init__(self):
        self.name = []
        self.position = []
        self.velocity = []
        self.effort = []


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class Node:
    def __init__(self):
        self.publishers = {}
        self.publisher_counts = {}
        self.destroyed = False

    def create_publisher(self, _type, topic, _depth):
        publisher = Publisher()
        self.publishers[topic] = publisher
        self.publisher_counts.setdefault(topic, 1)
        return publisher

    def create_subscription(self, _type, topic, callback, _depth):
        return (topic, callback)

    def count_publishers(self, topic):
        return self.publisher_counts[topic]

    def destroy_node(self):
        self.destroyed = True


def test_real_backend_requires_gate_before_ros_import():
    with pytest.raises(RuntimeError, match="enable_real"):
        RealRos2Backend(lambda: Node(), enable_real=False, arm_units="rad", sides=("left",))


def test_real_backend_with_fake_ros(monkeypatch):
    rclpy = types.ModuleType("rclpy")
    rclpy.ok = lambda: True
    rclpy.spin_once = lambda node, timeout_sec: None
    msg = types.ModuleType("sensor_msgs.msg")
    msg.JointState = JointState
    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs.msg = msg
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "sensor_msgs", sensor_msgs)
    monkeypatch.setitem(sys.modules, "sensor_msgs.msg", msg)
    node = Node()
    backend = RealRos2Backend(lambda: node, enable_real=True, arm_units="deg", sides=("left",))
    assert set(node.publishers) == {"/cb_left_hand_control_cmd", "/left_arm_joint_control"}
    backend.apply([_hand(), ArmCommand("left", (1.0,) * 7, 2.0)])
    assert backend.observe().hand_joint_positions == {}  # commands are not measurements
    with pytest.raises(ValueError, match="side 'right'"):
        backend.apply([ArmCommand("right", (0.0,) * 7, 3.0)])
    assert len(node.publishers["/cb_left_hand_control_cmd"].messages[0].position) == 20
    assert node.publishers["/left_arm_joint_control"].messages[0].position[0] == pytest.approx(57.2958)
    node.publisher_counts["/cb_left_hand_control_cmd"] = 2
    with pytest.raises(RuntimeError, match="another publisher"):
        backend.apply([_hand()])
    backend.close()
    assert node.destroyed
    assert set(BACKENDS) == {"null", "real", "sapien"}
    assert isinstance(BACKENDS["null"](None), NullBackend)
