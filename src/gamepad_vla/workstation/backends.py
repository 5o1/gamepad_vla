"""Workstation backend interface, fanout semantics, ROS2 bridge, and test backend."""

import importlib
import math
import time
from dataclasses import dataclass, field, replace
from typing import Protocol

import numpy as np

from gamepad_vla.workstation.commands import ArmCommand, HandCommand
from gamepad_vla.workstation.g20_mapping import G20_JOINT_NAMES, joint_positions_to_motor_positions


@dataclass(frozen=True)
class Observation:
    stamp: float
    hand_joint_positions: dict[str, float]
    arm_joint_positions: tuple[float, ...] | None
    hand_root_pose_world: tuple[float, ...] | None
    gamepad_signal: np.ndarray | None
    gamepad_mask: np.ndarray | None
    backend: str
    extra: dict[str, "Observation"] = field(default_factory=dict)


class Backend(Protocol):
    def reset(self) -> None: ...
    def apply(self, commands) -> None: ...
    def step(self, dt) -> None: ...
    def observe(self) -> Observation: ...
    def close(self) -> None: ...


class FanoutBackend:
    """Apply and step every backend as one failure domain."""

    def __init__(self, primary, *others):
        self.primary = primary
        self.others = tuple(others)
        self.name = "fanout"

    @property
    def backends(self):
        return (self.primary, *self.others)

    def reset(self):
        for backend in self.backends:
            backend.reset()

    def apply(self, commands):
        commands = list(commands)
        try:
            for backend in self.backends:
                backend.apply(commands)
        except Exception:
            self._hold_all()
            raise

    def step(self, dt):
        try:
            for backend in self.backends:
                backend.step(dt)
        except Exception:
            self._hold_all()
            raise

    def observe(self):
        primary = self.primary.observe()
        extra = dict(primary.extra)
        for backend in self.others:
            observation = backend.observe()
            extra[observation.backend] = observation
        return replace(primary, extra=extra)

    def close(self):
        for backend in reversed(self.backends):
            backend.close()

    def _hold_all(self):
        for backend in self.backends:
            hold = getattr(backend, "hold", None)
            if hold is not None:
                try:
                    hold()
                except Exception:
                    pass


class NullBackend:
    """Echo the latest commands into an observation without external IO."""

    name = "null"

    def __init__(self):
        self.reset()

    def reset(self):
        self._hand = {name: 0.0 for name in G20_JOINT_NAMES}
        self._arm = None
        self._stamp = 0.0

    def apply(self, commands):
        for command in commands:
            if isinstance(command, HandCommand):
                self._hand = dict(command.joint_positions)
                self._stamp = command.stamp
            elif isinstance(command, ArmCommand):
                self._arm = tuple(command.positions)
                self._stamp = command.stamp

    def step(self, dt):
        self._stamp += float(dt)

    def observe(self):
        return Observation(self._stamp, dict(self._hand), self._arm, None, None, None, self.name)

    def hold(self):
        pass

    def close(self):
        pass


class RealRos2Backend:
    """Explicitly enabled ROS2 command publisher with publisher-conflict checks."""

    name = "real"

    def __init__(self, node_factory=None, *, enable_real: bool, arm_units: str, sides,
                 hand_topic_template="/cb_{side}_hand_control_cmd",
                 arm_topic_template="/{side}_arm_joint_control",
                 state_topic_template="/cb_{side}_hand_state"):
        if not enable_real:
            raise RuntimeError("RealRos2Backend requires enable_real=True")
        if arm_units not in {"rad", "deg"}:
            raise ValueError("arm_units must be explicitly 'rad' or 'deg'")
        # Only the configured sides get publishers: an idle publisher on another side's
        # topic would still trip the station relay's and our own exclusivity checks.
        self.sides = tuple(sides)
        if not self.sides or set(self.sides) - {"left", "right"}:
            raise ValueError("sides must be a non-empty subset of ('left', 'right')")
        try:
            self._rclpy = importlib.import_module("rclpy")
            self._joint_state_type = importlib.import_module("sensor_msgs.msg").JointState
        except ImportError as exc:
            raise RuntimeError("ROS2 Python packages rclpy and sensor_msgs are required for the real backend") from exc
        self._owns_rclpy = not self._rclpy.ok()
        if self._owns_rclpy:
            self._rclpy.init()
        self.node = node_factory() if node_factory is not None else self._rclpy.create_node("gamepad_vla_workstation")
        self.arm_units = arm_units
        self._hand_topics = {side: hand_topic_template.format(side=side) for side in self.sides}
        self._arm_topics = {side: arm_topic_template.format(side=side) for side in self.sides}
        self._hand_publishers = {
            side: self.node.create_publisher(self._joint_state_type, topic, 10)
            for side, topic in self._hand_topics.items()
        }
        self._arm_publishers = {
            side: self.node.create_publisher(self._joint_state_type, topic, 10)
            for side, topic in self._arm_topics.items()
        }
        self._hand_state = {}  # measured, from the SDK state topic only
        self._arm_state = None  # no measured arm feedback is subscribed yet
        self._stamp = time.monotonic()
        self._subscriptions = [
            self.node.create_subscription(
                self._joint_state_type, state_topic_template.format(side=side),
                lambda msg, side=side: self._state_callback(side, msg), 10)
            for side in self.sides
        ]

    def reset(self):
        pass

    def apply(self, commands):
        for command in commands:
            if command.side not in self.sides:
                raise ValueError(f"real backend is not configured for side {command.side!r}")
            if isinstance(command, HandCommand):
                topic = self._hand_topics[command.side]
                self._ensure_exclusive(topic)
                motor = joint_positions_to_motor_positions(command.side, command.joint_positions)
                positions = [float(min(max(round(value), 0), 255)) for value in motor]
                message = self._joint_state_type()
                message.name = [f"motor_{index}" for index in range(20)]
                message.position = positions
                message.velocity = []
                message.effort = []
                self._hand_publishers[command.side].publish(message)
            elif isinstance(command, ArmCommand):
                topic = self._arm_topics[command.side]
                self._ensure_exclusive(topic)
                positions = list(command.positions)
                if self.arm_units == "deg":
                    positions = [math.degrees(value) for value in positions]
                message = self._joint_state_type()
                message.position = positions
                message.name = []
                message.velocity = []
                message.effort = []
                self._arm_publishers[command.side].publish(message)

    def step(self, dt):
        self._rclpy.spin_once(self.node, timeout_sec=0.0)

    def observe(self):
        return Observation(self._stamp, dict(self._hand_state), self._arm_state,
                           None, None, None, self.name)

    def hold(self):
        pass

    def close(self):
        self.node.destroy_node()
        if self._owns_rclpy:
            self._rclpy.shutdown()

    def _ensure_exclusive(self, topic):
        if self.node.count_publishers(topic) > 1:
            raise RuntimeError(f"refusing to publish to {topic}: another publisher exists")

    def _state_callback(self, side, message):
        from gamepad_vla.workstation.g20_mapping import motor_positions_to_joint_positions
        if len(message.position) >= 20:
            positions = motor_positions_to_joint_positions(side, message.position)
            self._hand_state = dict(zip(G20_JOINT_NAMES, positions))
            self._stamp = time.monotonic()
