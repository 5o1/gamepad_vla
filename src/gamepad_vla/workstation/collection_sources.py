"""Subscriber-only ROS input and timed dual-arm/dual-hand replay."""
import importlib
import json
from dataclasses import asdict
import queue
import threading
import time
from pathlib import Path

import numpy as np

from gamepad_vla.workstation.collection_health import InputHealth
from gamepad_vla.workstation.commands import ArmCommand, HandCommand, hand_command_from_packet
from gamepad_vla.workstation.protocol import HandCommandPacket
from gamepad_vla.workstation.episode_schema import SIDES, read_episode


def ros_command_topics(prefix=""):
    return {side + "_" + kind: prefix.rstrip("/") + topic
            for side in SIDES
            for kind, topic in (("hand", f"/cb_{side}_hand_control_cmd"),
                                ("arm", f"/{side}_arm_joint_control"))}


class Ros2Source:
    """Continuous ROS reception; lossless event FIFO and separate latest targets.

    A single executor thread serializes callbacks in receive order. Only the
    recorder drains events; control polling never consumes or discards them.
    """
    def __init__(self, mapper, *, node=None, joint_state_type=None, spin_once=None,
                 stale_after_s=.25, clock=time.monotonic, packet_sink=None,
                 topic_prefix="", qos_depth=128, reliability=None):
        if qos_depth < 100:
            raise ValueError("ROS command QoS must buffer at least 100 messages per channel")
        self.health = InputHealth(clock)
        self.mapper, self.clock = mapper, clock
        self.packet_sink, self.stale_after_s = packet_sink, stale_after_s
        self.decode_errors = self.received_packets = 0
        self.latest, self.polled_sequences = {}, {}
        self._lock, self._events = threading.Lock(), queue.Queue()
        self._rclpy = self._executor = self._thread = None
        self._owns_context, self._closing = False, False
        self._thread_error = None
        self.spin_once = spin_once or (lambda: None)  # injected unit-test node only
        if node is None:
            try:
                self._rclpy = importlib.import_module("rclpy")
                joint_state_type = importlib.import_module("sensor_msgs.msg").JointState
            except ImportError as exc:
                raise RuntimeError("ROS2 input needs ROS Humble rclpy and sensor_msgs; source the ROS environment") from exc
            self._owns_context = not self._rclpy.ok()
            if self._owns_context:
                self._rclpy.init()
            node = self._rclpy.create_node("gamepad_collection_subscriber", enable_rosout=False,
                                           start_parameter_services=False)
        self.node = node
        self.subscriptions = []
        # Station publishers: retarget hand commands are RELIABLE, LinkerTA arm
        # commands are BEST_EFFORT. A RELIABLE subscription never matches a
        # BEST_EFFORT publisher, so each channel follows its publisher.
        self.reliability = {"hand": "reliable", "arm": "best_effort", **(reliability or {})}
        for key, topic in ros_command_topics(topic_prefix).items():
            side, kind = key.split("_")
            qos = qos_depth
            if self._rclpy is not None:
                qos_module = importlib.import_module("rclpy.qos")
                qos = qos_module.QoSProfile(depth=qos_depth, reliability=getattr(
                    qos_module.ReliabilityPolicy, self.reliability[kind].upper()))
            self.subscriptions.append(node.create_subscription(
                joint_state_type, topic,
                lambda msg, s=side, k=kind: self._receive(s, k, msg), qos))
        if self._rclpy is not None:
            executor_type = importlib.import_module("rclpy.executors").SingleThreadedExecutor
            self._executor = executor_type(context=node.context)
            self._executor.add_node(node)
            self._thread = threading.Thread(target=self._spin, name="collection-ros-receiver")
            self._thread.start()

    def _spin(self):
        try:
            self._executor.spin()
        except Exception as exc:
            if not self._closing:
                self._thread_error = exc

    def _receive(self, side, kind, msg):
        with self._lock:
            self._receive_locked(side, kind, msg)

    def _receive_locked(self, side, kind, msg):
        from gamepad_vla.workstation.protocol import ArmCommandPacket, encode, decode
        received_mono, received_wall = time.monotonic_ns(), time.time_ns()
        received_clock = self.clock()
        header = getattr(msg, "header", None)
        sec = int(header.stamp.sec) if header else received_wall // 10**9
        nanosec = int(header.stamp.nanosec) if header else received_wall % 10**9
        stamp = sec + nanosec * 1e-9
        raw = dict(name=list(msg.name), position=list(msg.position),
                   velocity=list(getattr(msg, "velocity", [])), effort=list(getattr(msg, "effort", [])))
        event = dict(sequence=self.received_packets, channel=side + "_" + kind,
                     received_monotonic_ns=received_mono, received_wall_time_ns=received_wall,
                     header=dict(stamp=dict(sec=sec, nanosec=nanosec),
                                 frame_id=getattr(header, "frame_id", "")),
                     raw=raw, converted_command=None, error=None)
        self.received_packets += 1
        packet = command = None
        try:
            if kind == "hand":
                if len(raw["position"]) != 20:
                    raise ValueError("expected 20 hand motors")
                # Convert a separate copy. Preserve original binary64 values in raw.
                packet = decode(encode(HandCommandPacket(side, "motor", raw["name"],
                                      raw["position"], raw["velocity"], raw["effort"], stamp)))
                command = hand_command_from_packet(packet)
            else:
                command = self.mapper.command(side, raw["position"], stamp)
                packet = ArmCommandPacket(side, raw["position"], stamp, "master-deg")
            event["converted_command"] = asdict(command)
        except (KeyError, TypeError, ValueError) as exc:
            self.decode_errors += 1
            event["error"] = str(exc)
        self.health.receive(event["channel"], error=event["error"] is not None)
        self._events.put_nowait(event)
        if command is not None:
            self.latest[(kind, side)] = (received_clock, command, event["sequence"])
            if self.packet_sink:
                self.packet_sink(packet)

    def boundary(self):
        with self._lock:
            return self.drain_events(), self.received_packets

    def drain_events(self):
        events = []
        while True:
            try:
                events.append(self._events.get_nowait())
            except queue.Empty:
                return events

    def poll(self, now):
        if self._thread_error:
            raise RuntimeError("ROS receiver failed") from self._thread_error
        if self._rclpy is None:
            self.spin_once()
        with self._lock:
            fresh = [(k, v) for k, v in self.latest.items() if now - v[0] <= self.stale_after_s]
            self.polled_sequences = {side + "_" + kind: v[2] for (kind, side), v in fresh}
            return [v[1] for k, v in fresh]

    def close(self):
        self._closing = True
        if self._executor:
            self._executor.shutdown(timeout_sec=5.)
            self._thread.join(timeout=5.)
            if self._thread.is_alive():
                raise RuntimeError("ROS receiver did not stop")
        self.node.destroy_node()
        if self._owns_context:
            self._rclpy.shutdown()


class CollectionReplay:
    """Replay final slave targets directly. Optional loops preserve original rate."""
    def __init__(self, path, *, loop=False):
        path = Path(path)
        self.loop = loop
        self.decode_errors = 0
        self.cycles = 0
        self.start = None
        self.last_index = None
        self.events = []
        self._input_events = []
        self.received_packets = 0
        self.health = InputHealth()
        self.uniform_grid = path.is_dir()
        if path.is_dir():
            meta, rows = read_episode(path)
            fps = float(meta["fps"])
            if not np.isfinite(fps) or fps <= 0:
                raise ValueError("episode control fps must be positive")
            # Commands belong to the control grid; observations are post-step
            # samples on the engine's quantized clock, not action event times.
            self.times = np.arange(len(rows), dtype=float) / fps
            for row in rows:
                commands = []
                for side in SIDES:
                    h = row["commands"][side + "_hand"]
                    a = row["commands"][side + "_arm"]
                    if h is not None:
                        commands.append(HandCommand(side, dict(h["joint_positions"]), h["stamp"],
                                                    None if h["motor_positions"] is None else tuple(h["motor_positions"])))
                    if a is not None:
                        commands.append(ArmCommand(side, tuple(a["positions"]), a["stamp"],
                                                   None if a["raw_master_deg"] is None else tuple(a["raw_master_deg"]),
                                                   a.get("clamp_flags")))
                self.events.append(commands)
        else:
            with np.load(path, allow_pickle=False) as z:
                action = z["action"]
                self.times = np.asarray(z["timestamp"], float)
                names, units = z["names"].tolist(), z["units"].tolist()
                if len(set(z["episode_index"].tolist())) != 1:
                    raise ValueError("replay NPZ must contain exactly one source episode")
                for row, stamp in zip(action, self.times):
                    commands = []
                    for side in SIDES:
                        ai = [names.index(f"{side}_arm_joint_{j}") for j in range(1, 8)]
                        hi = [names.index(f"{side}_hand_motor_{j}") for j in range(20)]
                        if any(units[i] != "rad" for i in ai) or any(units[i] != "motor_0_255" for i in hi):
                            raise ValueError("replay units must be slave rad / motor_0_255")
                        if not np.isfinite(row).all() or np.any(row[hi] < 0) or np.any(row[hi] > 255):
                            raise ValueError("invalid replay action")
                        commands.append(ArmCommand(side, tuple(row[ai]), float(stamp)))
                        commands.append(hand_command_from_packet(HandCommandPacket(
                            side, "motor", [], row[hi].tolist(), [], [], float(stamp))))
                    self.events.append(commands)
        if len(self.times) < 2 or not np.isfinite(self.times).all() or not np.all(np.diff(self.times) > 0):
            raise ValueError("replay timestamps must increase and contain at least two rows")
        self.times -= self.times[0]
        self.cycle_s = float(self.times[-1] + np.median(np.diff(self.times)))

    def poll(self, now):
        if self.start is None:
            self.start = float(now)
        elapsed = now - self.start
        self.cycles = int(elapsed // self.cycle_s) if self.loop else 0
        phase = elapsed % self.cycle_s if self.loop else elapsed
        query = phase + (1e-12 if self.uniform_grid else 0.)
        index = min(len(self.events) - 1, max(0, np.searchsorted(self.times, query, side="right") - 1))
        marker = (self.cycles, index)
        if marker == self.last_index:
            return []
        self.last_index = marker
        commands = self.events[index]
        for command in commands:
            self._input_events.append(command_event(command, self.received_packets))
            self.received_packets += 1
            self.health.receive(command.side + ("_hand" if isinstance(command, HandCommand) else "_arm"))
        return commands

    def boundary(self):
        return self.drain_events(), self.received_packets

    def drain_events(self):
        events, self._input_events = self._input_events, []
        return events

    def close(self):
        pass


def command_event(command, sequence):
    wall = time.time_ns()
    channel = command.side + ('_hand' if isinstance(command, HandCommand) else '_arm')
    return dict(sequence=sequence, channel=channel, received_monotonic_ns=time.monotonic_ns(),
                received_wall_time_ns=wall, header=dict(stamp=dict(sec=wall//10**9, nanosec=wall%10**9), frame_id=''),
                raw=asdict(command), converted_command=asdict(command), error=None)


class CollectionUdpSource:
    """Existing UDP decoder/control behavior plus receive journal and health."""
    def __init__(self, *args, **kwargs):
        from .sources import UdpSource
        self.source = UdpSource(*args, **kwargs)
        self.source.event_sink = self._receive
        self.health = InputHealth()
        self._events = []
        self.received_packets = 0

    @property
    def decode_errors(self):
        return self.source.decode_errors

    def _receive(self, data, command, error):
        if command is not None:
            event = command_event(command, self.received_packets)
        else:
            wall = time.time_ns()
            event = dict(sequence=self.received_packets, channel='unknown', received_monotonic_ns=time.monotonic_ns(),
                         received_wall_time_ns=wall, header=dict(stamp=dict(sec=wall//10**9, nanosec=wall%10**9)),
                         converted_command=None)
        if command is None:
            try:
                raw = json.loads(data)
                kind = 'arm' if raw.get('type')=='arm_command' else 'hand' if raw.get('type')=='command' else None
                side = raw.get('side') if kind=='arm' else raw.get('hand')
                if kind and side in SIDES:event['channel']=side+'_'+kind
            except (ValueError, AttributeError):pass
        event.update(raw=dict(datagram_hex=data.hex()), error=error)
        self.received_packets += 1
        self._events.append(event)
        self.health.receive(event['channel'], error=error is not None)

    def poll(self, now):
        return self.source.poll(now)

    def drain_events(self):
        events, self._events = self._events, []
        return events

    def boundary(self):
        return self.drain_events(), self.received_packets

    def close(self):
        self.source.close()
