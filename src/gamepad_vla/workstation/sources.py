"""Non-blocking UDP, recorded-episode, and scripted command sources."""

import json
import ipaddress
import socket
from pathlib import Path

from gamepad_vla.workstation.commands import (
    ArmCommand,
    HandCommand,
    arm_command_from_packet,
    hand_command_from_packet,
)
from gamepad_vla.workstation.protocol import ArmCommandPacket, HandCommandPacket, decode
from gamepad_vla.workstation.recorder import read_jsonl


def _key(command):
    return ("hand" if isinstance(command, HandCommand) else "arm", command.side)


class UdpSource:
    """Hold the newest valid command per kind and side until it becomes stale."""

    def __init__(self, bind_host, port, *, stale_after_s, allow_source=None, arm_mapper=None):
        if stale_after_s < 0:
            raise ValueError("stale_after_s must be non-negative")
        self.stale_after_s = float(stale_after_s)
        self.arm_mapper = arm_mapper
        self.allow_source = None if allow_source is None else frozenset(
            str(ipaddress.IPv4Address(ip)) for ip in allow_source)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((bind_host, port))
        self.socket.setblocking(False)
        self.bound_address = self.socket.getsockname()
        self.port = self.bound_address[1]
        self.decode_errors = 0
        self.rejected_packets = 0
        self.received_packets = 0
        self._newest = {}
        self.event_sink = None

    def poll(self, now):
        while True:
            try:
                data, address = self.socket.recvfrom(65535)
            except BlockingIOError:
                break
            if self.allow_source is not None and address[0] not in self.allow_source:
                self.rejected_packets += 1
                continue
            self.received_packets += 1
            try:
                packet = decode(data)
                if isinstance(packet, HandCommandPacket):
                    command = hand_command_from_packet(packet)
                elif isinstance(packet, ArmCommandPacket):
                    command = arm_command_from_packet(packet, self.arm_mapper)
                else:
                    continue
            except (TypeError, ValueError) as exc:
                self.decode_errors += 1
                if self.event_sink:
                    self.event_sink(data, None, str(exc))
                continue
            if self.event_sink:
                self.event_sink(data, command, None)
            self._newest[_key(command)] = (float(now), command)
        stale = [key for key, (received, _command) in self._newest.items()
                 if now - received > self.stale_after_s]
        for key in stale:
            del self._newest[key]
        return [item[1] for item in self._newest.values()]

    def close(self):
        self.socket.close()


class ReplaySource:
    """Emit recorded command events at their episode-relative times."""

    def __init__(self, episode_dir):
        self._events = []
        path = Path(episode_dir) / "commands.jsonl"
        for number, row in enumerate(read_jsonl(path), start=1):
            try:
                self._events.append((float(row["t"]), _command_from_row(row)))
            except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
                raise ValueError(f"{path}:{number}: invalid command row") from exc
        self._events.sort(key=lambda item: item[0])
        self._start = None
        self._index = 0

    def poll(self, now):
        if self._start is None:
            self._start = float(now)
        elapsed = float(now) - self._start
        commands = []
        while self._index < len(self._events) and self._events[self._index][0] <= elapsed + 1e-12:
            commands.append(self._events[self._index][1])
            self._index += 1
        return commands

    def close(self):
        pass


class ScriptedSource:
    """Zero-order-hold a relative-time command schedule per kind and side."""

    def __init__(self, schedule):
        self._schedule = sorted(((float(t), command) for t, command in schedule), key=lambda item: item[0])
        if any(t < 0 for t, _command in self._schedule):
            raise ValueError("scripted times must be non-negative")
        if not all(isinstance(command, (HandCommand, ArmCommand)) for _t, command in self._schedule):
            raise TypeError("scripted commands must be HandCommand or ArmCommand values")
        self._start = None
        self._index = 0
        self._held = {}

    def poll(self, now):
        if self._start is None:
            self._start = float(now)
        elapsed = float(now) - self._start
        while self._index < len(self._schedule) and self._schedule[self._index][0] <= elapsed + 1e-12:
            _time, command = self._schedule[self._index]
            self._held[_key(command)] = command
            self._index += 1
        return list(self._held.values())

    def close(self):
        pass


def _command_from_row(row):
    kind = row["kind"]
    side = row["side"]
    stamp = float(row["stamp"])
    if kind == "hand":
        positions = row["joint_positions"]
        if not isinstance(positions, dict):
            raise ValueError("hand joint_positions must be an object")
        return HandCommand(side, {str(name): float(value) for name, value in positions.items()}, stamp)
    if kind == "arm":
        positions = tuple(float(value) for value in row["positions"])
        if len(positions) != 7:
            raise ValueError("arm positions must have length 7")
        return ArmCommand(side, positions, stamp)
    raise ValueError(f"unknown command kind {kind!r}")
