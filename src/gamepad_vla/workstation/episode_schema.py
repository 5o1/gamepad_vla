"""Collection v3: SI units, world poses in xyz/wxyz, left then right.

Unavailable original commands use null, never reconstructed measurements.
Fingertips use distal-frame orientation and the collision mesh's +Z extremum
at local x=y=0; the resulting offsets are recorded in episode metadata.
"""
from dataclasses import dataclass, asdict
import copy
import hashlib
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np

SIDES = ("left", "right")
FINGERS = ("thumb", "index", "middle", "ring", "pinky")
SCHEMA_VERSION = 3


@dataclass
class Observation:
    sim_time_s: float
    wall_time_ns: int
    hand_q: np.ndarray
    arm_q: np.ndarray
    palm_pose_world: np.ndarray
    fingertip_pose_world: np.ndarray
    gamepad_signal: np.ndarray
    gamepad_mask: np.ndarray
    control_joint_positions: dict
    robot_contact: bool
    backend: str

    def validate(self):
        for name, shape in (("hand_q", (2, 21)), ("arm_q", (2, 7)),
                            ("palm_pose_world", (2, 7)), ("fingertip_pose_world", (2, 5, 7)),
                            ("gamepad_signal", (32,)), ("gamepad_mask", (32,))):
            a = np.asarray(getattr(self, name))
            if a.shape != shape or (name != "gamepad_mask" and not np.isfinite(a).all()):
                raise ValueError(f"invalid {name}: expected finite {shape}")
        for a in (self.palm_pose_world, self.fingertip_pose_world):
            if not np.allclose(np.linalg.norm(np.asarray(a)[..., 3:], axis=-1), 1, atol=1e-5):
                raise ValueError("world pose quaternion must have unit norm")
        if not np.isfinite(self.sim_time_s) or self.sim_time_s < 0 or self.wall_time_ns <= 0:
            raise ValueError("invalid observation clock")
        if not all(np.isfinite(v) for v in self.control_joint_positions.values()):
            raise ValueError("nonfinite control displacement")

    def to_dict(self):
        self.validate()
        return {k: v.tolist() if isinstance(v, np.ndarray) else v for k, v in asdict(self).items()}


def fingertip_offsets(spec):
    import trimesh
    from gamepad_vla.kinematics import rpy_to_matrix
    result = {}
    for side in SIDES:
        urdf = Path(getattr(spec, side + "_hand").urdf)
        root = ET.parse(urdf).getroot()
        result[side] = {}
        for finger in FINGERS:
            col = root.find(f"link[@name='{finger}_distal']/collision")
            mesh = col.find("geometry/mesh")
            vertices = np.asarray(trimesh.load(urdf.parent / mesh.get("filename"), force="mesh").vertices).copy()
            vertices *= np.fromstring(mesh.get("scale", "1 1 1"), sep=" ")
            origin = col.find("origin")
            if origin is not None:
                vertices = vertices @ rpy_to_matrix(np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")).T
                vertices += np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
            result[side][finger] = [0., 0., float(vertices[:, 2].max())]
    return result


def tip_pose(link_pose, offset):
    from gamepad_vla.workstation.spec import pose_to_matrix
    pose = np.asarray(link_pose, float).copy()
    pose[:3] += pose_to_matrix(pose.tolist())[:3, :3] @ offset
    if pose[3] < 0:
        pose[3:] *= -1
    return pose


class EpisodeWriter:
    """One start/stop session, synchronous append-only portable JSON journal."""
    def __init__(self, path, meta):
        self.path = Path(path)
        self.path.mkdir(parents=True, exist_ok=False)
        self.meta = dict(copy.deepcopy(meta), schema_version=SCHEMA_VERSION, status="recording", rows=0, input_messages=0)
        scene = self.meta["scene_spec_text"]
        self.meta["scene_spec_sha256"] = hashlib.sha256(scene.encode()).hexdigest()
        self.meta.update(side_order=list(SIDES), finger_order=list(FINGERS),
                         pose_convention="world_from_frame:[x,y,z,qw,qx,qy,qz];m;rad",
                         created_wall_time_ns=time.time_ns())
        self._write_meta()
        self.stream = (self.path / "steps.jsonl").open("x", buffering=1)
        self.input_stream = (self.path / "commands.jsonl").open("x", buffering=1)
        self._last_input_sequence = -1
        self._last_receive_ns = -1
        self.closed = False
        self.last_sim_time = -1.

    def _write_meta(self):
        (self.path / "meta.json").write_text(json.dumps(self.meta, ensure_ascii=False, indent=2, allow_nan=False) + "\n")

    def append(self, observation, command_state, overlay, *, applied_monotonic_ns=None, input_sequences=None):
        row = observation.to_dict()
        if observation.sim_time_s <= self.last_sim_time:
            raise ValueError("simulation timestamps must increase")
        self.last_sim_time = observation.sim_time_s
        row["commands"] = command_state
        row["overlay"] = overlay
        row["applied_monotonic_ns"] = applied_monotonic_ns
        row["input_sequences"] = dict(input_sequences or {})
        self.stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False, separators=(",", ":")) + "\n")
        self.meta["rows"] += 1

    def append_inputs(self, events):
        for event in events:
            if event["sequence"] <= self._last_input_sequence or event["received_monotonic_ns"] < self._last_receive_ns:
                raise ValueError("input events must preserve receive order")
            self.input_stream.write(json.dumps(event, ensure_ascii=False, allow_nan=False) + "\n")
            self._last_input_sequence = event["sequence"]
            self._last_receive_ns = event["received_monotonic_ns"]
            self.meta["input_messages"] += 1

    def note(self, text):
        self.meta.setdefault("operator_notes", []).append({"wall_time_ns": time.time_ns(), "text": str(text)})
        self._write_meta()

    def close(self, status="complete"):
        if not self.closed:
            self.stream.close()
            self.input_stream.close()
            self.meta["status"] = status
            self._write_meta()
            self.closed = True


def read_episode(path):
    path = Path(path)
    meta = json.loads((path / "meta.json").read_text())
    if meta.get("schema_version") not in (2, SCHEMA_VERSION):
        raise ValueError("expected collection schema v2 or v3")
    if hashlib.sha256(meta["scene_spec_text"].encode()).hexdigest() != meta["scene_spec_sha256"]:
        raise ValueError("scene specification hash mismatch")
    rows = [json.loads(s) for s in (path / "steps.jsonl").read_text().splitlines()]
    last = -1.
    for row in rows:
        fields = {k: row[k] for k in Observation.__dataclass_fields__}
        Observation(**fields).validate()
        if row["sim_time_s"] <= last:
            raise ValueError("simulation timestamps must increase")
        last = row["sim_time_s"]
    if meta["rows"] != len(rows):
        raise ValueError("episode length differs from metadata; recording may be interrupted")
    if meta["schema_version"] == 3:
        read_input_events(path, meta=meta)
    return meta, rows


def read_input_events(path, *, meta=None):
    """Read every received command; v2 has no event journal (return empty)."""
    path = Path(path)
    meta = meta or json.loads((path / "meta.json").read_text())
    if meta["schema_version"] == 2:
        return []
    if meta["schema_version"] != SCHEMA_VERSION:
        raise ValueError("expected collection schema v2 or v3")
    events = [json.loads(line) for line in (path / "commands.jsonl").read_text().splitlines()]
    if len(events) != meta["input_messages"]:
        raise ValueError("input journal length differs from metadata")
    previous_sequence = previous_receive = -1
    for event in events:
        if event["sequence"] <= previous_sequence or event["received_monotonic_ns"] < previous_receive:
            raise ValueError("input journal receive order is invalid")
        if event["received_wall_time_ns"] <= 0 or not isinstance(event["header"]["stamp"]["nanosec"], int):
            raise ValueError("invalid input clocks")
        previous_sequence, previous_receive = event["sequence"], event["received_monotonic_ns"]
    return events
