"""Backend-neutral workcell poses and JSON schema.

Poses are child-in-parent ``[x, y, z, qw, qx, qy, qz]`` values in metres.
Quaternion output is normalized and canonicalized to non-negative ``w``.
"""

import json
import re
from dataclasses import asdict, dataclass, field, replace
from pathlib import Path

import numpy as np

from gamepad_vla.kinematics import rpy_to_matrix
from gamepad_vla.workstation.paths import resolve_workspace_path
from gamepad_vla.site import site_value


@dataclass(frozen=True)
class TableSpec:
    pose: tuple[float, ...]
    half_extents: tuple[float, ...]


@dataclass(frozen=True)
class ArmSpec:
    urdf: str
    base_link: str
    tip_link: str
    base_pose: tuple[float, ...]
    home_q: tuple[float, ...]
    joint_names: tuple[str, ...]


@dataclass(frozen=True)
class HandSpec:
    urdf: str
    root_link: str
    mount: tuple[float, ...]
    mount_status: str
    home_q: tuple[float, ...] = (0.0,) * 21


@dataclass(frozen=True)
class IntrinsicsSpec:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    distortion: tuple[float, ...]
    model: str


@dataclass(frozen=True)
class CameraSpec:
    name: str
    serial: str
    model: str
    parent: str
    pose_optical: tuple[float, ...]
    intrinsics: IntrinsicsSpec
    status: str
    source: str


@dataclass(frozen=True)
class FixtureSpec:
    name: str
    type: str
    pose: tuple[float, ...]
    collision: bool
    status: str
    source: str
    half_extents: tuple[float, ...] | None = None
    mesh: str | None = None
    parent: str | None = None


@dataclass(frozen=True)
class GamepadSpec:
    release: str
    profile: str
    collision: str
    root_pose: tuple[float, ...]
    fixed_root: bool


@dataclass(frozen=True)
class WorkcellSpec:
    name: str
    world_frame: str
    table: TableSpec
    left_arm: ArmSpec
    left_hand: HandSpec
    gamepad: GamepadSpec
    notes: tuple[str, ...]
    cameras: tuple[CameraSpec, ...] = ()
    right_arm: ArmSpec | None = None
    right_hand: HandSpec | None = None
    fixtures: tuple[FixtureSpec, ...] = ()
    appearance: str | None = None
    source_dir: Path | None = field(default=None, compare=False, repr=False)


def _finite(values, length, label):
    if not isinstance(values, (list, tuple)) or len(values) != length:
        raise ValueError(f"{label} must have exactly {length} values")
    try:
        result = np.asarray(values, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must contain numbers") from exc
    if not np.isfinite(result).all():
        raise ValueError(f"{label} must contain only finite numbers")
    return result


def _pose(values, label):
    pose = _finite(values, 7, label)
    norm = np.linalg.norm(pose[3:])
    if norm == 0:
        raise ValueError(f"{label} quaternion must be non-zero")
    pose[3:] /= norm
    if pose[3] < 0:
        pose[3:] *= -1
    return tuple(float(value) for value in pose)


def pose_to_matrix(pose):
    """Convert a child-in-parent pose to a homogeneous transform."""
    x, y, z, w, qx, qy, qz = _pose(pose, "pose")
    matrix = np.eye(4)
    matrix[:3, :3] = [
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * w), 2 * (qx * qz + qy * w)],
        [2 * (qx * qy + qz * w), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * w)],
        [2 * (qx * qz - qy * w), 2 * (qy * qz + qx * w), 1 - 2 * (qx * qx + qy * qy)],
    ]
    matrix[:3, 3] = [x, y, z]
    return matrix


def matrix_to_pose(matrix):
    """Convert a rigid homogeneous transform to canonical pose form."""
    matrix = np.asarray(matrix, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError("matrix must be finite with shape (4, 4)")
    rotation = matrix[:3, :3]
    trace = np.trace(rotation)
    if trace > 0:
        scale = 2 * np.sqrt(trace + 1)
        quat = np.array([scale / 4,
                         (rotation[2, 1] - rotation[1, 2]) / scale,
                         (rotation[0, 2] - rotation[2, 0]) / scale,
                         (rotation[1, 0] - rotation[0, 1]) / scale])
    else:
        index = int(np.argmax(np.diag(rotation)))
        if index == 0:
            scale = 2 * np.sqrt(1 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2])
            quat = np.array([(rotation[2, 1] - rotation[1, 2]) / scale, scale / 4,
                             (rotation[0, 1] + rotation[1, 0]) / scale,
                             (rotation[0, 2] + rotation[2, 0]) / scale])
        elif index == 1:
            scale = 2 * np.sqrt(1 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2])
            quat = np.array([(rotation[0, 2] - rotation[2, 0]) / scale,
                             (rotation[0, 1] + rotation[1, 0]) / scale, scale / 4,
                             (rotation[1, 2] + rotation[2, 1]) / scale])
        else:
            scale = 2 * np.sqrt(1 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1])
            quat = np.array([(rotation[1, 0] - rotation[0, 1]) / scale,
                             (rotation[0, 2] + rotation[2, 0]) / scale,
                             (rotation[1, 2] + rotation[2, 1]) / scale, scale / 4])
    return _pose([*matrix[:3, 3], *quat], "matrix pose")


def compose(a, b):
    """Compose child-in-parent poses ``a @ b``."""
    return matrix_to_pose(pose_to_matrix(a) @ pose_to_matrix(b))


def invert(pose):
    """Invert a child-in-parent pose."""
    matrix = pose_to_matrix(pose)
    inverse = np.eye(4)
    inverse[:3, :3] = matrix[:3, :3].T
    inverse[:3, 3] = -inverse[:3, :3] @ matrix[:3, 3]
    return matrix_to_pose(inverse)


def rpy_to_quat(roll, pitch, yaw):
    """Return the quaternion for URDF fixed-axis ``Rz(yaw) Ry(pitch) Rx(roll)``."""
    matrix = np.eye(4)
    matrix[:3, :3] = rpy_to_matrix((roll, pitch, yaw))
    return matrix_to_pose(matrix)[3:]


def _mapping(value, keys, label, optional=()):
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be an object")
    unknown = set(value) - set(keys) - set(optional)
    missing = set(keys) - set(value)
    if unknown:
        raise ValueError(f"{label} has unknown keys: {sorted(unknown)}")
    if missing:
        raise ValueError(f"{label} is missing keys: {sorted(missing)}")
    return value


def _string(value, label):
    if not isinstance(value, str):
        raise ValueError(f"{label} must be a string")
    return value


def _strings(value, length, label):
    if not isinstance(value, (list, tuple)) or (length is not None and len(value) != length):
        expected = "a list" if length is None else f"a list of {length} strings"
        raise ValueError(f"{label} must be {expected}")
    if not all(isinstance(item, str) for item in value):
        raise ValueError(f"{label} must contain only strings")
    return tuple(value)


def optical_to_sapien_pose(pose_optical):
    """T_parent_sapien: SAPIEN +x/+y/+z are optical +z/-x/-y.

    This rotates the local basis, without moving the optical centre.
    """
    optical_from_sapien = np.eye(4)
    optical_from_sapien[:3, :3] = [[0, -1, 0], [0, 0, -1], [1, 0, 0]]
    return matrix_to_pose(pose_to_matrix(pose_optical) @ optical_from_sapien)


def _arm(data, label):
    data = _mapping(data, ("urdf", "base_link", "tip_link", "base_pose", "home_q", "joint_names"), label)
    return ArmSpec(*(_string(data[k], label + "." + k) for k in ("urdf", "base_link", "tip_link")),
                   _pose(data["base_pose"], label + ".base_pose"),
                   tuple(_finite(data["home_q"], 7, label + ".home_q")),
                   _strings(data["joint_names"], 7, label + ".joint_names"))


def _hand(data, label):
    data = _mapping(data, ("urdf", "root_link", "mount", "mount_status"), label, ("home_q",))
    if data["mount_status"] not in {"measured", "unverified"}:
        raise ValueError(label + ".mount_status must be 'measured' or 'unverified'")
    return HandSpec(_string(data["urdf"], label + ".urdf"),
                    _string(data["root_link"], label + ".root_link"),
                    _pose(data["mount"], label + ".mount"), data["mount_status"],
                    tuple(_finite(data.get("home_q", [0.] * 21), 21, label + ".home_q")))


def _status(value, coarse=False):
    allowed={"measured", "design", "assumed", "estimated", "approximate"}
    if coarse:allowed.add("粗配准")
    if value not in allowed:
        raise ValueError("status must be measured, design, assumed, estimated or approximate")
    return value


def _cameras(values):
    if not isinstance(values, (list, tuple)):
        raise ValueError("cameras must be a list")
    result = []
    for data in values:
        data = _mapping(data, ("name", "serial", "model", "parent", "pose_optical", "intrinsics", "status", "source"), "camera")
        intr = _mapping(data["intrinsics"], ("width", "height", "fx", "fy", "cx", "cy", "distortion", "model"), "intrinsics")
        for key in ("width", "height"):
            if type(intr[key]) is not int or intr[key] <= 0:
                raise ValueError("intrinsics dimensions must be positive integers")
        numbers = _finite([intr[k] for k in ("fx", "fy", "cx", "cy")], 4, "intrinsics")
        if min(numbers[:2]) <= 0:
            raise ValueError("intrinsics focal lengths must be positive")
        model = _string(intr["model"], "intrinsics.model")
        distortion = intr["distortion"]
        if model not in {"none", "plumb_bob", "rational_polynomial"}:
            raise ValueError("unsupported intrinsics.model")
        if not isinstance(distortion, (tuple, list)) or len(distortion) not in {0, 4, 5, 8, 12, 14}:
            raise ValueError("unsupported distortion coefficient count")
        coeff = tuple(_finite(distortion, len(distortion), "distortion"))
        result.append(CameraSpec(*(_string(data[k], "camera." + k) for k in ("name", "serial", "model", "parent")),
                                 _pose(data["pose_optical"], "pose_optical"),
                                 IntrinsicsSpec(intr["width"], intr["height"], *map(float, numbers), coeff, model),
                                 _status(data["status"],coarse=True), _string(data["source"], "camera.source")))
    names = [c.name for c in result]
    if len(set(names)) != len(names):
        raise ValueError("camera names must be unique")
    return tuple(result)


def _fixtures(values):
    if not isinstance(values, (list, tuple)):
        raise ValueError("fixtures must be a list")
    result = []
    for data in values:
        data = _mapping(data, ("name", "type", "pose", "collision", "status", "source"), "fixture", ("half_extents", "mesh", "parent"))
        kind = data["type"]
        if kind not in {"box", "mesh"}:
            raise ValueError("fixture.type must be box or mesh")
        if type(data["collision"]) is not bool:
            raise ValueError("fixture.collision must be bool")
        if (kind == "box" and ("half_extents" not in data or "mesh" in data)) or (kind == "mesh" and ("mesh" not in data or "half_extents" in data)):
            raise ValueError("fixture needs exactly its type's geometry")
        half = tuple(_finite(data["half_extents"], 3, "fixture.half_extents")) if kind == "box" else None
        if half is not None and min(half) <= 0:
            raise ValueError("fixture.half_extents must be positive")
        result.append(FixtureSpec(_string(data["name"], "fixture.name"), kind,
                                  _pose(data["pose"], "fixture.pose"), data["collision"],
                                  _status(data["status"]), _string(data["source"], "fixture.source"),
                                  half, _string(data["mesh"], "fixture.mesh") if kind == "mesh" else None,
                                  _string(data["parent"],"fixture.parent") if "parent" in data else None))
    if len({f.name for f in result}) != len(result):
        raise ValueError("fixture names must be unique")
    return tuple(result)


def spec_from_dict(data):
    """Validate and construct a workcell spec from JSON-compatible data."""
    root_keys = ("name", "world_frame", "table", "left_arm", "left_hand", "gamepad", "notes")
    data = _mapping(data, root_keys, "spec", ("cameras", "right_arm", "right_hand", "fixtures", "appearance"))
    if ("right_arm" in data) != ("right_hand" in data):
        raise ValueError("right_arm and right_hand must be supplied together")
    for camera in data.get("cameras", []):
        parent = camera.get("parent")
        if parent != data["world_frame"] and (not isinstance(parent,str) or not re.fullmatch(r"(left|right)_(arm|hand):[^:]+",parent)):
            raise ValueError("camera.parent must be world_frame or <side>_<arm|hand>:<link>")
        if isinstance(parent,str) and parent.startswith("right_") and "right_arm" not in data:
            raise ValueError("right camera parent requires right robot")
    table = _mapping(data["table"], ("pose", "half_extents"), "table")
    arm = _mapping(data["left_arm"],
                   ("urdf", "base_link", "tip_link", "base_pose", "home_q", "joint_names"),
                   "left_arm")
    hand = _mapping(data["left_hand"], ("urdf", "root_link", "mount", "mount_status"), "left_hand", ("home_q",))
    gamepad = _mapping(data["gamepad"],
                      ("release", "profile", "collision", "root_pose", "fixed_root"), "gamepad")
    mount_status = _string(hand["mount_status"], "left_hand.mount_status")
    if mount_status not in {"measured", "unverified"}:
        raise ValueError("left_hand.mount_status must be 'measured' or 'unverified'")
    collision = _string(gamepad["collision"], "gamepad.collision")
    if collision not in {"sdf", "coacd"}:
        raise ValueError("gamepad.collision must be 'sdf' or 'coacd'")
    if not isinstance(gamepad["fixed_root"], bool):
        raise ValueError("gamepad.fixed_root must be a bool")
    half_extents = _finite(table["half_extents"], 3, "table.half_extents")
    if np.any(half_extents <= 0):
        raise ValueError("table.half_extents must be positive")
    return WorkcellSpec(
        name=_string(data["name"], "name"),
        world_frame=_string(data["world_frame"], "world_frame"),
        table=TableSpec(_pose(table["pose"], "table.pose"), tuple(float(value) for value in half_extents)),
        left_arm=ArmSpec(
            _string(arm["urdf"], "left_arm.urdf"),
            _string(arm["base_link"], "left_arm.base_link"),
            _string(arm["tip_link"], "left_arm.tip_link"),
            _pose(arm["base_pose"], "left_arm.base_pose"),
            tuple(float(value) for value in _finite(arm["home_q"], 7, "left_arm.home_q")),
            _strings(arm["joint_names"], 7, "left_arm.joint_names"),
        ),
        left_hand=HandSpec(
            _string(hand["urdf"], "left_hand.urdf"),
            _string(hand["root_link"], "left_hand.root_link"),
            _pose(hand["mount"], "left_hand.mount"),
            mount_status,
            tuple(_finite(hand.get("home_q", [0.] * 21), 21, "left_hand.home_q")),
        ),
        gamepad=GamepadSpec(
            _string(gamepad["release"], "gamepad.release"),
            _string(gamepad["profile"], "gamepad.profile"), collision,
            _pose(gamepad["root_pose"], "gamepad.root_pose"), gamepad["fixed_root"],
        ),
        notes=_strings(data["notes"], None, "notes"),
        cameras=_cameras(data.get("cameras", [])),
        right_arm=_arm(data["right_arm"], "right_arm") if "right_arm" in data else None,
        right_hand=_hand(data["right_hand"], "right_hand") if "right_hand" in data else None,
        fixtures=_fixtures(data.get("fixtures", [])),
        appearance=_string(data["appearance"], "appearance") if "appearance" in data else None,
    )


def spec_to_dict(spec):
    """Return a JSON-compatible representation of ``spec``."""
    data = asdict(spec)
    data.pop("source_dir")
    for key in ("cameras", "right_arm", "right_hand", "fixtures", "appearance"):
        if not data[key]:
            data.pop(key)
    for side in ("left", "right"):
        hand = data.get(f"{side}_hand")
        if hand and not any(hand["home_q"]):
            hand.pop("home_q")
    if "fixtures" in data:
        data["fixtures"] = [{k: v for k, v in f.items() if v is not None} for f in data["fixtures"]]
    return data


def load_spec(path):
    """Load and strictly validate a JSON workcell spec."""
    try:
        data = json.loads(Path(path).read_text())
    except json.JSONDecodeError as exc:
        raise ValueError(f"{path}: invalid JSON: {exc.msg}") from exc
    for key in ("left_arm", "right_arm", "left_hand", "right_hand"):
        if key in data:
            data[key]["urdf"] = resolve_workspace_path(data[key]["urdf"])
    for fixture in data.get("fixtures", []):
        if "mesh" in fixture:
            fixture["mesh"] = resolve_workspace_path(fixture["mesh"])
    if data.get("appearance"):
        data["appearance"] = resolve_workspace_path(data["appearance"])
    for camera in data.get("cameras", []):
        camera["serial"] = site_value("camera.serials." + camera["name"]) or camera["serial"]
    spec = spec_from_dict(data)
    if (spec.right_arm is None) != (spec.right_hand is None):
        raise ValueError("right_arm and right_hand must be supplied together")
    return replace(spec, source_dir=Path(path).resolve().parent)


def dump_spec(spec, path):
    """Write a workcell spec as deterministic, readable JSON."""
    validated = spec_from_dict(spec_to_dict(spec))
    Path(path).write_text(json.dumps(spec_to_dict(validated), indent=2) + "\n")


def hand_root_in_world(spec, arm_tip_in_base):
    """Compose world-to-arm-base, base-to-tip, and tip-to-hand-root poses."""
    return compose(compose(spec.left_arm.base_pose, arm_tip_in_base), spec.left_hand.mount)
