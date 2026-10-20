import json
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from gamepad_vla.workstation import spec


CONFIG_DIR = Path(__file__).parents[1] / "configs/workcell"
CONFIG = CONFIG_DIR / "station_left_g20_v2.json"


@pytest.mark.parametrize("name", ["station_left_g20_v1.json", "station_left_g20_v2.json",
                                  "station_left_g20_v3.json"])
def test_station_spec_and_pose_composition(tmp_path, name):
    workcell = spec.load_spec(CONFIG_DIR / name)
    assert workcell.world_frame == "world"
    assert workcell.left_hand.mount_status == "unverified"
    assert workcell.left_arm.joint_names[-1].endswith("joint_7")
    np.testing.assert_allclose(np.linalg.det(spec.pose_to_matrix(workcell.left_arm.base_pose)[:3, :3]), 1.0)
    identity = spec.compose(workcell.left_arm.base_pose, spec.invert(workcell.left_arm.base_pose))
    np.testing.assert_allclose(identity, [0, 0, 0, 1, 0, 0, 0], atol=1e-12)
    tip = [0.1, -0.2, 0.3, 1, 0, 0, 0]
    expected = spec.compose(spec.compose(workcell.left_arm.base_pose, tip), workcell.left_hand.mount)
    np.testing.assert_allclose(spec.hand_root_in_world(workcell, tip), expected)
    output = tmp_path / "spec.json"
    spec.dump_spec(workcell, output)
    assert spec.load_spec(output) == workcell


def test_v2_table_is_urdf_top_plate_and_gamepad_rests_above_it():
    workcell = spec.load_spec(CONFIG)
    centre, half = np.array(workcell.table.pose[:3]), np.array(workcell.table.half_extents)
    np.testing.assert_allclose(centre - half, [0.093415, 0.0002, 0.752155], atol=1e-5)
    np.testing.assert_allclose(centre + half, [0.718415, 1.2502, 0.767155], atol=1e-5)
    root = np.array(workcell.gamepad.root_pose[:3])
    gap = root[2] - 0.0299 - (centre[2] + half[2])  # collision bottom of controller_body, v0.3.1
    assert 0 < gap < 0.002
    assert np.all(np.abs(root[:2] - centre[:2]) + [0.0766, 0.0555] < half[:2])


def test_v3_uses_station_measurements():
    workcell = spec.load_spec(CONFIG_DIR / "station_left_g20_v3.json")
    centre, half = np.array(workcell.table.pose[:3]), np.array(workcell.table.half_extents)
    top, back, left = centre[2] + half[2], centre[0] - half[0], centre[1] - half[1]
    np.testing.assert_allclose([top, 2 * half[2], 2 * half[0], 2 * half[1]], [0.767, 0.010, 0.625, 1.250])  # A B3 B2 B1
    base = np.array(workcell.left_arm.base_pose[:3])
    np.testing.assert_allclose(base[2] - top, 0.495, atol=1e-9)  # D1
    np.testing.assert_allclose(base[1] - left, 0.560 + 0.060 + 0.091, atol=1e-9)  # C1 + C2/2 + 182/2
    np.testing.assert_allclose(back - base[0], 0.113 / 2, atol=1e-6)  # flush column (C3), URDF depth
    np.testing.assert_allclose(workcell.left_hand.mount[:3], [0, 0, 0.020])  # E1
    root = np.array(workcell.gamepad.root_pose[:3])
    np.testing.assert_allclose([root[0] - back, root[1] - centre[1]], [0.380, 0.0], atol=1e-6)
    np.testing.assert_allclose(workcell.gamepad.root_pose[3:], [np.sqrt(0.5), 0, 0, -np.sqrt(0.5)], atol=1e-6)
    assert 0 < root[2] - 0.0299 - top < 0.002


def test_quaternion_normalization_and_rpy_convention():
    pose = spec.matrix_to_pose(spec.pose_to_matrix([1, 2, 3, -2, 0, 0, 0]))
    assert pose == (1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0)
    quat = spec.rpy_to_quat(0.2, -0.4, 0.9)
    matrix = spec.pose_to_matrix([0, 0, 0, *quat])
    np.testing.assert_allclose(matrix[:3, :3], spec.rpy_to_matrix((0.2, -0.4, 0.9)), atol=1e-12)


@pytest.mark.parametrize("mutate,message", [
    (lambda data: data.update(extra=1), "unknown keys"),
    (lambda data: data["left_arm"].update(home_q=[0] * 6), "exactly 7"),
    (lambda data: data["table"].update(pose=[0, 0, 0, 1, 0, 0, float("nan")]), "finite"),
    (lambda data: data["left_hand"].update(mount_status="guessed"), "mount_status"),
    (lambda data: data["gamepad"].update(collision="mesh"), "collision"),
])
def test_strict_validation(tmp_path, mutate, message):
    data = json.loads(CONFIG.read_text())
    mutate(data)
    path = tmp_path / "bad.json"
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match=message):
        spec.load_spec(path)
