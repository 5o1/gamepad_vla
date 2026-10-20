import numpy as np
import pytest

from gamepad_vla import kinematics

PLANAR = """<robot name="planar">
  <link name="base"/><link name="l1"/><link name="l2"/><link name="tip"/>
  <joint name="j1" type="revolute"><parent link="base"/><child link="l1"/>
    <axis xyz="0 0 1"/><limit lower="-3" upper="3"/></joint>
  <joint name="j2" type="revolute"><parent link="l1"/><child link="l2"/>
    <origin xyz="0.1 0 0"/><axis xyz="0 0 1"/><limit lower="-3" upper="3"/>
    <mimic joint="j1" multiplier="2" offset="0.1"/></joint>
  <joint name="j3" type="fixed"><parent link="l2"/><child link="tip"/>
    <origin xyz="0.05 0 0" rpy="0 0 1.5707963267948966"/></joint>
</robot>"""


@pytest.fixture
def chain(tmp_path):
    path = tmp_path / "planar.urdf"
    path.write_text(PLANAR)
    return kinematics.SerialChain(path, "tip")


def test_planar_chain_with_mimic(chain):
    assert chain.root_link == "base" and chain.active == ["j1"]
    q1 = np.array([[0.0], [0.3], [-1.0]])
    pose = chain.forward(q1)
    q2 = 2 * q1[:, 0] + 0.1
    expected = np.stack([0.1 * np.cos(q1[:, 0]) + 0.05 * np.cos(q1[:, 0] + q2),
                         0.1 * np.sin(q1[:, 0]) + 0.05 * np.sin(q1[:, 0] + q2)], axis=1)
    np.testing.assert_allclose(pose[:, :2, 3], expected, atol=1e-12)
    yaw = np.arctan2(pose[:, 1, 0], pose[:, 0, 0])
    np.testing.assert_allclose(np.cos(yaw), np.cos(q1[:, 0] + q2 + np.pi / 2), atol=1e-12)
    np.testing.assert_allclose(chain.active_limits[0], [-3.0])


def test_link_poses_include_intermediate_links(chain):
    poses = chain.link_poses(np.array([[0.5]]))
    assert set(poses) == {"l1", "l2", "tip"}
    np.testing.assert_allclose(poses["l2"][0, :2, 3], [0.1 * np.cos(0.5), 0.1 * np.sin(0.5)], atol=1e-12)
    np.testing.assert_array_equal(poses["tip"], chain.forward(np.array([[0.5]])))


def test_rpy_convention():
    # URDF rpy is fixed-axis XYZ: roll about x first, then pitch about y, then yaw about z.
    r, p, y = 0.2, -0.4, 0.9
    expected = (kinematics.axis_angle_to_matrix([0, 0, 1], np.array([y]))[0]
                @ kinematics.axis_angle_to_matrix([0, 1, 0], np.array([p]))[0]
                @ kinematics.axis_angle_to_matrix([1, 0, 0], np.array([r]))[0])
    np.testing.assert_allclose(kinematics.rpy_to_matrix([r, p, y]), expected, atol=1e-12)


def test_wrong_width_rejected(chain):
    with pytest.raises(ValueError):
        chain.forward(np.zeros((1, 2)))
