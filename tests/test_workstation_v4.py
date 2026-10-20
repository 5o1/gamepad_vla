import copy
import json
from pathlib import Path

import numpy as np
import pytest
import sys
from types import SimpleNamespace

from gamepad_vla.workstation.spec import load_spec, spec_from_dict, spec_to_dict, pose_to_matrix, optical_to_sapien_pose, compose
from gamepad_vla.workstation.appearance import load_appearance, match_material
from gamepad_vla.workstation.render_review import create_spec_camera, mask_iou, camera_world_optical, robot_only_review_visuals
from gamepad_vla.workstation.sapien_backend import SapienBackend
from gamepad_vla.workstation.commands import ArmCommand

ROOT = Path(__file__).parents[1]
SPEC = ROOT / "configs/workcell/station_g20_v4.json"


def camera():
    return dict(name="camera_1",serial="test-camera",model="D435",parent="world",
                pose_optical=[1,2,3,1,0,0,0],
                intrinsics=dict(width=640,height=480,fx=500.,fy=501.,cx=321.,cy=239.,distortion=[0]*5,model="plumb_bob"),
                status="estimated",source="unit test fixture")


def test_v4_roundtrip_and_preserves_v3_numbers(tmp_path):
    v4=load_spec(SPEC)
    v3=load_spec(ROOT/"configs/workcell/station_left_g20_v3.json")
    assert v4.left_arm == v3.left_arm
    assert v4.left_hand == v3.left_hand
    assert v4.table == v3.table and v4.gamepad == v3.gamepad
    assert v4.right_arm and len(v4.right_hand.home_q)==21
    assert all(f.collision for f in v4.fixtures if not f.name.startswith("camera_"))
    data=spec_to_dict(v4)
    data["cameras"]=[camera()]
    parsed=spec_from_dict(data)
    assert spec_from_dict(spec_to_dict(parsed))==parsed


@pytest.mark.parametrize("mutate", [
    lambda d:d["cameras"][0]["intrinsics"].update(fx=0),
    lambda d:d["cameras"][0]["intrinsics"].update(width=640.5),
    lambda d:d["cameras"][0].update(status="calibrated"),
    lambda d:d["cameras"].append(copy.deepcopy(d["cameras"][0])),
    lambda d:d["right_hand"].update(home_q=[0]*20),
    lambda d:d.pop("right_hand"),
    lambda d:d["fixtures"][0].update(collision="false"),
    lambda d:d["fixtures"][0].update(half_extents=[1,-1,1]),
    lambda d:d["fixtures"][0].update(unexpected=1),
])
def test_v4_rejects_invalid_fields(mutate):
    data=json.loads(SPEC.read_text()); data["cameras"]=[camera()]
    mutate(data)
    with pytest.raises(ValueError): spec_from_dict(data)


def test_optical_basis_and_offcentre_projection():
    pose=[1,2,3,1,0,0,0]
    transformed=pose_to_matrix(optical_to_sapien_pose(pose))
    np.testing.assert_allclose(transformed[:3,3],pose[:3])
    np.testing.assert_allclose(transformed[:3,:3]@[1,0,0],[0,0,1],atol=1e-14)
    np.testing.assert_allclose(transformed[:3,:3]@[0,1,0],[-1,0,0],atol=1e-14)
    np.testing.assert_allclose(transformed[:3,:3]@[0,0,1],[0,-1,0],atol=1e-14)
    rotated=[.2,-.1,.8,.5,.5,.5,.5]
    expected=pose_to_matrix(rotated) @ np.linalg.inv(pose_to_matrix(pose)) @ transformed
    np.testing.assert_allclose(pose_to_matrix(optical_to_sapien_pose(rotated)),expected,atol=1e-14)


def test_appearance_parsing_first_match_and_rejection(tmp_path):
    table=load_appearance(ROOT/"configs/appearance/station_v1.json")
    assert match_material(table,"table")["name"]=="table_green"
    assert match_material(table,"AR5_5_07R_W4C4A2_base")["name"]=="ar5_base_gray"
    assert match_material(table,"AR5_5_07L_W4C4A2_link1")["name"]=="ar5_warm_white"
    assert match_material(table,"right_index_distal")["name"]=="g20_dark"
    assert match_material(table,"unmatched") is None
    table["materials"][0]["base_color"]=[1.1,0,0,1]
    path=tmp_path/"bad.json";path.write_text(json.dumps(table))
    with pytest.raises(ValueError,match="base_color"):load_appearance(path)


def test_calibrated_camera_passes_full_intrinsics_and_optical_pose(monkeypatch):
    data=spec_to_dict(load_spec(SPEC));data["cameras"]=[camera()]
    definition=spec_from_dict(data).cameras[0]
    calls={}
    rendered=SimpleNamespace(
        set_perspective_parameters=lambda *args:calls.update(intrinsics=args),
        entity=SimpleNamespace(set_pose=lambda pose:calls.update(pose=pose)))
    backend=SimpleNamespace(spec=SimpleNamespace(world_frame="world"),
                            scene=SimpleNamespace(add_camera=lambda *args:rendered))
    monkeypatch.setitem(sys.modules,"sapien",SimpleNamespace(Pose=lambda p,q:[*p,*q]))
    assert create_spec_camera(backend,definition) is rendered
    assert calls["intrinsics"]==(.001,20.,500.,501.,321.,239.,0.)
    np.testing.assert_allclose(pose_to_matrix(calls["pose"]),pose_to_matrix(optical_to_sapien_pose([1,2,3,1,0,0,0])))
    assert mask_iou(np.array([True,True,False]),np.array([False,True,True])) == 1/3
    assert mask_iou(np.zeros(3,bool),np.zeros(3,bool)) is None


@pytest.mark.parametrize("parent,resolved",[("left_hand:hand_base_link","hand_base_link"),
                                           ("right_hand:hand_base_link","right_hand_base_link"),
                                           ("left_arm:AR5_link","AR5_link"),
                                           ("right_arm:AR5_right_link","AR5_right_link")])
def test_eye_in_hand_parent_follows_link(parent,resolved):
    from dataclasses import replace
    data=spec_to_dict(load_spec(SPEC));data["cameras"]=[camera()]
    definition=replace(spec_from_dict(data).cameras[0],parent=parent)
    root_pose=[.2,.3,.8,.5,.5,.5,.5]
    seen=[]
    def pose(name):
        seen.append(name)
        return root_pose
    backend=SimpleNamespace(spec=SimpleNamespace(world_frame="world"),link_pose=pose)
    np.testing.assert_allclose(camera_world_optical(definition,backend),compose(root_pose,definition.pose_optical))
    assert seen == [resolved]
    root_pose[0]+=.1
    np.testing.assert_allclose(camera_world_optical(definition,backend),compose(root_pose,definition.pose_optical))


def test_mounted_camera_attaches_component_without_moving_hand(monkeypatch):
    from dataclasses import replace
    data=spec_to_dict(load_spec(SPEC));data["cameras"]=[camera()]
    definition=replace(spec_from_dict(data).cameras[0],parent="left_hand:hand_base_link")
    mount=object();calls={}
    def mounted(name,entity,pose,*parameters):
        calls.update(entity=entity,pose=pose,parameters=parameters)
        return SimpleNamespace(set_perspective_parameters=lambda *args:None)
    backend=SimpleNamespace(spec=SimpleNamespace(world_frame="world"),
        camera_parent_entity=lambda parent:mount,scene=SimpleNamespace(add_mounted_camera=mounted))
    monkeypatch.setitem(sys.modules,"sapien",SimpleNamespace(Pose=lambda p,q:[*p,*q]))
    create_spec_camera(backend,definition)
    assert calls["entity"] is mount
    np.testing.assert_allclose(calls["pose"],optical_to_sapien_pose(definition.pose_optical))


def test_right_live_commands_remain_rejected():
    backend=object.__new__(SapienBackend)
    with pytest.raises(ValueError,match="only for the left"):
        backend.apply([ArmCommand("right",(0.,)*7,0.)])


def test_old_epoch_review_hides_entity_and_link_visuals_only(monkeypatch):
    bodies=[SimpleNamespace(visibility=1.) for _ in range(3)]
    entities=[SimpleNamespace(find_component_by_type=lambda kind,body=body:body) for body in bodies]
    monkeypatch.setitem(sys.modules,"sapien.render",SimpleNamespace(RenderBodyComponent=object))
    backend=SimpleNamespace(table=entities[0],fixtures=[entities[1]],
        gamepad=SimpleNamespace(articulation=SimpleNamespace(get_links=lambda:[SimpleNamespace(entity=entities[2])])),
        spec=SimpleNamespace(fixtures=[]),robot=None,right_robot=None)
    robot_only_review_visuals(backend)
    assert [b.visibility for b in bodies] == [0.,0.,0.]


def test_v42_changes_only_ego_camera_and_visual_shell():
    directory = SPEC.parent
    before = json.loads((directory / 'station_g20_v4_1.json').read_text())
    after = json.loads((directory / 'station_g20_v4_2.json').read_text())
    spec = load_spec(directory / 'station_g20_v4_2.json')
    assert spec.cameras[0].status == 'approximate'
    assert spec.cameras[0].intrinsics.width == 1280
    assert spec.cameras[0].pose_optical[2] == 1.492
    for d in (before, after):
        for key in ('name', 'notes', 'cameras'):
            d.pop(key)
        d['fixtures'] = [f for f in d['fixtures'] if f['name'] != 'camera_3_body']
    assert after == before


def test_v43_applies_archived_extrinsics_with_existing_intrinsics_and_shell_offset():
    directory = SPEC.parent
    before = json.loads((directory / 'station_g20_v4_2.json').read_text())
    after = json.loads((directory / 'station_g20_v4_3.json').read_text())
    archive = ROOT / 'references/calibration/2026-10-20-ego/measured'
    extrinsics = json.loads((archive / 'extrinsics_project_world.json').read_text())
    captured = json.loads((archive / 'intrinsics.json').read_text())
    loaded = load_spec(directory / 'station_g20_v4_3.json')
    old_camera, new_camera = before['cameras'][0], after['cameras'][0]
    assert loaded.cameras[0].status == 'estimated'
    np.testing.assert_allclose(pose_to_matrix(new_camera['pose_optical']),
                               extrinsics['T_project_world_camera_optical'], atol=1e-14)
    assert new_camera['intrinsics'] == old_camera['intrinsics']
    intrinsic = loaded.cameras[0].intrinsics
    np.testing.assert_allclose(
        [intrinsic.fx, intrinsic.fy, intrinsic.cx, intrinsic.cy],
        [captured['K'][0][0], captured['K'][1][1], captured['K'][0][2], captured['K'][1][2]],
        atol=.0004, rtol=0)
    assert [intrinsic.width, intrinsic.height] == [captured['width'], captured['height']]
    assert list(intrinsic.distortion) == captured['D']
    offsets = []
    for data, camera in ((before, old_camera), (after, new_camera)):
        shell = next(f for f in data['fixtures'] if f['name'] == 'camera_3_body')
        offsets.append(np.linalg.inv(pose_to_matrix(camera['pose_optical'])) @ pose_to_matrix(shell['pose']))
        for key in ('name', 'notes', 'cameras'):
            data.pop(key)
        data['fixtures'] = [f for f in data['fixtures'] if f['name'] != 'camera_3_body']
    np.testing.assert_allclose(offsets[0], offsets[1], atol=1e-14)
    assert before == after
