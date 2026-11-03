"""Fixture schema, multi-object import and native collision policy regression."""
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from gamepad_vla.workstation.fixture_collision import (
    convex_objects, add_mujoco_fixture, add_sapien_fixture_collision,
    configure_sapien_shapes, CONTACT_OFFSET, FIXTURE_GAMEPAD_IGNORE_BIT,
)
from gamepad_vla.workstation.spec import spec_from_dict, spec_to_dict

ROOT = Path(__file__).parents[1]
# Outward tetrahedron, with two copies in a single global OBJ index space.
OBJ = 'o one\nv 0 0 0\nv .01 0 0\nv 0 .01 0\nv 0 0 .01\nf 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n'


def test_convex_schema_roundtrip_and_alternative():
    data = json.loads((ROOT/'configs/workcell/station_g20_v4.json').read_text())
    f = dict(name='fixture', type='convex', pose=(0,0,0,1,0,0,0), mesh='test.obj',
             collision=True, status='design', source='test')
    data['fixtures'] = [f]
    result = spec_from_dict(data)
    assert spec_to_dict(result)['fixtures'] == [f]
    f.update(type='mesh', mesh='visual.glb', convex_mesh='hulls.obj')
    assert spec_to_dict(spec_from_dict(data))['fixtures'] == [f]
    f['type'] = 'convex'
    with pytest.raises(ValueError, match='convex_mesh'):
        spec_from_dict(data)


def test_multi_object_vertices_and_invalid_hulls(tmp_path):
    obj = tmp_path/'hulls.obj'
    obj.write_text(OBJ + OBJ.replace('one', 'two').replace('f 1 3 2', 'f 5 7 6').replace('f 1 2 4', 'f 5 6 8').replace('f 1 4 3', 'f 5 8 7').replace('f 2 3 4', 'f 6 7 8'))
    hulls = convex_objects(obj)
    assert len(hulls) == 2
    np.testing.assert_array_equal(hulls[0][0], hulls[1][0])
    obj.write_text(OBJ.replace('f 2 3 4\n', ''))
    with pytest.raises(ValueError, match='closed'):
        convex_objects(obj)
    obj.write_text(OBJ.replace('v 0 0 .01', 'v .002 .003 0'))
    with pytest.raises(ValueError, match='volume'):
        convex_objects(obj)
    obj.write_text(OBJ.replace('.01', 'nan', 1))
    with pytest.raises(ValueError, match='finite'):
        convex_objects(obj)


def test_native_builders_choose_triangles_or_each_convex_object(tmp_path):
    obj = tmp_path/'hulls.obj'; obj.write_text(OBJ)
    fixture = SimpleNamespace(name='fixture', type='mesh', collision=True, mesh='exact.glb',
        convex_mesh=str(obj), pose=[0,0,0,1,0,0,0])
    calls = []
    builder = SimpleNamespace(add_nonconvex_collision_from_file=lambda p:calls.append(('triangles',p)),
        add_multiple_convex_collisions_from_file=lambda p,**k:calls.append(('convex',p,k)))
    add_sapien_fixture_collision(builder, fixture)
    assert calls == [('triangles','exact.glb')]
    fixture.type, fixture.mesh = 'convex', str(obj)
    add_sapien_fixture_collision(builder, fixture)
    assert calls[-1][2] == dict(decomposition='none')
    geoms, meshes = [], []
    parent = SimpleNamespace(add_mesh=lambda **kw:meshes.append(kw),
        worldbody=SimpleNamespace(add_geom=lambda **kw:geoms.append(kw)))
    mj = SimpleNamespace(mjtGeom=SimpleNamespace(mjGEOM_MESH='mesh'))
    add_mujoco_fixture(parent, mj, fixture)
    assert len(meshes) == len(geoms) == 1
    assert geoms[0]['contype'] == 8 and geoms[0]['conaffinity'] == 6
    assert geoms[0]['margin'] == CONTACT_OFFSET
    # MuJoCo accepts a pair if either direction shares a bit. Both must be off.
    assert not (8 & (2|4|16) or 1 & 6)
    assert (8 & (1|4|8)) or (2 & 6)


def test_sapien_filter_preserves_asset_internal_and_robot_bits():
    class Shape:
        def __init__(self, groups): self.groups = groups
        def get_collision_groups(self): return self.groups
        def set_collision_groups(self, groups): self.groups = groups
        def set_contact_offset(self, value): self.offset = value
    fixture, pad, hand, arm = [Shape(g) for g in ([1,1,0,0], [1,1,0x80000000,0x5842], [1,1,0x10000,0], [1,1,0x10000,0])]
    for sh in (fixture,pad,hand,arm):
        configure_sapien_shapes(SimpleNamespace(get_collision_shapes=lambda:[sh]), exclude_fixture_gamepad=sh in (fixture,pad))
        assert sh.offset == .0002
    def allowed(a,b):
        x,y=a.groups,b.groups
        return bool((x[0]&y[1] or y[0]&x[1]) and (not x[2]&y[2] or x[3]!=y[3]))
    assert not allowed(fixture,pad)
    assert allowed(fixture,hand) and allowed(fixture,arm)
    assert not allowed(pad,pad) and not allowed(hand,arm)
    assert pad.groups[2] == 0x80000000|FIXTURE_GAMEPAD_IGNORE_BIT


def test_sapien_static_entity_uses_concrete_component(monkeypatch):
    import sys
    from gamepad_vla.workstation.fixture_collision import sapien_shapes
    static_type, dynamic_type = object(), object()
    component = SimpleNamespace(get_collision_shapes=lambda:['static-shape'])
    actor = SimpleNamespace(find_component_by_type=lambda kind:component if kind is static_type else None)
    monkeypatch.setitem(sys.modules, 'sapien', SimpleNamespace(physx=SimpleNamespace(
        PhysxRigidStaticComponent=static_type, PhysxRigidDynamicComponent=dynamic_type)))
    assert sapien_shapes(actor) == ['static-shape']


def test_load_spec_resolves_convex_alternative_in_workspace(tmp_path, monkeypatch):
    from gamepad_vla.workstation.spec import load_spec
    data = json.loads((ROOT/'configs/workcell/station_g20_v4.json').read_text())
    data['fixtures'] = [dict(name='fixture', type='mesh', pose=[0,0,0,1,0,0,0],
        mesh='workspace://geometry.glb', convex_mesh='workspace://hulls.obj',
        collision=True, status='design', source='test')]
    path = tmp_path/'spec.json'; path.write_text(json.dumps(data))
    monkeypatch.setenv('GAMEPAD_VLA_WORKSPACE', str(tmp_path))
    f = load_spec(path).fixtures[0]
    assert f.mesh == str(tmp_path/'geometry.glb')
    assert f.convex_mesh == str(tmp_path/'hulls.obj')
