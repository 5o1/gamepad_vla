"""Geometry, independent panes and disabled-camera scheduling."""
import sys
from types import SimpleNamespace
import numpy as np
import pytest
from gamepad_vla.workstation.auxiliary_views import (
    AUXILIARY_VIEWS, ATLAS_WIDTH, ATLAS_HEIGHT, CAMERA_OFFSETS,
    AuxiliaryCameras, atlas_tiles, camera_vectors, selected_views, frame_capacity,
)


def test_gamepad_relative_views_follow_full_scene_pose():
    base=np.eye(4)
    moved=np.array([[0.,-1.,0.,.65],[1.,0.,0.,.62],[0.,0.,1.,.99],[0.,0.,0.,1.]])
    for name in CAMERA_OFFSETS:
        old=camera_vectors(base,name);new=camera_vectors(moved,name)
        for i in (0,1):assert np.allclose(new[i],moved[:3,:3]@old[i]+moved[:3,3])
        assert np.allclose(new[2],moved[:3,:3]@old[2])
    position,target,up=camera_vectors(moved,'overview')
    assert np.allclose(target,moved[:3,3]) and np.allclose(position-target,[0,0,1.1])
    assert np.allclose(up,[1,0,0])


def test_fixed_tiles_do_not_overlap_and_fit_existing_mmap():
    tiles=atlas_tiles();coverage=np.zeros((ATLAS_HEIGHT,ATLAS_WIDTH),np.uint8)
    for t in tiles.values():coverage[t['y']:t['y']+t['height'],t['x']:t['x']+t['width']]+=1
    assert len(tiles)==6 and (coverage==1).all()
    assert frame_capacity(ATLAS_WIDTH,ATLAS_HEIGHT)==ATLAS_WIDTH*ATLAS_HEIGHT*3+65536
    for width,height in ((1920,1080),(2560,1440)):
        coverage=np.zeros((height,width),np.uint8)
        for t in atlas_tiles({'auxiliary':dict(width=width,height=height)}).values():
            coverage[t['y']:t['y']+t['height'],t['x']:t['x']+t['width']]+=1
        assert (coverage==1).all(),'Every row and column must belong to exactly one tile' 
    assert selected_views(['right','face','right'])==['face','right']
    for value in ('face',None,['bogus']):
        with pytest.raises(ValueError):selected_views(value)


@pytest.mark.parametrize('size',[(1920,1080),(2560,1440)])
def test_closed_cameras_never_take_picture(monkeypatch,size):
    # Test scheduling with real pixel assembly and fake camera calls, no renderer needed.
    scene_module=SimpleNamespace(look_at=lambda *args:args)
    monkeypatch.setitem(sys.modules,'xbox_gamepad_physics.examples.scene',scene_module)
    from gamepad_vla.workstation import collection_camera
    monkeypatch.setattr(collection_camera,'color_to_rgb8',lambda value:value)
    calls=[]
    class Camera:
        entity=SimpleNamespace(set_pose=lambda pose:None)
        def __init__(self,name,w,h):self.name,self.w,self.h=name,w,h
        def take_picture(self):calls.append(self.name)
        def get_picture(self,kind):return np.ones((self.h,self.w,3),np.uint8)
    scene=SimpleNamespace(add_camera=lambda name,w,h,*args:Camera(name,w,h),update_render=lambda:None)
    backend=SimpleNamespace(scene=scene,spec=SimpleNamespace(gamepad=SimpleNamespace(root_pose=(0,0,0,1,0,0,0))))
    cameras=AuxiliaryCameras(backend,{'auxiliary':dict(width=size[0],height=size[1])})
    assert not cameras.capture([]).any() and calls==[]
    cameras.capture(['face','overview']);cameras.capture(['overview'])
    assert calls==['aux_face','aux_overview','aux_overview']
    assert set(cameras.cameras)=={'face','overview'}


def test_v3_approach_camera_is_above_controller_and_on_camera_3_side():
    from pathlib import Path
    from gamepad_vla.workstation.spec import load_spec,pose_to_matrix
    root=Path(__file__).parents[1]
    spec=load_spec(root/'configs/workcell/placement_trials/gamepad_fixture_v3.json')
    transform=pose_to_matrix(spec.gamepad.root_pose)
    position,_,_=camera_vectors(transform,'front')
    reference=np.array(next(c for c in spec.cameras if c.name=='camera_3').pose_optical[:3])
    origin=transform[:3,3]
    assert position[2]>origin[2]+.05
    assert np.dot(position-origin,reference-origin)>0
