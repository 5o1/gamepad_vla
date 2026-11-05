"""Every PNG must come from its own native-resolution camera exposure."""
import json
import sys
from types import SimpleNamespace
import numpy as np
import pytest
pytest.importorskip('PIL')
from PIL import Image
from gamepad_vla.workstation.spec import IntrinsicsSpec, CameraSpec
from gamepad_vla.workstation.scene_snapshot import render_pending_snapshot


@pytest.mark.parametrize('native_size',[(1280,720),(1920,1080)])
def test_seven_independent_native_cameras_and_extrinsics(tmp_path,monkeypatch,native_size):
    native_width,native_height=native_size
    cfg=dict(spec_text='{}',code_identity={'test':'fake'},width=native_width,height=native_height)
    from gamepad_vla.workstation import render_review
    monkeypatch.setitem(sys.modules,'xbox_gamepad_physics.examples.scene',SimpleNamespace(look_at=lambda *args:args))
    calls=[];created=[];last={'camera':None}
    class Camera:
        def __init__(self,name,w,h):
            self.name,self.width,self.height=name,w,h;self.number=len(created)+1
            self.entity=SimpleNamespace(set_pose=lambda pose:None,components=[self])
            self.entity.remove_component=lambda camera:self.entity.components.remove(camera)
            created.append(self)
        def take_picture(self):calls.append(self.name);last['camera']=self
        def get_picture(self,kind):
            assert last['camera'] is self,'Cannot reuse a stream or another camera exposure'
            return np.full((self.height,self.width,4),self.number/10,dtype=np.float32)
        def get_model_matrix(self):return np.eye(4)
        def get_intrinsic_matrix(self):return np.diag([1000.,1000.,1.])
    removed=[]
    scene=SimpleNamespace(remove_entity=removed.append,add_camera=lambda name,w,h,*args:Camera(name,w,h),update_render=lambda:None)
    monkeypatch.setattr(render_review,'create_spec_camera',lambda backend,definition:Camera(definition.name,definition.intrinsics.width,definition.intrinsics.height))
    intr=IntrinsicsSpec(1280,720,800,800,640,360,(), 'pinhole')
    spec=SimpleNamespace(name='frozen-test',left_arm=SimpleNamespace(joint_names=['l'+str(i) for i in range(7)]),right_arm=SimpleNamespace(joint_names=['r'+str(i) for i in range(7)]),gamepad=SimpleNamespace(root_pose=(0,0,0,1,0,0,0)),
                         cameras=[CameraSpec('camera_3','','','world',(0,0,0,1,0,0,0),intr,'test','fake')])
    pose=SimpleNamespace(to_transformation_matrix=lambda:np.eye(4))
    articulation=SimpleNamespace(get_active_joints=lambda:[SimpleNamespace(name='stick')],set_qpos=lambda q:None,get_root_pose=lambda:pose)
    backend=SimpleNamespace(spec=spec,scene=scene,gamepad=SimpleNamespace(articulation=articulation),fixtures=[SimpleNamespace(name='fixture',get_pose=lambda:pose)],set_review_state=lambda state:None)
    scene.render_system=SimpleNamespace(device='fake')
    frozen=dict(step=91,arm_q=[[0]*7]*2,hand_q=[[0]*21]*2,control_joint_positions={'stick':.1})
    (tmp_path/'state.json').write_text(json.dumps(frozen))
    render_pending_snapshot(backend,dict(folder=str(tmp_path),width=1280,height=720,max_bytes=64*1024*1024),cfg)
    result=json.loads((tmp_path/'render-result.json').read_text());assert 'error' not in result,result
    assert result['camera_count']==7 and len(calls)==7 and len(set(calls))==7
    assert set(result['cameras'])=={'camera_3','aux_face','aux_front','aux_back','aux_left','aux_right','aux_overview'}
    means=[]
    for meta in result['cameras'].values():
        with Image.open(tmp_path/meta['file']) as image:
            assert image.size==native_size;means.append(float(np.asarray(image).mean()))
        assert meta['source']=='independent_sapien_camera_take_picture'
        T=np.array(meta['world_from_camera_optical']);inverse=np.array(meta['camera_optical_from_world'])
        assert np.allclose(T@inverse,np.eye(4))
    assert len(set(means))==7
    assert len(removed)==7 and all(not c.entity.components for c in created)
    # Repeated snapshots take new pictures and release their native camera buffers.
    render_pending_snapshot(backend,dict(folder=str(tmp_path),width=1280,height=720,max_bytes=64*1024*1024),cfg)
    assert len(created)==14 and len(calls)==14 and len(removed)==14
    # SAPIEN mounted cameras share the robot link entity; keep its body intact.
    body=object();owner=SimpleNamespace(components=[body])
    owner.remove_component=lambda component:owner.components.remove(component)
    def mounted(backend,definition):
        camera=Camera(definition.name,definition.intrinsics.width,definition.intrinsics.height)
        camera.entity=owner;owner.components.append(camera);return camera
    monkeypatch.setattr(render_review,'create_spec_camera',mounted)
    failure=tmp_path/'limited';failure.mkdir();(failure/'state.json').write_text(json.dumps(frozen))
    render_pending_snapshot(backend,dict(folder=str(failure),width=1280,height=720,max_bytes=100),cfg)
    assert 'exceed' in json.loads((failure/'render-result.json').read_text())['error']
    assert owner.components==[body] and owner not in removed

