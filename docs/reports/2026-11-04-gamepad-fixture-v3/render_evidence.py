#!/usr/bin/env python3
"""Bounded CPU evidence capture through the existing operator_view mmap path.

No replacement scene builder or CAD renderer. A read-only camera hook records
SAPIEN's actual segmentation after operator_view takes its ordinary picture.
"""
from __future__ import annotations
import argparse,json,os,subprocess,sys,time,hashlib,importlib.metadata
from pathlib import Path
import numpy as np

def write(p,v):p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(v,indent=2)+'\n')

def worker(run):
    from gamepad_vla.workstation import sapien_backend,collection_camera
    from PIL import Image
    from xbox_gamepad_physics.teleop.stream.ipc import Latest,SNAPSHOT_CAPACITY
    holder={};seen=set();records=[]
    init=sapien_backend.SapienBackend.__init__
    def capture_init(self,*args,**kw):
        init(self,*args,**kw);holder['backend']=self
        holder['fixture_hashes']={f.name:hashlib.sha256(Path(f.mesh).read_bytes()).hexdigest() for f in self.spec.fixtures if f.type=='mesh' and f.name.startswith('v3_visual_')}
    sapien_backend.SapienBackend.__init__=capture_init
    convert=collection_camera.color_to_rgb8
    def capture_picture(color):
        rgb=convert(color);backend=holder['backend'];snapshot=Latest(run/'snapshot.mmap',SNAPSHOT_CAPACITY)
        try:state=json.loads(snapshot.read()[1])
        finally:snapshot.close()
        # The parent can change view.json while the previous picture is in
        # flight. Identify the camera from the picture actually converted,
        # and wait for actual joint q to match the requested command.
        actual={j.name:float(q) for j,q in zip(backend.gamepad.articulation.get_active_joints(),backend.gamepad.articulation.get_qpos())}
        if any(abs(actual[n]-float(v))>1e-7 for n,v in state['control_joint_positions'].items()):return rgb
        requested=json.loads((run/'view.json').read_text())['view'];preferred='camera_3' if requested=='camera_3' else 'operator_only'
        cameras=sorted(backend.scene.get_cameras(),key=lambda c:c.name!=preferred);camera=None
        for candidate in cameras:
            try:picture=candidate.get_picture('Color')
            except IndexError:continue
            if picture.shape==color.shape and np.array_equal(picture,color):camera=candidate;break
        if camera is None:raise RuntimeError('Cannot identify the actual operator_view camera picture')
        view='camera_3' if camera.name=='camera_3' else 'operator';key=(state['command_id'],view)
        if key not in seen:
            seg=camera.get_picture('Segmentation')
            links=backend.gamepad.articulation.get_links();entity_ids=[int(l.entity.per_scene_id) for l in links]
            mask=np.isin(seg[:,:,1],entity_ids);yy,xx=np.where(mask)
            proxies=[int(f.entity.per_scene_id) if hasattr(f,'entity') else int(f.per_scene_id) for f in backend.fixtures if f.name.startswith('v3_proxy_')]
            proxy_pixels=int(np.isin(seg[:,:,1],proxies).sum())
            if not len(xx):raise RuntimeError('Empty controller segmentation mask')
            label=f"{state['command_id']:02d}_{view}";Image.fromarray(mask.astype(np.uint8)*255).save(run/(label+'_mask.png'))
            Image.fromarray(rgb).save(run/(label+'.png'))
            margins=[int(xx.min()),int(rgb.shape[1]-1-xx.max()),int(yy.min()),int(rgb.shape[0]-1-yy.max())]
            meshactors=[a for a in backend.fixtures if a.name.startswith('v3_visual_')]
            record=dict(command_id=state['command_id'],view=view,file=label+'.png',mask_file=label+'_mask.png',mask_pixels=len(xx),
                mask_bounds_px=[[int(xx.min()),int(yy.min())],[int(xx.max()),int(yy.max())]],margins_px=dict(zip(('left','right','top','bottom'),margins)),
                collision_proxy_segmentation_pixels=proxy_pixels,mask_avoids_boundary=min(margins)>0,mask_margin_20px=min(margins)>=20,entity_ids=entity_ids,
                gamepad_root_matrix=backend.gamepad.articulation.get_root_pose().to_transformation_matrix().tolist(),
                camera_model=camera.get_model_matrix().tolist(),camera_intrinsic=camera.get_intrinsic_matrix().tolist(),
                render_device=str(backend.scene.render_system.device),visual_mesh_actors=len(meshactors),
                visual_mesh_actor_poses={a.name:a.get_pose().to_transformation_matrix().tolist() for a in meshactors},
                gamepad_joint_state={j.name:float(q) for j,q in zip(backend.gamepad.articulation.get_active_joints(),backend.gamepad.articulation.get_qpos())})
            records.append(record);write(run/'segmentation.json',records);write(run/'fixture_hashes.json',holder['fixture_hashes']);seen.add(key)
        return rgb
    collection_camera.color_to_rgb8=capture_picture
    from gamepad_vla.workstation.operator_view import render
    render(run)

def capture(args):
    from PIL import Image
    from gamepad_vla.workstation.spec import load_spec,pose_to_matrix
    from gamepad_vla.workstation.operator_view import load_config
    from xbox_gamepad_physics.motion import load_motion_contract
    from xbox_gamepad_physics.teleop.stream.ipc import Latest,SNAPSHOT_CAPACITY,FRAME_CAPACITY,unpack
    spec=load_spec(args.spec);run=args.run;run.mkdir(parents=True,exist_ok=False)
    cfg=load_config(Path('configs/render/operator_view.yaml'))
    cfg.update(collection=True,review_camera=True,view='camera_3',width=1280,height=720,fps=1,resources=os.environ['XBOX_GAMEPAD_PHYSICS_RESOURCES'],pip=dict(enabled=False),overview=dict(enabled=False))
    W=pose_to_matrix(spec.gamepad.root_pose);r=W[:3,:3].T
    # Operator-only side view uses the same existing camera adapter.
    cfg['camera']=dict(position_gamepad=(r@(np.array([.74,-.02,1.10])-W[:3,3])).tolist(),target_gamepad=(r@(np.array([.67,.6252,.91])-W[:3,3])).tolist(),up_gamepad=(r@np.array([0.,0.,1.])).tolist(),fovy_degrees=40.)
    write(run/'config.json',cfg);(run/'spec_path.txt').write_text(str(args.spec.resolve()))
    snap=Latest(run/'snapshot.mmap',SNAPSHOT_CAPACITY,True);frames=Latest(run/'frame.mmap',FRAME_CAPACITY,True)
    projection=json.loads(args.projection.read_text());states=[{}]
    for c in projection['worst_cases'].values():
        if c['joint_state'] not in states:states.append(c['joint_state'])
    for endpoint in projection.get('all_joint_endpoints',{}).values():
        if endpoint not in states:states.append(endpoint)
    contract=load_motion_contract('full_controls');man=[]
    with (run/'render.log').open('w') as log:
        proc=subprocess.Popen([sys.executable,__file__,'worker','--run',str(run)],stdout=log,stderr=subprocess.STDOUT)
        try:
            for index,joints in enumerate(states,1):
                state=dict(arm_q=[list(spec.left_arm.home_q),list(spec.right_arm.home_q)],hand_q=[list(spec.left_hand.home_q),list(spec.right_hand.home_q)],command_id=index,command_applied_ns=None,control_joint_positions={n:float(joints.get(n,0)) for n in contract.joint_by_name})
                snap.write(json.dumps(state).encode())
                for view in (['camera_3','operator'] if index==1 else ['camera_3']):
                    tmp=run/'view.tmp';write(tmp,dict(view=view));tmp.replace(run/'view.json');end=time.monotonic()+180
                    while time.monotonic()<end:
                        frame=frames.read()
                        if frame:
                            meta,rgb=unpack(frame[1])
                            if meta.get('command_id')==index and meta.get('view')==view:
                                name=f'{index:02d}_{view}.png';Image.fromarray(np.frombuffer(rgb,np.uint8).reshape(720,1280,3)).save(run/name);man.append(dict(file=name,view=view,command_id=index,joints=joints));break
                        if proc.poll() is not None:raise RuntimeError('operator_view exited; see render.log')
                        time.sleep(.1)
                    else:raise TimeoutError(view)
        finally:
            (run/'stop').touch()
            try:proc.wait(timeout=15)
            except subprocess.TimeoutExpired:proc.kill();proc.wait()
            snap.close();frames.close()
    records=json.loads((run/'segmentation.json').read_text());expected=W
    allpose=[]
    for record in records:
        err=float(np.max(abs(np.asarray(record['gamepad_root_matrix'])-expected)))
        fixture_err=max(float(np.max(abs(np.asarray(T)-expected))) for T in record['visual_mesh_actor_poses'].values())
        record.update(root_matrix_max_error=err,fixture_matrix_max_error=fixture_err)
        allpose.append(err<1e-6 and fixture_err<1e-6)
    result=dict(executed=True,resource_versions={n:importlib.metadata.version(n) for n in ('sapien','xbox-gamepad-physics')},fixture_mesh_hashes=json.loads((run/'fixture_hashes.json').read_text()),exit_code=proc.returncode,pipeline='operator_view -> SapienBackend -> public robotwin.create_scene / load_into_scene -> mmap',
        states=states,frames=records,pass_all_camera_3_masks=all(r['mask_margin_20px'] for r in records if r['view']=='camera_3'),pass_cad_alignment=all(allpose),gpu=False)
    write(run/'verification.json',result)
    if proc.returncode or not result['pass_all_camera_3_masks'] or not result['pass_cad_alignment']:raise RuntimeError('render cross-check failed')

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('mode',choices=('capture','worker'));ap.add_argument('--run',type=Path,required=True);ap.add_argument('--spec',type=Path);ap.add_argument('--projection',type=Path);a=ap.parse_args()
    worker(a.run) if a.mode=='worker' else capture(a)
