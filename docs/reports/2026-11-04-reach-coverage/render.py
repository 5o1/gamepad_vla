"""Z evidence capture through operator_view, with actual-link Y annotations."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import numpy as np


def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')


def key(state):
    return ','.join(f'{v:.7f}' for v in state['right_arm']+state['right_hand'])


def worker(run):
    from gamepad_vla.workstation import sapien_backend
    from sapien.render import RenderBodyComponent
    original=sapien_backend.SapienBackend.set_review_state
    records={}
    def capture(self,state):
        original(self,state)
        hidden=[]
        for link in self.robot.get_links():
            body=link.entity.find_component_by_type(RenderBodyComponent)
            if body is not None:
                body.visibility=0.;hidden.append(dict(link=link.name,visibility=float(body.visibility)))
        k=key(state)
        if k not in records:
            robot=self.right_robot
            records[k]=dict(q_by_name={j.name:float(q) for j,q in zip(robot.get_active_joints(),robot.get_qpos())},
                render_device=str(self.scene.render_system.device),
                links={l.name:l.get_entity_pose().to_transformation_matrix().tolist() for l in robot.get_links()},
                cameras={c.name:dict(model=c.get_model_matrix().tolist(),intrinsic=c.get_intrinsic_matrix().tolist())
                    for c in self.scene.get_cameras()},hidden_left_render_bodies=hidden)
            write(run/'actual-links.json',records)
    sapien_backend.SapienBackend.set_review_state=capture
    from gamepad_vla.workstation.operator_view import render
    render(run)


def capture(args):
    from PIL import Image
    from gamepad_vla.workstation.spec import load_spec,pose_to_matrix
    from gamepad_vla.workstation.operator_view import load_config
    from gamepad_vla.workstation.g20_mapping import G20_JOINT_NAMES,G20_MIMIC_JOINTS
    from xbox_gamepad_physics.motion import load_motion_contract
    from xbox_gamepad_physics.teleop.stream.ipc import Latest,SNAPSHOT_CAPACITY,FRAME_CAPACITY,unpack
    specpath=args.source/'final/rank-1/spec.json';spec=load_spec(specpath)
    result=json.loads((args.run/'solve/d050-h220-t+10/right/result.json').read_text())
    targets=[dict(label='strict_hold',check=result['strict']['check'],gamepad_q={}),
        dict(label='realistic_hold',check=result['realistic']['check'],gamepad_q={})]
    states=[s for s in result['realistic']['states'] if s['control'] in result['realistic']['controls']]
    if states:
        preferred=[s for s in states if s['control'] not in result['strict']['controls']] or states
        s=max(preferred,key=lambda s:s['motion']['centroid_displacement_mm'])
        # Recover displaced-control joint values from unchanged S goal contract.
        # These values are exported with the solver result by finalize.py.
        gamepad_q=s['gamepad_q']
        targets.append(dict(label='realistic_press_'+s['control'],check=s['check'],gamepad_q=gamepad_q,motion=s['motion']))
    run=args.run/'render';run.mkdir(exist_ok=False)
    cfg=load_config(Path('configs/render/operator_view.yaml'))
    cfg.update(collection=True,review_camera=True,view='camera_3',width=1280,height=720,fps=2,
        resources=os.environ['XBOX_GAMEPAD_PHYSICS_RESOURCES'],pip=dict(enabled=False),overview=dict(enabled=False))
    frame=pose_to_matrix(spec.gamepad.root_pose);r=frame[:3,:3].T
    cfg['camera']=dict(position_gamepad=(r@(np.array([.35,-.75,1.16])-frame[:3,3])).tolist(),
        target_gamepad=(r@(np.array([.35,.6252,1.10])-frame[:3,3])).tolist(),
        up_gamepad=(r@np.array([0.,0.,1.])).tolist(),fovy_degrees=43)
    write(run/'config.json',cfg);(run/'spec_path.txt').write_text(str(specpath))
    snap=Latest(run/'snapshot.mmap',SNAPSHOT_CAPACITY,True);frames=Latest(run/'frame.mmap',FRAME_CAPACITY,True)
    manifests=[]
    with (run/'render.log').open('w') as log:
        proc=subprocess.Popen([sys.executable,__file__,'worker','--run',str(run)],stdout=log,stderr=subprocess.STDOUT)
        try:
            for index,t in enumerate(targets,1):
                hand=dict(t['check']['hand_q'])
                for name,(parent,mult) in G20_MIMIC_JOINTS['right'].items():hand[name]=hand[parent]*mult
                state=dict(arm_q=[list(spec.left_arm.home_q),t['check']['arm_q']],
                    hand_q=[list(spec.left_hand.home_q),[hand[n] for n in G20_JOINT_NAMES]],
                    command_id=index,command_applied_ns=None,candidate_side='right',
                    control_joint_positions={n:t['gamepad_q'].get(n,0.) for n in load_motion_contract('full_controls').joint_by_name})
                snap.write(json.dumps(state).encode())
                k=key(dict(right_arm=state['arm_q'][1],right_hand=state['hand_q'][1]))
                for view,label in [('camera_3','camera_3'),('operator','side')]:
                    temporary=run/'view.tmp';write(temporary,dict(view=view));temporary.replace(run/'view.json')
                    end=time.monotonic()+150
                    while time.monotonic()<end:
                        row=frames.read()
                        if row:
                            meta,rgb=unpack(row[1])
                            if meta.get('command_id')==index and meta.get('view')==view:
                                destination=run/(t['label']+'_'+label+'_raw.png')
                                Image.fromarray(np.frombuffer(rgb,np.uint8).reshape(720,1280,3)).save(destination)
                                manifests.append(dict(label=t['label'],view=label,file=str(destination),key=k,
                                    check=t['check'],motion=t.get('motion'),metadata=meta));break
                        if proc.poll() is not None:raise RuntimeError('renderer exited; see render.log')
                        time.sleep(.1)
                    else:raise TimeoutError('render capture '+t['label']+'/'+view)
        finally:
            (run/'stop').touch()
            try:proc.wait(timeout=15)
            except subprocess.TimeoutExpired:proc.kill();proc.wait()
            snap.close();frames.close()
    write(run/'manifest.json',dict(images=manifests,returncode=proc.returncode))
    if proc.returncode:raise RuntimeError('renderer failed')


def annotate(args):
    from PIL import Image,ImageDraw,ImageFont
    # Reuse Y's independent axis-line reconstruction on SAPIEN actual links.
    sys.path.insert(0,str(args.source.parent/'arm-posture-y-20261104'))
    from audit import axis_points
    case=next(c for c in json.loads((args.source.parent/'arm-posture-y-20261104/native.json').read_text())
        if c['rank']==1 and c['side']=='right')
    run=args.run/'render';manifest=json.loads((run/'manifest.json').read_text())
    actual=json.loads((run/'actual-links.json').read_text())
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',18)
    checks=[]
    for row in manifest['images']:
        a=actual[row['key']];links={k:np.array(v) for k,v in a['links'].items()}
        children=case['joint_children'];axes=np.array(case['joint_local_axes'])
        pts=axis_points([links[k][:3,3] for k in children],[links[k][:3,:3]@v for k,v in zip(children,axes)])
        error=max(np.linalg.norm(np.array(pts[k])-row['check']['natural_points_m'][k])*1000 for k in pts)
        assert error<=1.,'native/actual render joint centres differ by >1 mm'
        cam=a['cameras']['operator_only' if row['view']=='side' else 'camera_3']
        model=np.array(cam['model']);intr=np.array(cam['intrinsic'])
        def uv(p):
            v=(np.array(p)-model[:3,3])@model[:3,:3]
            return tuple(int(round(x)) for x in [intr[0,0]*v[0]/-v[2]+intr[0,2],intr[1,2]-intr[1,1]*v[1]/-v[2]])
        def visible(p):
            v=(np.array(p)-model[:3,3])@model[:3,:3]
            x,y=uv(p)
            return v[2]<0 and 0<=x<1280 and 90<=y<670
        im=Image.open(row['file']).convert('RGB');draw=ImageDraw.Draw(im)
        def label(p,text,color=(255,255,255)):
            box=draw.textbbox(p,text,font=font);draw.rectangle((box[0]-3,box[1]-2,box[2]+3,box[3]+2),fill=(15,20,26));draw.text(p,text,font=font,fill=color)
        poly=[uv(pts[k]) for k in ('shoulder','elbow','wrist')]
        for segment_start,segment_end in [('shoulder','elbow'),('elbow','wrist')]:
            if visible(pts[segment_start]) and visible(pts[segment_end]):
                draw.line([uv(pts[segment_start]),uv(pts[segment_end])],fill=(50,255,100),width=5)
        for name,(x,y),letter,color in zip(('shoulder','elbow','wrist'),poly,['S','E','W'],[(255,65,65),(255,220,40),(30,220,255)]):
            if visible(pts[name]):
                draw.ellipse((x-7,y-7,x+7,y+7),fill=color,outline=(0,0,0),width=2)
                label((min(1190,x+10),max(92,y-26)),'R-'+letter,color)
            else:label((1020,100+['S','E','W'].index(letter)*27),letter+' outside view',color)
        if row['view']=='side':
            s=np.array(pts['shoulder']);e=np.array(pts['elbow']);w=np.array(pts['wrist'])
            h=w-e;h[2]=0;h*=.20/np.linalg.norm(h)
            draw.line([uv(s),uv(s+[0,0,-.20])],fill=(255,65,65),width=2)
            draw.line([uv(e),uv(e+h)],fill=(255,220,40),width=2)
        draw.rectangle((0,0,1280,85),fill=(15,20,26))
        label((15,8),'d050-h220-t+10 | '+row['label']+' | '+row['view'])
        label((15,36),'RIGHT selected. LEFT arm and hand hidden. S=shoulder, E=elbow, W=wrist; 3D centres (x-ray).')
        m=row['check']['natural']
        draw.rectangle((0,670,1280,720),fill=(15,20,26))
        label((15,675),f'upper {m["upper_tilt_deg"]:.2f} deg | elbow {m["elbow_angle_deg"]:.2f} deg | forearm {m["forearm_pitch_deg"]:+.2f} deg | heading {m["forearm_heading_deg"]:.2f} deg')
        if row['motion']:
            label((15,696),f'Palm centroid shift {row["motion"]["centroid_displacement_mm"]:.2f} mm | rotation {row["motion"]["rotation_deg"]:.2f} deg')
        dest=args.run/'images'/(row['label']+'_'+row['view']+'.png');dest.parent.mkdir(exist_ok=True);im.save(dest)
        checks.append(dict(image=str(dest),max_actual_link_difference_mm=error,points_m=pts,
            pixels=poly,hidden_left_render_bodies=a['hidden_left_render_bodies']))
    write(run/'annotation-checks.json',checks)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('stage',choices=['worker','capture','annotate'])
    p.add_argument('--run',type=Path,required=True);p.add_argument('--source',type=Path)
    a=p.parse_args()
    if a.stage=='worker':worker(a.run)
    elif a.stage=='capture':capture(a)
    else:annotate(a)
