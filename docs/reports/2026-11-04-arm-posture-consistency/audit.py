"""Task Y bounded CPU audit. Reuses operator_view.render; never steps physics.

Run native with the fixed-gamepad environment, render with SAPIEN's environment,
then compare with the fixed-gamepad environment. No dependency installation.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import numpy as np


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def axis_points(origins, axes):
    """Independent least-squares reconstruction from actual joint axis lines."""
    p, a = np.asarray(origins), np.asarray(axes)
    t = np.linalg.lstsq(np.c_[a[0], -a[1]], p[1]-p[0], rcond=None)[0]
    shoulder = (p[0]+t[0]*a[0]+p[1]+t[1]*a[1])/2
    projectors = np.eye(3)-a[4:, :, None]*a[4:, None, :]
    mat = projectors.sum(axis=0)
    anchor = p[4:].mean(axis=0)
    wrist = anchor+np.linalg.lstsq(mat, np.einsum('kij,kj->i',projectors,p[4:])-mat@anchor,rcond=None)[0]
    line = wrist-shoulder; line /= np.linalg.norm(line)
    t = np.linalg.lstsq(np.c_[a[3], -line], shoulder-p[3], rcond=None)[0]
    elbow = p[3]+t[0]*a[3]
    return dict(shoulder=shoulder.tolist(), elbow=elbow.tolist(), wrist=wrist.tolist())


def angles(points):
    u = np.array(points['elbow'])-points['shoulder'];u /= np.linalg.norm(u)
    f = np.array(points['wrist'])-points['elbow'];f /= np.linalg.norm(f)
    return dict(upper_tilt_deg=float(np.degrees(np.arccos(-u[2]))),
                forearm_pitch_deg=float(np.degrees(np.arcsin(f[2]))),
                elbow_angle_deg=float(np.degrees(np.arccos(np.clip(-u@f,-1,1)))))


def native(args):
    import mujoco as mj
    from gamepad_vla.feasibility import clean_urdf
    from gamepad_vla.workstation.spec import load_spec,pose_to_matrix
    from k1_natural_reference import ArmFeatures
    out=[]
    for rank in range(1,5):
        folder=args.source/'final'/f'rank-{rank}';spec=load_spec(folder/'spec.json')
        c=json.loads((folder/'candidates.json').read_text())
        for side in ('left','right'):
            arm=getattr(spec,side+'_arm');hand=getattr(spec,side+'_hand')
            feature=ArmFeatures(arm,hand);q=c['sides'][side]['check']['arm_q'];f=feature.frames(q)
            work=args.output/'native-models'/f'rank-{rank}'/side;work.mkdir(parents=True,exist_ok=True)
            urdf=work/'arm.urdf';clean_urdf(arm.urdf,urdf,arm=arm)
            parent=mj.MjSpec();robot=mj.MjSpec.from_file(str(urdf))
            frame=parent.worldbody.add_frame(pos=arm.base_pose[:3],quat=arm.base_pose[3:])
            parent.attach(robot,prefix='arm_',frame=frame);model=parent.compile();data=mj.MjData(model)
            for name,value in zip(arm.joint_names,q):data.qpos[model.joint('arm_'+name).qposadr[0]]=value
            mj.mj_forward(model,data)
            frames={}
            for name in [j['child'] for j in feature.joints]+[arm.tip_link]:
                b=model.body('arm_'+name).id;t=np.eye(4);t[:3,:3]=data.xmat[b].reshape(3,3);t[:3,3]=data.xpos[b];frames[name]=t
            origins=[frames[j['child']][:3,3] for j in feature.joints]
            axes=[frames[j['child']][:3,:3]@j['axis'] for j in feature.joints]
            metric={k:f[k][0].tolist() for k in ('shoulder','elbow','wrist')}
            metric['palm_root']=f['palm'][0,:3,3].tolist()
            metric['palm_centroid']=(f['palm'][0,:3,:3]@feature.palm_point_local+f['palm'][0,:3,3]).tolist()
            points=axis_points(origins,axes);palm=frames[arm.tip_link]@pose_to_matrix(hand.mount)
            points['palm_root']=palm[:3,3].tolist();points['palm_centroid']=(palm[:3,:3]@feature.palm_point_local+palm[:3,3]).tolist()
            out.append(dict(rank=rank,id=c['placement']['id'],side=side,q=q,metric=metric,
                native=points,angles=angles(metric),joint_children=[j['child'] for j in feature.joints],
                joint_local_axes=[j['axis'].tolist() for j in feature.joints],palm_local=feature.palm_point_local.tolist(),
                native_link_frames={k:v.tolist() for k,v in frames.items()}))
    write(args.output/'native.json',out)
    print('Native/metric cases:',len(out))


def worker(args):
    from gamepad_vla.workstation import sapien_backend
    from gamepad_vla.workstation.spec import pose_to_matrix
    original=sapien_backend.SapienBackend.set_review_state
    output=args.output/'render-audit.json'
    def capture(self,state):
        original(self,state)
        if output.exists():return
        rows={}
        for side,robot in [('left',self.robot),('right',self.right_robot)]:
            links=robot.get_links();pin=robot.create_pinocchio_model();pin.compute_forward_kinematics(robot.get_qpos())
            root=robot.get_root_pose().to_transformation_matrix()
            rows[side]=dict(q_by_name={j.name:float(q) for j,q in zip(robot.get_active_joints(),robot.get_qpos())},
                links={l.name:l.get_entity_pose().to_transformation_matrix().tolist() for l in links},
                pinocchio={l.name:(root@pin.get_link_pose(i).to_transformation_matrix()).tolist() for i,l in enumerate(links)})
        cams={c.name:dict(model=c.get_model_matrix().tolist(),intrinsic=c.get_intrinsic_matrix().tolist()) for c in self.scene.get_cameras()}
        write(output,dict(sides=rows,cameras=cams))
    sapien_backend.SapienBackend.set_review_state=capture
    from gamepad_vla.workstation.operator_view import render
    render(args.output)


def fk(args):
    from gamepad_vla.workstation.sapien_backend import SapienBackend
    from gamepad_vla.workstation.spec import load_spec
    rows=[]
    for rank in range(1,5):
        folder=args.source/'final'/f'rank-{rank}';spec=load_spec(folder/'spec.json')
        c=json.loads((folder/'candidates.json').read_text())
        b=SapienBackend(spec,render=False,work_dir=args.output/'fk-models'/f'rank-{rank}')
        state={}
        for side in ('left','right'):
            state[side+'_arm']=c['sides'][side]['check']['arm_q']
            state[side+'_hand']=c['sides'][side]['hand_q_all_candidate']
        b.set_review_state(state)
        for side,robot in [('left',b.robot),('right',b.right_robot)]:
            pin=robot.create_pinocchio_model();pin.compute_forward_kinematics(robot.get_qpos())
            root=robot.get_root_pose().to_transformation_matrix()
            rows.append(dict(rank=rank,side=side,
                q_by_name={j.name:float(q) for j,q in zip(robot.get_active_joints(),robot.get_qpos())},
                links={l.name:l.get_entity_pose().to_transformation_matrix().tolist() for l in robot.get_links()},
                pinocchio={l.name:(root@pin.get_link_pose(i).to_transformation_matrix()).tolist()
                           for i,l in enumerate(robot.get_links())}))
        b.close()
    write(args.output/'render-fk-all.json',rows)
    print('SAPIEN FK/link cases:',len(rows))


def render(args):
    from PIL import Image
    from gamepad_vla.workstation.spec import load_spec,pose_to_matrix
    from gamepad_vla.workstation.operator_view import load_config
    from xbox_gamepad_physics.motion import load_motion_contract
    from xbox_gamepad_physics.teleop.stream.ipc import Latest,SNAPSHOT_CAPACITY,FRAME_CAPACITY,unpack
    folder=args.source/'final/rank-1';spec=load_spec(folder/'spec.json');c=json.loads((folder/'candidates.json').read_text())
    frame=pose_to_matrix(spec.gamepad.root_pose);rotation=frame[:3,:3].T
    # Original camera plus an opposite side camera (active arm is nearer) and top.
    views=dict(original=([.43,1.70,1.32],[.40,.6252,1.10],[0,0,1],48),
        side=([.35,-.75,1.16],[.35,.6252,1.10],[0,0,1],43),
        top=([.35,.6252,2.75],[.35,.6252,1.10],[1,0,0],43))
    for active in ('left','right','both'):
        for view,(pos,target,up,fovy) in views.items():
            if view=='original' and active!='right':continue
            # Look from the active arm's side to limit the inactive arm's occlusion.
            if view=='side' and active=='left':pos=[.35,2.0,1.16]
            run=args.output/f'{active}_{view}';run.mkdir(parents=True,exist_ok=True)
            cfg=load_config(Path('configs/render/operator_view.yaml'));cfg.update(collection=True,review_camera=True,
                view='operator',width=1280,height=720,fps=2,resources=os.environ['XBOX_GAMEPAD_PHYSICS_RESOURCES'],
                pip=dict(enabled=False),overview=dict(enabled=False),camera=dict(
                    position_gamepad=(rotation@(np.array(pos)-frame[:3,3])).tolist(),
                    target_gamepad=(rotation@(np.array(target)-frame[:3,3])).tolist(),
                    up_gamepad=(rotation@up).tolist(),fovy_degrees=fovy))
            write(run/'config.json',cfg);(run/'spec_path.txt').write_text(str((folder/'spec.json').resolve()))
            snap=Latest(run/'snapshot.mmap',SNAPSHOT_CAPACITY,True);frames=Latest(run/'frame.mmap',FRAME_CAPACITY,True)
            state=dict(arm_q=[],hand_q=[],control_joint_positions={n:0. for n in load_motion_contract('full_controls').joint_by_name},command_id=0,candidate_side=active)
            for side in ('left','right'):
                selected=active in (side,'both')
                state['arm_q'].append(c['sides'][side]['check']['arm_q'] if selected else list(getattr(spec,side+'_arm').home_q))
                state['hand_q'].append(c['sides'][side]['hand_q_all_candidate'] if selected else list(getattr(spec,side+'_hand').home_q))
            snap.write(json.dumps(state).encode())
            with (run/'render.log').open('w') as log:
                proc=subprocess.Popen([sys.executable,__file__,'worker','--source',str(args.source),'--output',str(run)],stdout=log,stderr=subprocess.STDOUT)
                try:
                    end=time.monotonic()+120
                    while time.monotonic()<end:
                        row=frames.read()
                        if row:
                            meta,rgb=unpack(row[1]);Image.fromarray(np.frombuffer(rgb,np.uint8).reshape(720,1280,3)).save(run/'raw.png');break
                        if proc.poll() is not None:raise RuntimeError(f'{run}: renderer exited; see log')
                        time.sleep(.1)
                    else:raise TimeoutError(str(run))
                finally:
                    (run/'stop').touch()
                    try:proc.wait(timeout=10)
                    except subprocess.TimeoutExpired:proc.kill();proc.wait()
                    snap.close();frames.close()
            if proc.returncode:raise RuntimeError(f'{run}: renderer exit {proc.returncode}')
            print(run,flush=True)


def compare(args):
    from PIL import Image,ImageDraw,ImageFont,ImageChops
    native=json.loads((args.output/'native.json').read_text())
    rendered=json.loads((args.output/'render-fk-all.json').read_text())
    checks=[]
    def reconstruct(case,frames,side):
        frames={k:np.array(v) for k,v in frames.items()}
        children=case['joint_children'];axes=np.array(case['joint_local_axes'])
        points=axis_points([frames[k][:3,3] for k in children],
            [frames[k][:3,:3]@a for k,a in zip(children,axes)])
        palm=frames[('right_' if side=='right' else '')+'hand_base_link']
        points['palm_root']=palm[:3,3].tolist()
        points['palm_centroid']=(palm[:3,:3]@case['palm_local']+palm[:3,3]).tolist()
        return points
    for case in native:
        r=next(x for x in rendered if (x['rank'],x['side'])==(case['rank'],case['side']))
        variants=dict(metric=case['metric'],native=case['native'],sapien_fk=reconstruct(case,r['pinocchio'],case['side']),
                      sapien_links=reconstruct(case,r['links'],case['side']))
        children=case['joint_children']
        raw={k:np.array(r['links'][children[i]])[:3,3].tolist() for k,i in [('shoulder',1),('elbow',3),('wrist',4)]}
        raw['palm_root']=variants['sapien_links']['palm_root'];raw['palm_centroid']=variants['sapien_links']['palm_centroid']
        variants['raw_link_origins']=raw
        errors={method:{k:dict(delta_mm=((np.array(p)-case['metric'][k])*1000).tolist(),
            norm_mm=float(np.linalg.norm(np.array(p)-case['metric'][k])*1000)) for k,p in pts.items()}
            for method,pts in variants.items() if method!='metric'}
        actual_q=[r['q_by_name'][name.replace('_link','_joint_')] for name in case['joint_children']]
        q_error=float(np.max(np.abs(np.array(actual_q)-case['q'])))
        checks.append(dict(rank=case['rank'],id=case['id'],side=case['side'],positions_m=variants,
            errors=errors,angles={k:angles(p) for k,p in variants.items()},max_q_error_rad=q_error,
            max_error_mm=max(v['norm_mm'] for e in errors.values() for v in e.values())))
    write(args.output/'comparison.json',checks)
    assert max(c['max_error_mm'] for c in checks)<=1.,'FK disagreement exceeds 1 mm'
    fontpath='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
    font=ImageFont.truetype(fontpath,20);small=ImageFont.truetype(fontpath,17)
    def annotate(name,selected):
        run=args.output/name;audit=json.loads((run/'render-audit.json').read_text())
        camera=audit['cameras']['operator_only'];model=np.array(camera['model']);intr=np.array(camera['intrinsic'])
        im=Image.open(run/'raw.png').convert('RGB');draw=ImageDraw.Draw(im)
        def uv(p):
            v=(np.array(p)-model[:3,3])@model[:3,:3]
            return tuple(int(x) for x in np.round([intr[0,0]*v[0]/-v[2]+intr[0,2],intr[1,2]-intr[1,1]*v[1]/-v[2]]))
        def label(p,text,color=(255,255,255),f=small):
            box=draw.textbbox(p,text,font=f);draw.rectangle((box[0]-3,box[1]-2,box[2]+3,box[3]+2),fill=(15,20,26));draw.text(p,text,fill=color,font=f)
        draw.rectangle((0,0,1280,122),fill=(15,20,26))
        label((20,8),f'{name} | d050-h220-t+10 | SAPIEN actual link poses',f=font)
        label((20,39),'S shoulder = red; E elbow = yellow; W wrist = cyan. Lines show 3D joint centres (x-ray).')
        label((20,67),'GREEN = selected grasp; MAGENTA = unused arm at HOME. Angles below are 3D, not projected.')
        polys={};actual={}
        for side in ('left','right'):
            case=next(x for x in native if x['rank']==1 and x['side']==side)
            pts=reconstruct(case,audit['sides'][side]['links'],side);actual[side]=pts
            is_selected=side in selected;line=(50,255,100) if is_selected else (255,70,210)
            poly=[uv(pts[k]) for k in ('shoulder','elbow','wrist')];polys[side]=poly
            draw.line(poly,fill=line,width=6 if is_selected else 3)
            for point,color,letter in zip(poly,[(255,65,65),(255,220,40),(30,220,255)],['S','E','W']):
                x,y=point;draw.ellipse((x-7,y-7,x+7,y+7),fill=color,outline=(0,0,0),width=2)
                # Separate coincident shoulder labels and home/selected elbows.
                label((x+10,y+(8 if side=='right' else -29)),f'{side[0].upper()}-{letter}',color)
            m=angles(pts);state='selected' if is_selected else 'HOME, unused'
            label((20,625 if side=='left' else 665),
                f'{side.upper()} {state}: upper vs DOWN vertical {m["upper_tilt_deg"]:.2f} deg | forearm vs horizontal {m["forearm_pitch_deg"]:+.2f} deg | elbow {m["elbow_angle_deg"]:.2f} deg',line)
            if is_selected and 'top' not in name:
                s=np.array(pts['shoulder']);e=np.array(pts['elbow']);w=np.array(pts['wrist'])
                draw.line([uv(s),uv(s+[0,0,-.20])],fill=(255,65,65),width=2)
                h=w-e;h[2]=0;h=.20*h/np.linalg.norm(h)
                draw.line([uv(e),uv(e+h)],fill=(255,220,40),width=2)
                label((uv(s+[0,0,-.20])[0]+8,uv(s+[0,0,-.20])[1]),'vertical',color=(255,120,120))
                label((uv(e+h)[0]+8,uv(e+h)[1]+12),'horizontal',color=(255,220,40))
        destination=args.output/'images'/f'{name}_annotated.png';destination.parent.mkdir(exist_ok=True);im.save(destination)
        write(run/'projected-points.json',dict(pixels=polys,world=actual))
        return im
    panels={}
    for name in ('left_side','left_top','right_original','right_side','right_top','both_side','both_top'):
        selected=('left','right') if name.startswith('both') else (name.split('_')[0],)
        panels[name]=annotate(name,selected)
    combined=Image.new('RGB',(2560,1440))
    for name,p in [('left_side',(0,0)),('left_top',(1280,0)),('right_side',(0,720)),('right_top',(1280,720))]:combined.paste(panels[name],p)
    combined.save(args.output/'images/posture_four_views.png')
    old=args.source/'final/rank-1/render/right_side.png'
    diff=np.asarray(ImageChops.difference(Image.open(old).convert('RGB'),Image.open(args.output/'right_original/raw.png').convert('RGB')))
    write(args.output/'image-replay.json',dict(mean_absolute_channel_difference=float(diff.mean()),maximum_difference=int(diff.max()),
        different_pixel_fraction=float(np.any(diff!=0,axis=2).mean())))
    print(json.dumps([dict(rank=c['rank'],side=c['side'],max_error_mm=c['max_error_mm']) for c in checks]))


def main():
    a=argparse.ArgumentParser();a.add_argument('stage',choices=['native','render','worker','fk','compare']);a.add_argument('--source',type=Path,required=True);a.add_argument('--output',type=Path,required=True)
    args=a.parse_args();args.source=args.source.resolve();args.output=args.output.resolve();args.output.mkdir(parents=True,exist_ok=True)
    globals()[args.stage](args)


if __name__=='__main__':main()
