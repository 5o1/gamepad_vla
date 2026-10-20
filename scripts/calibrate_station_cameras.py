#!/usr/bin/env python3
"""Station camera evidence preparation and joint focal/PnP fit (offline only)."""
import argparse
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np

from gamepad_vla.workstation.g20_mapping import motor_positions_to_joint_positions
from gamepad_vla.workstation.spec import matrix_to_pose
from gamepad_vla.workstation.spec import load_spec, pose_to_matrix
from gamepad_vla.workstation.g20_mapping import G20_JOINT_NAMES
from gamepad_vla.kinematics import SerialChain


def landmarks(spec_path, selected_path, urdf_dir, output):
    """FK landmarks in world; each retains its local geometry definition."""
    import xml.etree.ElementTree as ET
    import trimesh
    spec = load_spec(spec_path)
    selected = json.loads(selected_path.read_text())
    points = {}
    for side in ("left", "right"):
        arm = getattr(spec, side + "_arm")
        hand = getattr(spec, side + "_hand")
        prefix = "right_" if side == "right" else ""
        file = urdf_dir / ("workcell_right_robot.urdf" if side == "right" else "workcell_robot.urdf")
        q = dict(zip(arm.joint_names, selected[side + "_arm"]))
        q.update(dict(zip((prefix + n for n in G20_JOINT_NAMES), selected[side + "_hand"])))
        root = ET.parse(file).getroot()
        poses = {}
        def point(link, local, name):
            if link not in poses:
                chain = SerialChain(file, link)
                poses[link] = pose_to_matrix(arm.base_pose) @ chain.forward([[q[n] for n in chain.active]])[0]
            p = poses[link] @ np.r_[local, 1]
            points[side + ":" + name] = dict(xyz=p[:3].tolist(), link=link, local=list(local),
                                               link_pose_world=matrix_to_pose(poses[link]),
                                               source=f"{selected['episode']} frame{selected['frame_index']}; measured arm radians + station motor-to-joint conversion + URDF geometry")
        for link in root.findall("link"):
            name = link.get("name")
            if name == arm.base_link:
                continue
            point(name, [0,0,0], name + ":origin")
            if name.endswith("distal"):
                mesh=trimesh.load(link.find("visual/geometry/mesh").get("filename"),process=False)
                vertices=np.asarray(mesh.vertices)
                tip=vertices[vertices[:,2]>vertices[:,2].max()-.0005].mean(axis=0)
                point(name,tip,name+":tip")
            if name == prefix + "hand_base_link":
                mesh = trimesh.load(link.find("visual/geometry/mesh").get("filename"), process=False)
                lower,upper = mesh.bounds
                points[side + ":palm_bounds_local"] = [lower.tolist(),upper.tolist()]
                point(name,[0,0,upper[2]],name+":distal_center")
                for tag,y in (("minus",lower[1]),("plus",upper[1])):
                    point(name,[0,y,upper[2]],name+":distal_"+tag)
                # Box corners are labeled as geometric proxies, not guaranteed
                # identifiable surface vertices in a monocular image.
                for y in (lower[1],upper[1]):
                    for z in (lower[2],upper[2]):
                        point(name,[lower[0],y,z],f"palm_edge_y{y:.4f}_z{z:.4f}")
    output.write_text(json.dumps(points,indent=2))


def multiframe_prepare(spec_path, snapshot, urdf_dir, output):
    """Freeze a 12-frame, two-episode fit/holdout set before any fitting."""
    from PIL import Image, ImageDraw
    output.mkdir(parents=True,exist_ok=True)
    episodes=[("e1","workstation_20260831_201954-0","episode"),
              ("e2","workstation_20260901_144603-1","episode2")]
    frames=[]
    for label,episode,export in episodes:
        arrays=np.load(output/export/"states.npz")
        state=arrays["observation.state"]
        # Greedy farthest points in joint-state space; motor coordinates are
        # normalized by 255. Include both endpoints and avoid adjacent frames.
        normalized=state.copy(); normalized[:,14:]/=255.
        chosen=[0,len(state)-1]
        for _ in range(4):
            distance=np.min(np.linalg.norm(normalized[:,None]-normalized[chosen][None],axis=2),axis=1)
            for idx in chosen:distance[max(0,idx-15):idx+16]=-1
            chosen.append(int(np.argmax(distance)))
        for rank,idx in enumerate(sorted(chosen)):
            name=f"{label}_f{idx:06d}"
            directory=output/"frames"/name;directory.mkdir(parents=True,exist_ok=True)
            row=state[idx]
            selected=dict(episode=episode,row=idx,frame_index=int(arrays["frame_index"][idx]),
                          timestamp=float(arrays["timestamp"][idx]),state=row.tolist(),
                          left_arm=row[:7].tolist(),right_arm=row[7:14].tolist(),
                          left_hand=motor_positions_to_joint_positions("left",row[14:34]),
                          right_hand=motor_positions_to_joint_positions("right",row[34:54]))
            selected_path=directory/"snapshot.json"
            selected_path.write_text(json.dumps(selected,indent=2))
            landmarks(spec_path,selected_path,urdf_dir,directory/"landmarks.json")
            cameras={}
            for i in (1,2,3):
                video=snapshot/f"workstation/data/lerobot/{episode}/videos/observation.images.camera_{i}/chunk-000/file-000.mp4"
                cap=cv2.VideoCapture(str(video));cap.set(cv2.CAP_PROP_POS_FRAMES,int(selected["frame_index"]))
                ok,image=cap.read();cap.release()
                if not ok:raise RuntimeError(f"cannot read {video} frame {idx}")
                filename=directory/f"camera_{i}.png";cv2.imwrite(str(filename),image)
                cameras[f"camera_{i}"]=str(filename)
            frames.append(dict(name=name,episode=episode,frame_index=selected["frame_index"],
                               timestamp=selected["timestamp"],split="validation" if rank in (2,5) else "fit",
                               snapshot=str(selected_path),landmarks=str(directory/"landmarks.json"),images=cameras))
    manifest=dict(frames=frames,selection="greedy farthest 54-state coordinates, hand/255; endpoints; >=15 frame separation; rank2/5 per episode held out before annotation")
    (output/"frame_selection.json").write_text(json.dumps(manifest,indent=2))
    for i in (1,2,3):
        sheet=Image.new("RGB",(1280,4*300),"white");draw=ImageDraw.Draw(sheet)
        for j,f in enumerate(frames):
            picture=Image.open(f["images"][f"camera_{i}"]).resize((320,240))
            x,y=j%4*320,j//4*300
            sheet.paste(picture,(x,y+28));draw.text((x+4,y+4),f["name"]+" "+f["split"],fill="black")
        sheet.save(output/f"camera_{i}_selected.jpg")


def write_provenance(snapshot):
    records = []
    for path in sorted(snapshot.rglob("*")):
        if not path.is_file() or path.name == "PROVENANCE.md":
            continue
        stat = path.stat()
        records.append(dict(source="station-home://" + str(path.relative_to(snapshot)),
                            copied_path=str(path), bytes=stat.st_size,
                            sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                            copy_time_utc=datetime.fromtimestamp(stat.st_ctime, timezone.utc).isoformat()))
    (snapshot / "PROVENANCE.md").write_text(
        "# 只读采集台快照\n\n通过 linkerhand.sh -n 的 tar cf - stdout，经本机管道直接写到节点。"
        "未在采集台写文件。以下为复制后字节的 SHA256；没有再次读取源文件复核。"
        "copy_time 为目标文件 ctime（复制完成时刻的近似），原始采集时间未知。\n\n```json\n" +
        json.dumps(records, indent=2, ensure_ascii=False) + "\n```\n")


def prepare(snapshot, output):
    from PIL import Image, ImageDraw
    output.mkdir(parents=True, exist_ok=True)
    write_provenance(snapshot)
    state_file = output / "episode/states.npz"
    arrays = np.load(state_file)
    state = arrays["observation.state"]
    frame = 0
    row = state[frame]
    selected = dict(episode="workstation_20260831_201954-0", row=frame,
                    frame_index=int(arrays["frame_index"][frame]),
                    timestamp=float(arrays["timestamp"][frame]),
                    state=row.tolist(), left_arm=row[:7].tolist(), right_arm=row[7:14].tolist(),
                    left_hand=motor_positions_to_joint_positions("left", row[14:34]),
                    right_hand=motor_positions_to_joint_positions("right", row[34:54]),
                    hand_conversion="existing station motor_positions_to_joint_positions; not measured URDF angles")
    (output / "selected_frame.json").write_text(json.dumps(selected, indent=2))
    items = []
    for path in sorted(snapshot.rglob("*")):
        if path.suffix.lower() in {".png", ".jpg"}:
            items.append((str(path.relative_to(snapshot)), Image.open(path).convert("RGB")))
    for path in sorted(snapshot.rglob("*.mp4")):
        camera = next(p for p in path.parts if p.startswith("observation.images." )).split(".")[-1]
        cap = cv2.VideoCapture(str(path))
        frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        gallery = Image.new("RGB", (960, 410 * 2), "white")
        gd = ImageDraw.Draw(gallery)
        for panel, idx in enumerate(np.linspace(0,frame_count-1,6).astype(int)):
            cap.set(cv2.CAP_PROP_POS_FRAMES,int(idx))
            valid, sample=cap.read()
            if valid:
                thumb=Image.fromarray(cv2.cvtColor(sample,cv2.COLOR_BGR2RGB)).resize((320,240))
                px,py=panel%3*320,panel//3*410
                gallery.paste(thumb,(px,py+20))
                gd.text((px+5,py+3),f"{camera} frame {idx}",fill="black")
        gallery.save(output/f"{camera}_timeline.jpg")
        cap.set(cv2.CAP_PROP_POS_FRAMES,0)
        ok, image = cap.read()
        cap.release()
        if not ok:
            raise RuntimeError(f"cannot decode {path}")
        dst = output / f"{camera}_frame000000.png"
        cv2.imwrite(str(dst), image)
        items.append((str(dst.name), Image.open(dst).convert("RGB")))
    sheet = Image.new("RGB", (1200, 300 * ((len(items) + 2) // 3)), "#eeeeee")
    draw = ImageDraw.Draw(sheet)
    inventory = []
    for i, (label, picture) in enumerate(items):
        inventory.append(dict(file=label, width=picture.width, height=picture.height))
        picture.thumbnail((400, 260))
        x, y = (i % 3) * 400, (i // 3) * 300
        sheet.paste(picture, (x, y + 40))
        draw.text((x + 3, y), label[-65:], fill="black")
    sheet.save(output / "image_inventory.jpg")
    (output / "image_inventory.json").write_text(json.dumps(inventory, indent=2))


def fit(annotation, output):
    """Multi-frame robust PnP + shared f/cx/cy, without using held-out frames."""
    from scipy.optimize import least_squares
    from PIL import Image, ImageDraw
    data=json.loads(annotation.read_text())
    selection=json.loads(Path(data['selection']).read_text())
    frames={f['name']:f for f in selection['frames']}
    width,height=data['width'],data['height']
    points=[]
    for item in data['points']:
        frame=frames[item['frame']]
        lm=json.loads(Path(frame['landmarks']).read_text())
        original=lm[item['key']]
        xyz=np.asarray(original['xyz'])
        if data['parent']!='world':
            side=data['parent'].split('_')[0]
            root_key=side+':'+('right_' if side=='right' else '')+'hand_base_link:origin'
            hand_pose=pose_to_matrix(lm[root_key]['link_pose_world'])
            xyz=(np.linalg.inv(hand_pose)@np.r_[xyz,1])[:3]
        points.append(dict(**item,xyz=xyz.tolist(),split=frame['split'],image=frame['images'][data['name']]))
    train=[p for p in points if p['split']=='fit']
    xyz=np.asarray([p['xyz'] for p in train],float)
    uv=np.asarray([p['uv'] for p in train],float)
    if len(train)<6:raise ValueError('need at least six fitted correspondences')
    def intrinsic(x):
        return np.array([[x[6],0,x[7]],[0,x[6],x[8]],[0,0,1.]])
    f0=width/(2*np.tan(np.deg2rad(69/2)))
    cv2.setRNGSeed(20261015)
    seeds=[]
    ransac=[]
    for factor in (.6,.8,1.,1.3,1.6):
        focal=f0*factor
        k=np.array([[focal,0,width/2],[0,focal,height/2],[0,0,1.]])
        for threshold in (8.,30.):
            ok,r,t,inliers=cv2.solvePnPRansac(xyz,uv,k,None,iterationsCount=2000,
                       reprojectionError=threshold,confidence=.999,flags=cv2.SOLVEPNP_EPNP)
            ransac.append(dict(focal=focal,threshold_px=threshold,inliers=0 if inliers is None else len(inliers)))
            if ok:
                depths=(cv2.Rodrigues(r)[0]@xyz.T+t).T[:,2]
                if np.mean(depths>0)>.9:
                    seeds.append(np.r_[r.ravel(),t.ravel(),focal,width/2,height/2])
    if not seeds:
        # RANSAC failing is reported; SQPnP provides a coarse initialization,
        # not a claim that the consensus threshold was met.
        k=np.array([[f0,0,width/2],[0,f0,height/2],[0,0,1.]])
        ok,r,t=cv2.solvePnP(xyz,uv,k,None,flags=cv2.SOLVEPNP_SQPNP)
        if not ok:raise RuntimeError('no PnP initialization')
        seeds=[np.r_[r.ravel(),t.ravel(),f0,width/2,height/2]]
    def residual(x):
        projected,_=cv2.projectPoints(xyz,x[:3],x[3:6],intrinsic(x),None)
        fov=np.rad2deg(2*np.arctan(width/(2*x[6])))
        # 640x480's sensor crop is unknown: the nominal 69-degree prior is
        # deliberately 10 degrees wide, rather than transferring 1080p K.
        priors=[3*(fov-69)/10,3*(x[7]-width/2)/(width*.05),3*(x[8]-height/2)/(height*.05)]
        return np.r_[(projected.reshape(-1,2)-uv).ravel(),priors]
    bounds=([-np.inf]*6+[width/4,-width*.25,-height*.25],
            [np.inf]*6+[width*4,width*1.25,height*1.25])
    solutions=[least_squares(residual,x,bounds=bounds,loss='soft_l1',f_scale=3,max_nfev=2500) for x in seeds]
    solution=min(solutions,key=lambda s:s.cost)
    x=solution.x
    rotation=cv2.Rodrigues(x[:3])[0]
    parent_camera=np.eye(4);parent_camera[:3,:3]=rotation.T;parent_camera[:3,3]=-rotation.T@x[3:6]
    all_xyz=np.asarray([p['xyz'] for p in points],float)
    projection,_=cv2.projectPoints(all_xyz,x[:3],x[3:6],intrinsic(x),None)
    projection=projection.reshape(-1,2)
    errors=np.linalg.norm(projection-np.asarray([p['uv'] for p in points]),axis=1)
    output.mkdir(parents=True,exist_ok=True)
    per_frame={}
    for j,(p,projected,error) in enumerate(zip(points,projection,errors)):
        p.update(projected=projected.tolist(),error_px=float(error))
        frame_dir=output/p['frame'];frame_dir.mkdir(parents=True,exist_ok=True)
        image=Image.open(p['image']).convert('RGB')
        u,v=p['uv']
        crop=image.crop((u-32,v-32,u+32,v+32)).resize((384,384))
        draw=ImageDraw.Draw(crop);draw.line((180,192,204,192),fill='red',width=2);draw.line((192,180,192,204),fill='red',width=2)
        filename=frame_dir/f'point_{j:03d}.png';crop.save(filename)
        p['evidence']=str(filename)
        per_frame.setdefault(p['frame'],[]).append(p)
    def summarize(items):
        e=[p['error_px'] for p in items]
        return dict(count=len(e),median_px=float(np.median(e)) if e else None,p95_px=float(np.percentile(e,95)) if e else None)
    summaries={split:summarize([p for p in points if p['split']==split]) for split in ('fit','validation')}
    frame_metrics={}
    for name,items in per_frame.items():
        picture=Image.open(items[0]['image']).convert('RGB');draw=ImageDraw.Draw(picture)
        for p in items:
            u,v=p['uv'];color='lime' if p['split']=='fit' else 'yellow'
            draw.ellipse((u-3,v-3,u+3,v+3),outline=color,width=2)
            draw.line((u,v,*p['projected']),fill='red',width=2)
            draw.text((u+4,v+4),p['key'].split(':')[-2],fill=color)
        picture.save(output/name/'annotations.png')
        frame_metrics[name]=summarize(items)
    validation=summaries['validation']
    passed=validation['count']>0 and validation['median_px']<=3 and validation['p95_px']<=8
    result=dict(name=data['name'],serial=data['serial'],model='D435 (recorded, physical model unverified)',
        parent=data['parent'],pose_optical=matrix_to_pose(parent_camera),
        intrinsics=dict(width=width,height=height,fx=float(x[6]),fy=float(x[6]),cx=float(x[7]),cy=float(x[8]),distortion=[],model='none'),
        status='estimated',source=str(annotation),metrics=summaries,per_frame=frame_metrics,
        ransac=ransac,points=points,validation_passed=bool(passed),
        hfov_deg=float(np.rad2deg(2*np.arctan(width/(2*x[6])))),
        prior=dict(hfov_degrees=69,hfov_sigma_degrees=10,principal_sigma_fraction=.05,
                   note='640x480 crop unknown; profile-independent nominal prior; not 1080p intrinsics'),
        quality='estimated, holdout threshold met' if passed else '粗配准',
        limits='D unknown, zero prior. Frame and joint clocks not independently verified. Raw motor-to-joint map and mimic are model assumptions.')
    (output/'calibration.json').write_text(json.dumps(result,indent=2,ensure_ascii=False))
    print(json.dumps({k:result[k] for k in ('name','metrics','hfov_deg','quality')}))


def evidence_sheet(calibration):
    from PIL import Image,ImageDraw
    data=json.loads(calibration.read_text())
    points=data['points']
    sheet=Image.new('RGB',(960,120*((len(points)+9)//10)),'white')
    draw=ImageDraw.Draw(sheet)
    for j,p in enumerate(points):
        image=Image.open(p['evidence']).convert('RGB').resize((96,96))
        x,y=j%10*96,j//10*120
        sheet.paste(image,(x,y+24))
        draw.text((x+2,y+2),str(j)+' '+p['frame'][-6:],fill='black')
        draw.text((x+2,y+12),p['key'].split(':')[-1],fill='black')
    sheet.save(calibration.parent/'evidence_sheet.jpg')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    p = sub.add_parser("prepare")
    p.add_argument("--snapshot", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p = sub.add_parser("fit")
    p.add_argument("annotation", type=Path)
    p.add_argument("--output", type=Path, required=True)
    p = sub.add_parser("landmarks")
    p.add_argument("--spec", type=Path, required=True)
    p.add_argument("--selected", type=Path, required=True)
    p.add_argument("--urdf-dir", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p = sub.add_parser("multiframe-prepare")
    p.add_argument("--spec",type=Path,required=True)
    p.add_argument("--snapshot",type=Path,required=True)
    p.add_argument("--urdf-dir",type=Path,required=True)
    p.add_argument("--output",type=Path,required=True)
    p = sub.add_parser("provenance")
    p.add_argument("--snapshot",type=Path,required=True)
    p = sub.add_parser("evidence")
    p.add_argument("calibration",type=Path)
    args = parser.parse_args()
    if args.command == "prepare":
        prepare(args.snapshot, args.output)
    elif args.command == "fit":
        fit(args.annotation, args.output)
    elif args.command == "landmarks":
        landmarks(args.spec,args.selected,args.urdf_dir,args.output)
    elif args.command == "multiframe-prepare":
        multiframe_prepare(args.spec,args.snapshot,args.urdf_dir,args.output)
    elif args.command == "provenance":
        write_provenance(args.snapshot)
    else:
        evidence_sheet(args.calibration)


if __name__ == "__main__":
    main()
