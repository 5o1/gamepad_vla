"""Static, sampled feasibility of a fixed-palm two-hand gamepad grasp.

This module does not step dynamics, actuate hardware, train, or change task files.
All gamepad state writes construct geometric test fixtures, not control results.
"""
from __future__ import annotations

import json
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
import trimesh

from gamepad_vla.kinematics import SerialChain
from gamepad_vla.tasks.fixed_gamepad import project_inertia_tensor
from gamepad_vla.workstation.spec import pose_to_matrix, matrix_to_pose

FINGERS = ('thumb', 'index', 'middle', 'ring', 'pinky')
CONE = np.cos(np.deg2rad(30.0))
POINT_TOL = 0.00025
GRIP_PENETRATION = 0.001


def transform(points, frame):
    return np.asarray(points) @ frame[:3, :3].T + frame[:3, 3]


def mesh_for_geom(model, data, geom):
    """Original triangles in the world frame, including compiler mesh offsets."""
    mid = model.geom_dataid[geom]
    va, vn = model.mesh_vertadr[mid], model.mesh_vertnum[mid]
    fa, fn = model.mesh_faceadr[mid], model.mesh_facenum[mid]
    verts = model.mesh_vert[va:va+vn]
    verts = verts @ data.geom_xmat[geom].reshape(3, 3).T + data.geom_xpos[geom]
    return trimesh.Trimesh(verts, model.mesh_face[fa:fa+fn], process=False)


def clean_urdf(source, destination, *, arm=None):
    root = ET.parse(source).getroot()
    if arm is not None:
        by_child = {j.find('child').get('link'): j for j in root.findall('joint')}
        joints, link = [], arm.tip_link
        while link != arm.base_link:
            j = by_child[link]
            joints.append(j)
            link = j.find('parent').get('link')
        keep = {arm.base_link} | {j.find('child').get('link') for j in joints}
        for el in list(root):
            if (el.tag == 'joint' and el not in joints) or (el.tag == 'link' and el.get('name') not in keep):
                root.remove(el)
    for mesh in root.iter('mesh'):
        mesh.set('filename', str((Path(source).parent / mesh.get('filename')).resolve()))
    corrections = []
    for link in root.findall('link'):
        el = link.find('inertial/inertia')
        if el is None:
            continue
        a = {k: float(v) for k, v in el.attrib.items()}
        t = np.array([[a['ixx'], a['ixy'], a['ixz']], [a['ixy'], a['iyy'], a['iyz']], [a['ixz'], a['iyz'], a['izz']]])
        p = project_inertia_tensor(t)
        if not np.array_equal(t, p):
            corrections.append(dict(link=link.get('name'), before=t.tolist(), after=p.tolist()))
            for key, i, j in [('ixx',0,0),('iyy',1,1),('izz',2,2),('ixy',0,1),('ixz',0,2),('iyz',1,2)]:
                el.set(key, str(p[i,j]))
    ext = ET.SubElement(root, 'mujoco')
    ET.SubElement(ext, 'compiler', discardvisual='true', fusestatic='false', strippath='false')
    Path(destination).write_text(ET.tostring(root, encoding='unicode'))
    return corrections


class Finger:
    def __init__(self, urdf, name, samples, rng):
        self.name = name
        self.chain = SerialChain(urdf, name + '_distal')
        self.lower, self.upper = self.chain.active_limits
        # Mimic joint limits constrain the source as well.
        for j in self.chain.joints:
            if j['mimic'] and j['limits']:
                source, mult, offset = j['mimic']
                k = self.chain.active.index(source)
                lo, hi = sorted((np.asarray(j['limits']) - offset) / mult)
                self.lower[k] = max(self.lower[k], lo)
                self.upper[k] = min(self.upper[k], hi)
        root = ET.parse(urdf).getroot()
        el = root.find(f"link[@name='{name}_distal']/collision")
        mesh = el.find('geometry/mesh')
        path = Path(urdf).parent / mesh.get('filename')
        vertices = trimesh.load(path, force='mesh').vertices.copy()
        vertices *= np.array([float(x) for x in mesh.get('scale', '1 1 1').split()])
        origin = el.find('origin')
        if origin is not None:
            from gamepad_vla.kinematics import rpy_to_matrix
            vertices = vertices @ rpy_to_matrix([float(x) for x in origin.get('rpy','0 0 0').split()]).T
            vertices += [float(x) for x in origin.get('xyz','0 0 0').split()]
        q = (self.lower+self.upper)/2
        dq = q.copy()
        flex = 'thumb_mcp' if name == 'thumb' else name+'_mcp_pitch'
        dq[self.chain.active.index(flex)] += 1e-4
        frames = self.chain.forward([q, dq])
        tip = np.array([0, 0, vertices[:,2].max()])
        direction = frames[0,:3,:3].T @ (transform(tip,frames[1])-transform(tip,frames[0]))
        # Thumb may use tip or palmar surface; grip fingers always use finger pad.
        angles = [0,45,90] if name in ('thumb','index','middle') else [90]
        self.points, self.normals = [], []
        for angle in angles:
            n = np.array([np.sign(direction[0])*np.sin(np.deg2rad(angle)), 0, np.cos(np.deg2rad(angle))])
            depth = vertices @ n
            self.points.append(vertices[depth >= depth.max()-0.00015].mean(axis=0))
            self.normals.append(n)
        self.points, self.normals = np.asarray(self.points), np.asarray(self.normals)
        self.q = rng.uniform(self.lower, self.upper, (samples, len(q)))
        f = self.chain.forward(self.q)
        self.xyz = np.concatenate([f[:,:3,:3] @ p + f[:,:3,3] for p in self.points])
        self.nrm = np.concatenate([f[:,:3,:3] @ n for n in self.normals])
        self.tree = cKDTree(np.c_[self.xyz, .015*self.nrm])
        self.position_tree = cKDTree(self.xyz)

    def fit(self, targets, normals, *, iterations=35, normal_cone=True):
        """Batched constrained point/normal IK, seeded by sampled FK clouds."""
        targets, normals = np.asarray(targets), np.asarray(normals)
        _, nearest = (self.tree.query(np.c_[targets, .015*normals], workers=1) if normal_cone
                      else self.position_tree.query(targets, workers=1))
        mode = nearest // len(self.q)
        q = self.q[nearest % len(self.q)].copy()
        p, n = self.points[mode], self.normals[mode]
        def evaluate(x):
            f = self.chain.forward(x)
            xyz = np.einsum('nij,nj->ni',f[:,:3,:3],p)+f[:,:3,3]
            nrm = np.einsum('nij,nj->ni',f[:,:3,:3],n)
            # A cone is an inequality, not an orientation equality.
            cone = np.maximum(0, CONE - np.sum(nrm*normals,axis=1)) if normal_cone else np.zeros(len(x))
            return np.c_[xyz-targets,.015*cone], xyz, nrm
        for _ in range(iterations):
            e, _, _ = evaluate(q)
            jac = np.empty((len(q),4,q.shape[1]))
            for k in range(q.shape[1]):
                delta = q.copy(); delta[:,k] += 1e-5
                jac[:,:,k] = (evaluate(delta)[0]-e)/1e-5
            jt = jac.transpose(0,2,1)
            dq = np.linalg.solve(jt@jac + 1e-7*np.eye(q.shape[1]), -(jt@e[...,None]))[...,0]
            q = np.clip(q+np.clip(dq,-.12,.12),self.lower,self.upper)
        _, xyz, nrm = evaluate(q)
        error = np.linalg.norm(xyz-targets,axis=1)
        dot = np.sum(nrm*normals,axis=1)
        ok = (error <= POINT_TOL) & ((dot >= CONE) if normal_cone else np.ones(len(q), bool))
        return q, mode, error, dot, ok

    def nearest_seed(self, targets, normals, *, normal_cone=True):
        dist, idx = (self.tree.query(np.c_[targets,.015*normals],workers=1) if normal_cone
                     else self.position_tree.query(targets,workers=1))
        return dist, idx


class Pad:
    def __init__(self, resource_root):
        import mujoco as mj
        from xbox_gamepad_physics.mujoco import make_spec
        self.mj = mj
        self.asset = make_spec(resource_root=resource_root,attach_visual=False)
        self.model = self.asset.spec.compile()
        self.data = mj.MjData(self.model)
        mj.mj_forward(self.model,self.data)
        resource_root=Path(resource_root)
        if (resource_root/'manifest.json').exists():
            resource_root=resource_root.parents[1]
        self.motion=json.loads((resource_root/self.asset.manifest['motion_contract']).read_text())
        self.joints={j['name']:j for j in self.motion['joints']}

    def state(self, values):
        self.data.qpos[:]=0
        for name,value in values.items():
            self.data.qpos[self.model.joint(name).qposadr[0]]=value
        self.mj.mj_forward(self.model,self.data)

    def mesh(self, part):
        geoms=[i for i in range(self.model.ngeom) if self.model.geom(i).name.startswith('sdf_'+part+'_')]
        return trimesh.util.concatenate([mesh_for_geom(self.model,self.data,g) for g in geoms])

    def grips(self, side, samples, seed):
        self.state({})
        shell=self.mesh('shell')
        points, faces=trimesh.sample.sample_surface(shell,samples,seed=seed)
        normals=shell.face_normals[faces]
        sign=-1 if side=='left' else 1
        mask=(sign*points[:,0]>.025)&(points[:,1]<.005)&(points[:,2]<-.005)&(normals[:,2]<-.3)
        return points[mask],normals[mask]

    def top(self, part, outward):
        mesh=self.mesh(part)
        n=np.asarray(outward); n/=np.linalg.norm(n)
        depth=mesh.vertices@n
        keep=mesh.vertices[depth>depth.max()-.0003]
        return keep.mean(axis=0),n

    def targets(self, side):
        """Contact targets at actual displaced surfaces, in controller coordinates."""
        groups=[]
        def add(name,finger,part,states,outward=None,points=None):
            poses=[]
            for i,values in enumerate(states):
                self.state(values)
                if points is not None:
                    body=self.model.body(part).id
                    f=np.eye(4); f[:3,:3]=self.data.xmat[body].reshape(3,3);f[:3,3]=self.data.xpos[body]
                    p=transform(points[i],f); n=f[:3,2]
                else:
                    p,n=self.top(part,np.asarray(outward,dtype=float))
                poses.append(dict(point=p.tolist(),normal=n.tolist(),gamepad_q=values))
            groups.append(dict(name=name,finger=finger,part=part,states=poses))
        if side=='left':
            directions=[('up','dpad_vertical',-1,[0,.007,0]),('down','dpad_vertical',1,[0,-.007,0]),
                        ('left','dpad_horizontal',-1,[-.007,0,0]),('right','dpad_horizontal',1,[.007,0,0])]
            # Local dpad cap top = pivot + 7.19 mm.
            for label,joint,sign,offset in directions:
                add('Dpad_'+label,'thumb','dpad',[{joint:sign*self.joints[joint]['upper']*.31}],
                    points=[np.array(offset)+[0,0,.007190558736329]])
            digit=[('View','view'),('Xbox','guide')]
        else:
            digit=[('A','a'),('B','b'),('X','x'),('Y','y'),('Menu','menu')]
        for label,part in digit:
            joint=part+'_press'
            add(label,'thumb',part,[{joint:self.joints[joint]['upper']*.31}],outward=[0,0,1])
        stick=side+'_stick'; limit=self.joints[stick+'_x']['upper']
        states=[]
        for theta in np.arange(8)*np.pi/4:
            states.append({stick+'_x':.8*limit*np.cos(theta),stick+'_y':.8*limit*np.sin(theta)})
        add('LS' if side=='left' else 'RS','thumb',stick,states,outward=[0,0,1])
        add('L3' if side=='left' else 'R3','thumb',stick,[{stick+'_click':.31*self.joints[stick+'_click']['upper']}],outward=[0,0,1])
        shoulder=side+'_shoulder'; joint=shoulder+'_press'
        add('LB' if side=='left' else 'RB','index',shoulder,[{joint:.31*self.joints[joint]['upper']}],outward=-np.array(self.joints[joint]['axis']))
        trigger=side+'_trigger'; joint=trigger+'_hinge'
        # Select an exposed surface whose inward normal produces positive hinge torque.
        self.state({joint:.8*self.joints[joint]['upper']})
        mesh=self.mesh(trigger)
        p,face=trimesh.sample.sample_surface(mesh,3000,seed=27)
        n=mesh.face_normals[face]
        axis=np.array(self.joints[joint]['axis']);pivot=np.array(self.joints[joint]['origin'])
        lever=np.sum(np.cross(p-pivot,-n)*axis,axis=1)
        exposed=(n[:,1]>.3)&(p[:,1]>.035)&(lever>.003)
        choices=np.where(exposed)[0]
        if not len(choices):
            raise ValueError('no trigger force-bearing contact patch')
        ids=choices[np.argsort(lever[choices])[-3:]]
        poses=[dict(point=p[i].tolist(),normal=n[i].tolist(),gamepad_q={joint:.8*self.joints[joint]['upper']}) for i in ids]
        # These are alternatives for the same 80% endpoint, not three required endpoints.
        groups.append(dict(name='LT' if side=='left' else 'RT',finger='middle',part=trigger,states=poses,alternatives=True))
        self.state({})
        return groups


def palm_candidates(finger, grip_points, grip_normals, count, rng):
    """Anchor the ring pad to a sampled rear-shell point; sample its posture/roll."""
    qi=rng.integers(len(finger.q),size=count)
    si=rng.integers(len(grip_points),size=count)
    frames=finger.chain.forward(finger.q[qi])
    p=np.einsum('nij,j->ni',frames[:,:3,:3],finger.points[0])+frames[:,:3,3]
    n=frames[:,:3,:3]@finger.normals[0]
    target=-grip_normals[si]
    # Map an oriented normal frame to another; random spin spans the remaining DOF.
    def bases(v):
        helper=np.tile([1.,0,0],(len(v),1));helper[np.abs(v[:,0])>.9]=[0,1,0]
        u=np.cross(v,helper);u/=np.linalg.norm(u,axis=1)[:,None]
        return np.stack([u,np.cross(v,u),v],axis=2)
    spin=Rotation.from_rotvec(np.c_[np.zeros((count,2)),rng.uniform(-np.pi,np.pi,count)]).as_matrix()
    r=bases(target)@spin@bases(n).transpose(0,2,1)
    out=np.tile(np.eye(4),(count,1,1));out[:,:3,:3]=r
    out[:,:3,3]=grip_points[si]-np.einsum('nij,nj->ni',r,p)
    return out, finger.q[qi], si


def local_targets(palms, points, normals):
    # Returns (palms, targets, xyz) in the palm frame.
    r=palms[:,:3,:3]
    p=np.einsum('nji,ntj->nti',r,np.asarray(points)[None]-palms[:,None,:3,3])
    n=np.einsum('nji,tj->nti',r,np.asarray(normals))
    return p,n


def arm_ik(arm, hand, palms_world, rng, seeds=6, iterations=100):
    chain=SerialChain(arm.urdf,arm.tip_link)
    zero_links=chain.link_poses(np.zeros((1,7)))
    zero=np.eye(4) if chain.root_link==arm.base_link else zero_links[arm.base_link][0]
    design_from_world=zero@np.linalg.inv(pose_to_matrix(arm.base_pose))
    targets=design_from_world@palms_world@np.linalg.inv(pose_to_matrix(hand.mount))
    lo,hi=chain.active_limits
    children=[j['child'] for j in chain.joints if j['name'] in chain.active]
    axes=[j['axis'] for j in chain.joints if j['name'] in chain.active]
    best=np.full(len(targets),np.inf);result=np.zeros((len(targets),7))
    def rot_error(r):
        return Rotation.from_matrix(r).as_rotvec()
    for seed in range(seeds):
        q=np.tile(arm.home_q,(len(targets),1)) if seed==0 else rng.uniform(lo,hi,(len(targets),7))
        for _ in range(iterations):
            poses=chain.link_poses(q);tip=poses[arm.tip_link]
            e=np.c_[targets[:,:3,3]-tip[:,:3,3],rot_error(targets[:,:3,:3]@tip[:,:3,:3].transpose(0,2,1))]
            jac=np.empty((len(q),6,7))
            for k,(child,axis) in enumerate(zip(children,axes)):
                v=poses[child][:,:3,:3]@axis
                jac[:,:3,k]=np.cross(v,tip[:,:3,3]-poses[child][:,:3,3]);jac[:,3:,k]=v
            jt=jac.transpose(0,2,1)
            step=jt@np.linalg.solve(jac@jt+1e-4*np.eye(6),e[...,None])
            q=np.clip(q+np.clip(step[...,0],-.25,.25),lo,hi)
        f=chain.forward(q)
        pe=np.linalg.norm(targets[:,:3,3]-f[:,:3,3],axis=1)
        re=np.linalg.norm(rot_error(targets[:,:3,:3]@f[:,:3,:3].transpose(0,2,1)),axis=1)
        cost=pe+.05*re; better=cost<best
        best[better]=cost[better];result[better]=q[better]
    f=chain.forward(result)
    pe=np.linalg.norm(targets[:,:3,3]-f[:,:3,3],axis=1)
    re=np.linalg.norm(rot_error(targets[:,:3,:3]@f[:,:3,:3].transpose(0,2,1)),axis=1)
    return result,(pe<.0005)&(re<.005),pe,re


class CollisionScene:
    """Separate static scene; includes both hands, both arms and all box fixtures."""
    def __init__(self,spec,pad,work,*,moving_fixtures=(),force_static_hand_pairs=False):
        import mujoco as mj
        from xbox_gamepad_physics.mujoco import attach_to_spec
        self.mj=mj;self.spec=spec
        parent=mj.MjSpec()
        self.corrections={}
        for side,letter in [('left','L'),('right','R')]:
            arm,hand=getattr(spec,side+'_arm'),getattr(spec,side+'_hand')
            for kind,source in [('A',arm.urdf),('H',hand.urdf)]:
                path=Path(work)/(letter+kind+'.urdf')
                self.corrections[letter+kind]=clean_urdf(source,path,arm=arm if kind=='A' else None)
                robot=mj.MjSpec.from_file(str(path))
                for gi,geom in enumerate(robot.geoms):
                    geom.contype=1;geom.conaffinity=1
                    if force_static_hand_pairs:
                        geom.name=f'k1_geom_{gi}'
                if kind=='A':
                    frame=parent.worldbody.add_frame(pos=arm.base_pose[:3],quat=arm.base_pose[3:])
                else:
                    body=parent.worldbody.add_body(name=letter+'_palm',mocap=True,pos=[0,0,3])
                    frame=body.add_frame()
                parent.attach(robot,prefix=kind+letter+'_',frame=frame)
        self.attachment=attach_to_spec(parent,resource_root=pad,fixed_root=True,pose=[0,0,0,1,0,0,0],attach_visual=False,collision='sdf')
        table=spec.table
        parent.worldbody.add_geom(name='table',type=mj.mjtGeom.mjGEOM_BOX,size=table.half_extents,pos=table.pose[:3],quat=table.pose[3:],contype=1,conaffinity=1)
        for f in spec.fixtures:
            if f.type=='box' and f.collision:
                if f.name in moving_fixtures:
                    body=parent.worldbody.add_body(name=f.name+'_mover',mocap=True,pos=f.pose[:3],quat=f.pose[3:])
                    body.add_geom(name=f.name,type=mj.mjtGeom.mjGEOM_BOX,size=f.half_extents,contype=1,conaffinity=1)
                else:
                    parent.worldbody.add_geom(name=f.name,type=mj.mjtGeom.mjGEOM_BOX,size=f.half_extents,pos=f.pose[:3],quat=f.pose[3:],contype=1,conaffinity=1)
            elif f.type=='mesh' and f.collision:
                # K1-b's vertical thin post has a sloped top; a convex mesh
                # preserves that geometry without tilting the entire column.
                asset=parent.add_mesh(name=f.name+'_fixture_mesh',file=str(f.mesh))
                parent.worldbody.add_geom(name=f.name,type=mj.mjtGeom.mjGEOM_MESH,meshname=asset.name,
                                          pos=f.pose[:3],quat=f.pose[3:],contype=1,conaffinity=1)
        self.model=parent.compile()
        if force_static_hand_pairs:
            # MuJoCo omits pairs where both weld groups have no DOFs. Explicit
            # pairs restore whole palm/pad and palm/palm checks for K1 replay.
            m=self.model
            for a in range(m.ngeom):
                ba=m.body(m.geom_bodyid[a]).name
                ga=next((k for k in ('HL','HR') if ba.startswith(k+'_')), 'pad' if ba.endswith('controller_body') or ba.startswith('xbox_') else None)
                wa=m.body_weldid[m.geom_bodyid[a]]
                if ga is None or m.body_dofnum[wa]!=0:continue
                for b in range(a+1,m.ngeom):
                    bb=m.body(m.geom_bodyid[b]).name
                    gb=next((k for k in ('HL','HR') if bb.startswith(k+'_')), 'pad' if bb.endswith('controller_body') or bb.startswith('xbox_') else None)
                    wb=m.body_weldid[m.geom_bodyid[b]]
                    if gb is None or m.body_dofnum[wb]!=0:continue
                    if {ga,gb} not in ({'HL','pad'},{'HR','pad'},{'HL','HR'}):continue
                    parent.add_pair(geomname1=m.geom(a).name,geomname2=m.geom(b).name)
            self.model=parent.compile()
        self.data=mj.MjData(self.model)
        self.pad_prefix='xbox_'
        # Attach API exposes exact names via its prefix (currently xbox_).
        self.padroot=next(i for i in range(self.model.nbody) if self.model.body(i).name.endswith('controller_body'))
        self.pad_prefix=self.model.body(self.padroot).name.removesuffix('controller_body')
        self.groups=[]
        for g in range(self.model.ngeom):
            b=self.model.body(self.model.geom_bodyid[g]).name
            self.groups.append(next((x for x in ('HL','HR','AL','AR') if b.startswith(x+'_')),'pad' if b.startswith(self.pad_prefix) else 'fixture'))
        self.handj={}
        for side,letter in [('left','L'),('right','R')]:
            hand=getattr(spec,side+'_hand');root=ET.parse(hand.urdf).getroot()
            self.handj[side]=[(j.get('name'),self.model.joint('H'+letter+'_'+j.get('name')).qposadr[0],j.find('mimic')) for j in root.findall('joint') if j.get('type')!='fixed']
        for side,letter in [('left','L'),('right','R')]:
            self.set_arm(side,getattr(spec,side+'_arm').home_q)
        mj.mj_forward(self.model,self.data)

    def set_arm(self,side,q):
        letter='L' if side=='left' else 'R'
        for n,value in zip(getattr(self.spec,side+'_arm').joint_names,q):
            self.data.qpos[self.model.joint('A'+letter+'_'+n).qposadr[0]]=value

    def set_hand(self,side,palm,q):
        letter='L' if side=='left' else 'R'
        bid=self.model.body(letter+'_palm').mocapid[0]
        pose=matrix_to_pose(palm);self.data.mocap_pos[bid]=pose[:3];self.data.mocap_quat[bid]=pose[3:]
        for n,idx,mimic in self.handj[side]:
            if mimic is None:
                self.data.qpos[idx]=q.get(n,0.)
            else:
                self.data.qpos[idx]=q.get(mimic.get('joint'),0.)*float(mimic.get('multiplier','1'))+float(mimic.get('offset','0'))

    def set_pad(self,pose,values):
        self.model.body_pos[self.padroot]=pose[:3];self.model.body_quat[self.padroot]=pose[3:]
        for j in range(self.model.njnt):
            name=self.model.joint(j).name
            if name.startswith(self.pad_prefix):
                self.data.qpos[self.model.jnt_qposadr[j]]=values.get(name[len(self.pad_prefix):],0.)

    def check(self,side,part=None,finger=None):
        self.mj.mj_forward(self.model,self.data)
        hand='HL' if side=='left' else 'HR';arm='AL' if side=='left' else 'AR'
        bad=[];grips=set();min_dist=0.
        self.last_active_contact=False
        self.last_active_distance=None
        for c in self.data.contact:
            a,b=map(int,c.geom);ga,gb=self.groups[a],self.groups[b]
            ba=self.model.body(self.model.geom_bodyid[a]).name;bb=self.model.body(self.model.geom_bodyid[b]).name
            if ga==gb: continue  # Self-collision has the same evidence limit as the adopted scene.
            relevant=(hand in (ga,gb)) or (arm in (ga,gb) and ('fixture' in (ga,gb) or ('AL' in (ga,gb) and 'AR' in (ga,gb))))
            if not relevant:continue
            depth=float(c.dist);min_dist=min(min_dist,depth)
            if {ga,gb}=={hand,arm}:continue  # Connected mounting seam; no own arm/hand self-check.
            if hand in (ga,gb) and 'pad' in (ga,gb):
                hb=ba if ga==hand else bb;pb=bb if ga==hand else ba
                grip=next((f for f in ('ring','pinky') if hb.endswith(f+'_distal')),None)
                if grip and pb.endswith('controller_body'):
                    grips.add(grip)
                    if depth>=-GRIP_PENETRATION:continue
                if part and finger and hb.endswith(finger+'_distal') and pb.endswith(part):
                    if -.0001<=depth<=0:
                        self.last_active_contact=True
                        self.last_active_distance=depth
                        continue
            if depth < -1e-6:
                bad.append(dict(a=ba,b=bb,geom_a=self.model.geom(a).name,geom_b=self.model.geom(b).name,distance_m=depth))
        return not bad,bad,grips,min_dist

    def pair_clearance(self):
        """Native geom distances for hand-hand and arm-arm; mesh convex approximations."""
        out={}
        distance_groups=list(self.groups)
        # Fixed URDF root meshes can be fused into world; their mesh asset prefix
        # still identifies the arm, so include them in whole-arm spacing.
        for geom in range(self.model.ngeom):
            mid=int(self.model.geom_dataid[geom])
            if mid>=0 and int(self.model.geom_type[geom])==int(self.mj.mjtGeom.mjGEOM_MESH):
                name=self.model.mesh(mid).name
                distance_groups[geom]=next((x for x in ('HL','HR','AL','AR') if name.startswith(x+'_')),distance_groups[geom])
        self.distance_group_counts={x:distance_groups.count(x) for x in ('HL','HR','AL','AR')}
        for a,b in [('HL','HR'),('AL','AR'),('HL','AR'),('HR','AL')]:
            best=.5
            for i in np.where(np.array(distance_groups)==a)[0]:
                for j in np.where(np.array(distance_groups)==b)[0]:
                    best=min(best,float(self.mj.mj_geomDistance(self.model,self.data,int(i),int(j),best,None)))
            out[a+'_'+b]=best
        return out


def fixtures(base_pose):
    base=pose_to_matrix(base_pose)
    result=[]
    # Heights are root elevation relative to current placement, not guessed floor gaps.
    for name,h,tilt,yaw in [('a_flat',0,0,0),('b_raise40',.04,0,0),('b_raise70',.07,0,0),
                           ('c_tilt30_raise40',.04,30,0),('c_tilt45_raise70',.07,45,0),
                           ('d_yaw180_raise70',.07,30,180)]:
        f=base.copy();f[:3,:3]=base[:3,:3]@Rotation.from_euler('zx',[yaw,tilt],degrees=True).as_matrix();f[2,3]+=h
        result.append(dict(name=name,elevation_m=h,tilt_deg=tilt,extra_yaw_deg=yaw,pose=list(matrix_to_pose(f))))
    return result
