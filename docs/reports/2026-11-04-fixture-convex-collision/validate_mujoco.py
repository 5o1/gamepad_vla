"""Native MuJoCo convex import/filtering and saved V3 hand approach checks."""
import argparse,json,os,time
from pathlib import Path
from dataclasses import replace
import numpy as np
import mujoco as mj
import trimesh
import manifold3d as man
from scipy.spatial import cKDTree
from gamepad_vla.workstation.spec import load_spec,pose_to_matrix
from gamepad_vla.workstation.collection_mujoco import CollectionMujocoBackend
from gamepad_vla.feasibility import mesh_for_geom
from gamepad_vla.workstation.g20_mapping import G20_MIMIC_JOINTS
from gamepad_vla.workstation.fixture_collision import add_mujoco_fixture,convex_objects


def solid(m):return man.Manifold(man.Mesh(np.asarray(m.vertices,np.float32),np.asarray(m.faces,np.uint32)))

def independent_filter_test():
    counts=[]
    for allowed in (False,True):
        s=mj.MjSpec();s.worldbody.add_geom(name='fixture',type=mj.mjtGeom.mjGEOM_BOX,size=[.01]*3,contype=8,conaffinity=6,margin=.0002)
        b=s.worldbody.add_body(name='pad',pos=[0,0,.015]);b.add_freejoint();b.add_geom(name='pad_geom',type=mj.mjtGeom.mjGEOM_BOX,size=[.01]*3,contype=1,conaffinity=6|(8 if allowed else 0),mass=.1)
        model=s.compile();data=mj.MjData(model);mj.mj_forward(model,data);counts.append(dict(filter_enabled=not allowed,contact_pairs=int(data.ncon)))
    return counts


def main(a):
    spec=load_spec(a.spec);fixtures=[]
    for f in spec.fixtures:
        if a.hulls is not None and f.collision and f.type=='mesh' and f.name.startswith('v3_visual_'):
            f=replace(f,convex_mesh=str(a.hulls/(f.name.removeprefix('v3_visual_')+'.obj')))
        fixtures.append(f)
    spec=replace(spec,fixtures=tuple(fixtures));a.output.mkdir(parents=True,exist_ok=True)
    start=time.monotonic();b=CollectionMujocoBackend(spec,work_dir=a.output/'backend');load=time.monotonic()-start
    model,data=b.model,b.data;fx=[g for g in range(model.ngeom) if int(model.geom_contype[g])==8];pad=[g for g in range(model.ngeom) if model.geom(g).name.startswith('xbox_') and model.geom_contype[g]]
    hand=[g for g in range(model.ngeom) if int(model.geom_contype[g]) in (2,4)]
    allowed=lambda i,j:bool(model.geom_contype[i]&model.geom_conaffinity[j] or model.geom_contype[j]&model.geom_conaffinity[i])
    result=dict(spec=str(a.spec),hulls_override=None if a.hulls is None else str(a.hulls),version=mj.__version__,load_seconds=load,fixture_mesh_geoms=sum(model.geom_type[g]==mj.mjtGeom.mjGEOM_MESH for g in fx),fixture_gamepad_allowed_pairs=sum(allowed(i,j) for i in fx for j in pad),fixture_robot_allowed_pairs=sum(allowed(i,j) for i in fx for j in hand),native_filter_control=independent_filter_test(),margin_m=sorted(set(float(model.geom_margin[g]) for g in fx)))
    result['witnesses_executed']=False
    (a.output/'results.json').write_text(json.dumps(result,indent=2,default=lambda x:x.item())+'\n')
    geometry_error=0.
    for f in spec.fixtures:
        if not f.name.startswith('v3_visual_') or not f.collision:continue
        for index,(vertices,faces) in enumerate(convex_objects(f.convex_mesh)):
            geom=model.geom(f'{f.name}_convex_{index:03d}').id
            actual=mesh_for_geom(model,data,geom).vertices
            expected=trimesh.transform_points(vertices,pose_to_matrix(f.pose))
            geometry_error=max(geometry_error,float(cKDTree(expected).query(actual)[0].max()),float(cKDTree(actual).query(expected)[0].max()))
    result['compiled_fixture_vertex_roundtrip_max_mm']=geometry_error*1000
    W=pose_to_matrix(spec.gamepad.root_pose);P=np.linalg.inv(W);fixed=[]
    for f in spec.fixtures:
        if not f.name.startswith('v3_visual_') or not f.collision:continue
        hulls=[solid(trimesh.Trimesh(v*1000,faces,process=False)) for v,faces in convex_objects(f.convex_mesh)]
        s=man.Manifold.batch_boolean(hulls,man.OpType.Add);fixed.append((f.name,s,np.array(s.bounding_box()).reshape(2,3)))
    targets=json.loads((Path(__file__).parent/'control_targets.json').read_text());checks=[]
    for side in ('left','right'):
        source=json.loads((Path(__file__).parent.parent/'2026-11-04-gamepad-fixture-v3/coverage'/(side+'.json')).read_text())
        goals={g['name']:g for g in targets[side]};names=b._arm_names[side],b._hand_names[side]
        for mode,details in source['results'].items():
            for state in details['states']:
                if not state['passed'] or state['control'] not in details['retained_controls']:continue
                b.reset();c=state['check']
                hand_values=dict(c['hand_q'])
                for child,(source,multiplier) in G20_MIMIC_JOINTS[side].items():
                    hand_values[child]=hand_values[source]*multiplier
                for ns,vs in zip(names,(c['arm_q'],hand_values)):
                    values=[vs[name.removeprefix(side+'_').removeprefix('right_')] for name in ns] if isinstance(vs,dict) else vs
                    for name,value in zip(ns,values):data.qpos[model.joint(name).qposadr[0]]=value
                mj.mj_forward(model,data);normal=np.array(goals[state['control']]['states'][state['state']]['normal']);violations=[];max_volume=0.
                for gi in hand:
                    if int(model.geom_contype[gi]) != (2 if side=='left' else 4):continue
                    m=mesh_for_geom(model,data,gi);m.apply_transform(P);m.apply_scale(1000)
                    # Whole actual arm/hand convex collider, with a 3 mm outward
                    # approach translation at unchanged joints and orientation.
                    envelope=trimesh.convex.convex_hull(np.vstack([m.vertices,m.vertices+normal*3.]));ss=None
                    for name,fixture,bounds in fixed:
                        if np.any(envelope.bounds[1]<bounds[0]) or np.any(bounds[1]<envelope.bounds[0]):continue
                        if ss is None:ss=solid(envelope)
                        volume=abs(float((ss^fixture).volume()));max_volume=max(max_volume,volume)
                        if volume>1e-5:violations.append(dict(robot=model.geom(gi).name,fixture=name,volume_mm3=volume))
                checks.append(dict(side=side,mode=mode,control=state['control'],state=state['state'],approach_mm=3.,maximum_overlap_mm3=max_volume,violations=violations))
    result['witnesses_executed']=True
    result['accepted_witness_approaches']=checks;result['witness_approaches_zero_overlap']=all(not c['violations'] for c in checks)
    (a.output/'results.json').write_text(json.dumps(result,indent=2,default=lambda x:x.item())+'\n');print(json.dumps({k:v for k,v in result.items() if k!='accepted_witness_approaches'},default=lambda x:x.item()),flush=True)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--spec',type=Path,required=True);p.add_argument('--hulls',type=Path);p.add_argument('--output',type=Path,required=True);main(p.parse_args())
