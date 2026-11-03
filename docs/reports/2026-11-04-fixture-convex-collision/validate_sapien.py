"""Bounded CPU physics validation. Does not connect to station hardware/services."""
import argparse,gc,json,os,time,xml.etree.ElementTree as ET
import importlib.metadata
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import sapien
from gamepad_vla.workstation.spec import load_spec,pose_to_matrix
from gamepad_vla.workstation.collection_sapien import CollectionSapienBackend,_PhysicsScene
from gamepad_vla.workstation.fixture_collision import add_sapien_fixture_collision,configure_sapien_shapes,sapien_shapes
from gamepad_vla.workstation.robot_description import compose_robot_urdf
from xbox_gamepad_physics.robotwin import load_into_scene


def make_scene(spec,work,fixtures=True):
    work.mkdir(parents=True,exist_ok=True)
    scene=sapien.Scene([sapien.physx.PhysxCpuSystem()]);scene.set_timestep(.001)
    actors=[]
    if fixtures:
        for f in spec.fixtures:
            if not f.collision or not f.name.startswith('v3_'):continue
            b=scene.create_actor_builder();add_sapien_fixture_collision(b,f)
            a=b.build_static(name=f.name);a.set_pose(sapien.Pose(f.pose[:3],f.pose[3:]));configure_sapien_shapes(a,exclude_fixture_gamepad=True);actors.append(a)
    g=load_into_scene(_PhysicsScene(scene,work),sapien.Pose(spec.gamepad.root_pose[:3],spec.gamepad.root_pose[3:]),fixed_root=True,profile=spec.gamepad.profile,collision=spec.gamepad.collision,attach_visual=False)
    for link in g.articulation.get_links():configure_sapien_shapes(link,exclude_fixture_gamepad=True)
    return scene,g,actors


def pair_counts(scene,gamepad,actors,probe=None):
    pads={l.entity for l in gamepad.articulation.get_links()};fx=set(actors);result=dict(fixture_gamepad=0,fixture_probe=0,gamepad_probe=0,gamepad_internal=0)
    for c in scene.get_contacts():
        a,b=c.bodies[0].entity,c.bodies[1].entity
        if (a in fx and b in pads) or (b in fx and a in pads):result['fixture_gamepad']+=1
        if a in pads and b in pads:result['gamepad_internal']+=1
        if probe is not None:
            if (a in fx and b is probe) or (b in fx and a is probe):result['fixture_probe']+=1
            if (a in pads and b is probe) or (b in pads and a is probe):result['gamepad_probe']+=1
    return result


def plan(g,name,W):
    # Adapted from xbox-gamepad-physics acceptance/robotwin/probe.py:
    # ray through cooked control hulls, limited-force dynamic sphere.
    joint_name,link_name={'A':('a_press','a'),'B':('b_press','b'),'LB':('left_shoulder_press','left_shoulder'),'Pair':('pair_press','pair')}[name]
    link=next(l for l in g.articulation.get_links() if l.name==link_name);j=next(j for j in g.articulation.get_active_joints() if j.name==joint_name)
    direction=(j.get_child_link().pose*j.get_pose_in_child()).to_transformation_matrix()[:3,0]
    if name=='Pair':direction=-direction
    vertices=[];hulls=[]
    for shape in link.get_collision_shapes():
        T=(link.pose*shape.local_pose).to_transformation_matrix();v=np.asarray(shape.get_vertices())*shape.get_scale();v=v@T[:3,:3].T+T[:3,3];hulls.append(v);vertices.extend(v)
    center=np.mean(np.array([np.min(vertices,axis=0),np.max(vertices,axis=0)]),axis=0);outward=-direction;hits=[]
    # Supporting plane of the cooked control geometry; starts outside all
    # vertices. This avoids requiring SciPy in the minimal SAPIEN environment.
    surface=center+max(float(((v-center)@outward).max()) for v in hulls)*outward
    return surface+.009*outward,direction,joint_name,j


def presses(spec,work,fixtures):
    rows=[]
    for name in ('A','B','LB','Pair'):
        scene,g,actors=make_scene(spec,work/name,fixtures);W=pose_to_matrix(spec.gamepad.root_pose);start,direction,joint_name,j=plan(g,name,W)
        builder=scene.create_actor_builder();builder.add_sphere_collision(radius=.004);builder.set_mass_and_inertia(.05,sapien.Pose(),[3.2e-7]*3);builder.set_initial_pose(sapien.Pose(start));entity=builder.build(name='sphere_probe');body=entity.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent);configure_sapien_shapes(entity)
        initial=dict(g.joint_positions());limits=j.get_limits()[0];peak=0.;counts=dict(fixture_gamepad=0,fixture_probe=0,gamepad_probe=0,gamepad_internal=0);max_force=0.
        root=g.articulation.get_root_pose().to_transformation_matrix().copy()
        for step in range(3500):
            target=start+direction*min(.025,step*.001*.005)
            force=3000*(target-body.pose.p)-12*body.linear_velocity+np.array([0,0,.05*9.81]);force*=min(1,16/max(np.linalg.norm(force),1e-10));max_force=max(max_force,float(np.linalg.norm(force)));body.add_force_at_point(force,body.pose.p);scene.step()
            q=dict(g.joint_positions())[joint_name];peak=max(peak,float((q-initial[joint_name])/(limits[1]-initial[joint_name])))
            for key,value in pair_counts(scene,g,actors,entity).items():counts[key]+=value
        rows.append(dict(control=name,fixtures=fixtures,peak_normalized_displacement=peak,counts=counts,max_force_n=max_force,root_matrix_change=float(np.max(abs(g.articulation.get_root_pose().to_transformation_matrix()-root))),pressed=peak>=.95))
        del scene,g,actors,body,entity;gc.collect()
    return rows


def block_probes(spec,work):
    # Extract one real fingertip and one real arm convex collision mesh from
    # the composed production URDF; drop each onto the CAD base's broad face.
    urdf=compose_robot_urdf(spec,work/'probe_robot.urdf');xml=ET.parse(urdf).getroot();rows=[]
    for kind,pattern in [('hand','index_distal'),('arm','Link3')]:
        links=xml.findall('link');link=next((l for l in links if pattern.lower() in l.get('name','').lower() and l.find('collision/geometry/mesh') is not None),None)
        if link is None:
            link=next(l for l in links if l.find('collision/geometry/mesh') is not None and ('arm' in l.get('name','').lower() if kind=='arm' else 'distal' in l.get('name','').lower()))
        source=link.find('collision/geometry/mesh');filename=source.get('filename');scale=[float(v) for v in source.get('scale','1 1 1').split()]
        for enabled in (True,False):
            scene,g,actors=make_scene(spec,work/(kind+str(enabled)),enabled)
            b=scene.create_actor_builder();b.add_convex_collision_from_file(filename,scale=scale);b.set_mass_and_inertia(.05,sapien.Pose(),[1e-4]*3);e=b.build(name='actual_'+kind+'_convex');body=e.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent);configure_sapien_shapes(e)
            vs=np.vstack([np.asarray(s.get_vertices())*s.get_scale() for s in sapien_shapes(e)])
            # Base top = tabletop .767 + .75 mm base pad + 16 mm plate.
            point=np.array([.700,.535,.78375]);start=point-np.r_[vs[:,:2].mean(0),vs[:,2].min()]+np.array([0,0,.004]);e.set_pose(sapien.Pose(start))
            contact_steps=0;impulse=0.
            for _ in range(600):
                scene.step()
                for c in scene.get_contacts():
                    if any(x.entity is e for x in c.bodies):
                        contact_steps+=1;impulse+=sum(np.linalg.norm(p.impulse) for p in c.points)
            final_vertices=vs@e.pose.to_transformation_matrix()[:3,:3].T+e.pose.p;min_z=float(final_vertices[:,2].min());rows.append(dict(kind=kind,link=link.get('name'),fixture_enabled=enabled,minimum_z_m=min_z,base_top_m=point[2],contact_steps=contact_steps,impulse_ns=float(impulse),blocked=bool(enabled and abs(min_z-point[2])<.001),passed_through_without_fixture=bool(not enabled and min_z<point[2]-.05)))
    return rows


def filter_probe(spec,work):
    rows=[]
    for filtered in (True,False):
        scene,g,_=make_scene(spec,work/str(filtered),False)
        builder=scene.create_actor_builder();builder.add_box_collision(half_size=[.08,.06,.04])
        fixture=builder.build_static(name='overlapping_fixture_box');fixture.set_pose(sapien.Pose(spec.gamepad.root_pose[:3],spec.gamepad.root_pose[3:]))
        configure_sapien_shapes(fixture,exclude_fixture_gamepad=filtered)
        count=0
        for _ in range(20):
            scene.step();count+=pair_counts(scene,g,[fixture])['fixture_gamepad']
        rows.append(dict(filtered=filtered,fixture_gamepad_contact_pairs=count))
    return rows


def benchmark(spec,work):
    rows=[]
    for full in (False,True):
        for i in range(3):
            t=time.perf_counter();b=CollectionSapienBackend(spec,work_dir=work/f'{full}-{i}',environment_collisions=full,gamepad_collisions=True);load=time.perf_counter()-t
            for _ in range(100):b.scene.step()
            samples=[]
            for _ in range(1000):
                t=time.perf_counter();b.scene.step();samples.append((time.perf_counter()-t)*1e6)
            shapes=[s for a in b._fixed_actors for s in sapien_shapes(a)]
            rows.append(dict(variant='full' if full else 'gamepad-contact',repeat=i,load_seconds=load,step_mean_us=float(np.mean(samples)),step_p95_us=float(np.percentile(samples,95)),environment_shapes=len(shapes),offsets_m=sorted(set(round(s.contact_offset,7) for s in shapes))))
            b.close();del b;gc.collect()
    return rows


def main(a):
    spec=load_spec(a.spec);a.output.mkdir(parents=True,exist_ok=True);result=dict(versions={name:importlib.metadata.version(name) for name in ('sapien','xbox-gamepad-physics')},spec=str(a.spec),benchmark=benchmark(spec,a.output/'benchmark'))
    (a.output/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    if a.functional:
        result['pair_filter_probe']=filter_probe(spec,a.output/'pair_filter');result['press_with_fixture']=presses(spec,a.output/'press_fixture',True);result['press_without_fixture']=presses(spec,a.output/'press_control',False);result['blocking']=block_probes(spec,a.output)
    (a.output/'results.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--spec',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--functional',action='store_true');main(p.parse_args())
