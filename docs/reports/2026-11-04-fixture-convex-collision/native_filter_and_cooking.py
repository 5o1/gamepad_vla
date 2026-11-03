"""Read cooked static geometry and exercise filtering with a collidable box."""
import argparse,json,runpy
from pathlib import Path
import numpy as np
from gamepad_vla.workstation.spec import load_spec
from gamepad_vla.workstation.fixture_collision import sapien_shapes


def main(a):
    api=runpy.run_path(str(Path(__file__).with_name('validate_sapien.py')))
    spec=load_spec(a.spec);a.output.mkdir(parents=True,exist_ok=True)
    result=dict(pair_filter_probe=api['filter_probe'](spec,a.output/'filter'))
    scene,g,actors=api['make_scene'](spec,a.output/'cooking',True)
    meshes={}
    for actor in actors:
        if not actor.name.startswith('v3_visual_'):continue
        parts=[]
        for shape in sapien_shapes(actor):
            T=shape.local_pose.to_transformation_matrix();v=np.asarray(shape.get_vertices())*shape.get_scale();v=v@T[:3,:3].T+T[:3,3]
            parts.append(dict(vertices_m=v.tolist(),triangles=np.asarray(shape.get_triangles()).tolist(),contact_offset_m=float(shape.contact_offset),groups=list(shape.get_collision_groups())))
        meshes[actor.name.removeprefix('v3_visual_')]=parts
    (a.output/'cooked_meshes.json').write_text(json.dumps(meshes)+'\n')
    if a.hulls:
        convex={}
        for path in sorted(a.hulls.glob('*.obj')):
            builder=scene.create_actor_builder();builder.add_multiple_convex_collisions_from_file(str(path),decomposition='none')
            actor=builder.build_static(name='convex_'+path.stem);parts=[]
            for shape in sapien_shapes(actor):
                T=shape.local_pose.to_transformation_matrix();v=np.asarray(shape.get_vertices())*shape.get_scale();v=v@T[:3,:3].T+T[:3,3]
                parts.append(dict(vertices_m=v.tolist(),triangles=np.asarray(shape.get_triangles()).tolist()))
            convex[path.stem]=parts
        (a.output/'cooked_convex.json').write_text(json.dumps(convex)+'\n')
    (a.output/'filter.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result),flush=True)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--spec',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--hulls',type=Path);main(p.parse_args())
