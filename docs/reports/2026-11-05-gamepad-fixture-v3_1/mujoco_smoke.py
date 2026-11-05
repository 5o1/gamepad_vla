"""Compile the exported multi-convex scene with the existing production loader."""
import argparse,json
from pathlib import Path
from gamepad_vla.workstation.spec import load_spec
from gamepad_vla.workstation.collection_mujoco import CollectionMujocoBackend
from gamepad_vla.workstation.fixture_collision import convex_objects
p=argparse.ArgumentParser();p.add_argument('--spec',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
spec=load_spec(a.spec)
expected=sum(len(list(convex_objects(Path(f.convex_mesh)))) for f in spec.fixtures if f.collision and f.convex_mesh)
b=CollectionMujocoBackend(spec,work_dir=a.out/'mujoco',render=False)
ids=[i for i in range(b.model.ngeom) if b.model.geom(i).name.startswith('v3_visual_')]
r=dict(executed=True,compiled=True,fixture_geom_count=len(ids),mesh_geom_count=sum(int(b.model.geom_type[i])==int(b.mj.mjtGeom.mjGEOM_MESH) for i in ids),scope='Native compile/import only; no dynamic manipulation claim')
assert len(ids)==expected and r['mesh_geom_count']==expected and expected>0,(r,expected)
a.out.mkdir(parents=True,exist_ok=True);(a.out/'mujoco_smoke.json').write_text(json.dumps(r,indent=2)+'\n');print(r);b.close()
