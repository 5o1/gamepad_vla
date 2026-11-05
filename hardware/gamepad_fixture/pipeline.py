#!/usr/bin/env python3
"""Generate, validate STEP overrides, export simulation and a separate trial scene."""
import os
os.environ.setdefault("OPENBLAS_NUM_THREADS","1")
os.environ.setdefault("OMP_NUM_THREADS","1")
import argparse
import json
import shutil
from pathlib import Path
import trimesh
from fixture import HERE,ROOT,Parameters,export_all
from check_fixture import main as check_fixture
from convex_collision import cad_cover,write_obj,solid,mesh,subtract_convex
from check_collision import check as check_collision
from scene_export import export as export_scene


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,default=HERE/'out_v3_1');p.add_argument('--overrides',type=Path,default=HERE/'user_overrides');p.add_argument('--baseline-convex',type=Path,default=ROOT/'.local/fixture-convex/selected');p.add_argument('--spec-path',type=Path,default=ROOT/'configs/workcell/placement_trials/gamepad_fixture_v3_1.json');a=p.parse_args()
    mf=export_all(Parameters(),a.out,a.overrides)
    if not check_fixture(a.out):raise RuntimeError('CAD/print hard gate failed; see checks.json')
    finish(a.out,mf,a.baseline_convex,a.spec_path)

def finish(out,mf,baseline_convex,spec_path):
    directory=out/'convex';directory.mkdir(exist_ok=True);cover=cad_cover();overridden={r['part'] for r in mf['user_overrides']}
    import coacd
    coacd.set_log_level('error')
    for row in mf['parts']:
        n=row['name']
        if n.startswith('hardware_'):continue
        # Existing measured compact hulls are reusable only for unchanged geometry.
        # Compare final CAD with baseline actual STL before reusing a proxy.
        source=baseline_convex/(n+'.obj');baseline=HERE/'out/assembly_mesh'/(n+'.stl');actual=out/'assembly_mesh'/(n+'.stl')
        unchanged=baseline.exists() and baseline.read_bytes()==actual.read_bytes()
        if n not in overridden and unchanged and source.exists():shutil.copyfile(source,directory/(n+'.obj'))
        elif n not in overridden:write_obj(directory/(n+'.obj'),cover[n])
        else:
            # Small CAD edits often have exact convex difference bodies (e.g.
            # chamfers). Cut those actual differences from the nominal cover;
            # this retains small bores that normalized CoACD may fill.
            from build123d import import_step,export_stl
            nominal=out/'reference/nominal_steps'/(n+'.step')
            before=import_step(nominal);after=import_step(out/'assembly_parts'/(n+'.step'))
            removed=[v for v in (before-after).solids() if v.volume>1e-4]
            added=[v for v in (after-before).solids() if v.volume>1e-4]
            delta=[]
            for j,v in enumerate(removed+added):
                path=out/'reference'/f'delta_{n}_{j}.stl';export_stl(v,path,tolerance=.02)
                q=trimesh.load(path,force='mesh');delta.append(q)
            if all(q.is_convex for q in delta):
                cells=[solid(h) for h in cover[n]]
                for q in delta[:len(removed)]:cells=[piece for cell in cells for piece in subtract_convex(cell,solid(q))]
                hulls=[mesh(c).convex_hull for c in cells if c.volume()>1e-3]+[q.convex_hull for q in delta[len(removed):]]
                print(n,'actual convex STEP differences:',len(removed),'removed,',len(added),'added',flush=True)
            else:
                m=trimesh.load(actual,force='mesh')
                hulls=[m.convex_hull] if m.is_convex else [trimesh.Trimesh(v,f) for v,f in coacd.run_coacd(coacd.Mesh(m.vertices,m.faces),threshold=.01,preprocess_mode='off',max_ch_vertex=255,seed=0,mcts_iterations=60,mcts_nodes=10)]
            write_obj(directory/(n+'.obj'),hulls)
    accuracy=check_collision(directory,cad_out=out)
    if not all(accuracy['gates'].values()):raise RuntimeError('Convex hard gate failed; see convex/accuracy.json')
    export_scene(out,out/'scene_checks',directory,spec_path)
    if not json.loads((out/'scene_checks/projection.json').read_text())['pass_all_edges']:raise RuntimeError('camera_3 FOV gate failed')
    from replay_hands import replay
    replay(out,spec_path)
    from render import main as render
    from design_sheet import main as sheet
    render(out);sheet(out,out)
    print('PASS: CAD, printing, convex, projection, 34 hand states; scene:',spec_path)
if __name__=='__main__':main()
