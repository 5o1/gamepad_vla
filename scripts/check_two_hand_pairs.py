#!/usr/bin/env python3
"""Check paired witnesses and report spacing; failed diagnostic pairs stay failed."""
import argparse
import json
from pathlib import Path

import numpy as np

from gamepad_vla.feasibility import FINGERS,CollisionScene,arm_ik,fixtures
from gamepad_vla.tasks.fixed_gamepad import load_task
from gamepad_vla.workstation.spec import load_spec,pose_to_matrix


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--task',type=Path,default=Path('configs/tasks/fixed_dpad_causal.json'))
    p.add_argument('--left',type=Path,required=True);p.add_argument('--right',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--candidates',type=int,default=12)
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    cfg=load_task(args.task);spec=load_spec(cfg['workcell']);scene=CollisionScene(spec,cfg['resource_root'],args.output)
    inputs={s:Path(getattr(args,s)) for s in ('left','right')}
    raw={s:np.load(path/'kinematics.npz') for s,path in inputs.items()}
    summaries={s:json.loads((path/'result.json').read_text()) for s,path in inputs.items()}
    saved={s:json.loads((path/'witnesses.json').read_text()) for s,path in inputs.items()}
    # Reconstruct only chain names; FK clouds are not needed for diagnostics.
    from gamepad_vla.kinematics import SerialChain
    chains={s:{f:SerialChain(getattr(spec,s+'_hand').urdf,f+'_distal') for f in FINGERS} for s in inputs}
    reports=[]
    for fx in fixtures(cfg['gamepad_root_pose']):
        verified_pairs=[];grip_pairs_checked=0
        # First enumerate all actual accepted single-side grips, including the
        # rare partial-control witness, rather than replacing them with diagnostics.
        for left in saved['left'][fx['name']]:
            for right in saved['right'][fx['name']]:
                grip_pairs_checked+=1
                for side,e in [('left',left),('right',right)]:
                    scene.set_arm(side,e['arm_q'])
                    scene.set_hand(side,pose_to_matrix(e['palm_world']),e['resting_hand_q'])
                scene.set_pad(fx['pose'],{})
                lc,lb,lg,_=scene.check('left');rc,rb,rg,_=scene.check('right')
                if not (lc and rc and lg=={'ring','pinky'} and rg=={'ring','pinky'}):continue
                distances=scene.pair_clearance()
                controls=[];accepted_states=[]
                for side,e,other in [('left',left,right),('right',right,left)]:
                    opposite='right' if side=='left' else 'left'
                    for control in e['controls']:
                        goal=next(g for g in summaries[side]['goals'] if g['name']==control)
                        states=[state for state in e['states'] if state['control']==control]
                        passed=[]
                        for state in states:
                            scene.set_hand(opposite,pose_to_matrix(other['palm_world']),other['resting_hand_q'])
                            scene.set_hand(side,pose_to_matrix(e['palm_world']),state['hand_q'])
                            scene.set_pad(fx['pose'],state['gamepad_q'])
                            ok,bad,grips,_=scene.check(side,goal['part'],goal['finger'])
                            actual=scene.last_active_contact
                            other_ok,_,other_grips,_=scene.check(opposite)
                            good=ok and actual and other_ok and grips=={'ring','pinky'} and other_grips=={'ring','pinky'}
                            passed.append(good)
                            if good:accepted_states.append(dict(side=side,**state))
                        if (any(passed) if goal.get('alternatives') else bool(passed) and all(passed)):
                            controls.append(side+':'+control)
                    scene.set_hand(side,pose_to_matrix(e['palm_world']),e['resting_hand_q'])
                verified_pairs.append(dict(left_index=left['palm_index'],right_index=right['palm_index'],
                    controls=controls,distances_m=distances,accepted_states=accepted_states,
                    left_arm_q=left['arm_q'],right_arm_q=right['arm_q'],
                    left_palm_world=left['palm_world'],right_palm_world=right['palm_world'],
                    left_resting_q=left['resting_hand_q'],right_resting_q=right['resting_hand_q']))
        verified_pairs.sort(key=lambda x:(len(x['controls']),min(x['distances_m'].values())),reverse=True)
        candidates={};rng=np.random.default_rng(113)
        for side,d in raw.items():
            ids=d['grip_ids'];c=d['coverage'][ids]
            ids=ids[np.argsort(-c.sum(axis=1),kind='stable')[:args.candidates]]
            palms=pose_to_matrix(fx['pose'])@d['palms'][ids]
            arm=getattr(spec,side+'_arm');hand=getattr(spec,side+'_hand')
            aq,ok,pe,re=arm_ik(arm,hand,palms,rng,seeds=6,iterations=100)
            rows=[]
            for k in np.where(ok)[0]:
                pi=int(ids[k]);j=int(np.searchsorted(d['grip_ids'],pi))
                q={**dict(zip(chains[side]['ring'].active,d['ringq'][pi])),**dict(zip(chains[side]['pinky'].active,d['pinkq'][pi]))}
                for finger in ('thumb','index','middle'):
                    ch=chains[side][finger];q.update(zip(ch.active,ch.active_limits[0]))
                    for gi,g in enumerate(summaries[side]['goals']):
                        if g['finger']==finger and d['coverage'][pi,gi]:
                            si=int(np.where(d[g['name']+'_ok'][j])[0][0])
                            q.update(zip(ch.active,d[g['name']+'_q'][j,si]));break
                rows.append(dict(index=pi,palm=palms[k],q=q,arm=aq[k],coverage=int(d['coverage'][pi].sum())))
            candidates[side]=rows
        pairs=[]
        for left in candidates['left']:
            for right in candidates['right']:
                for side,row in [('left',left),('right',right)]:
                    scene.set_arm(side,row['arm']);scene.set_hand(side,row['palm'],row['q'])
                scene.set_pad(fx['pose'],{})
                lc,lb,lg,_=scene.check('left');rc,rb,rg,_=scene.check('right')
                distances=scene.pair_clearance()
                pairs.append(dict(left_index=left['index'],right_index=right['index'],
                                  left_covered_kinematically=left['coverage'],right_covered_kinematically=right['coverage'],
                                  distances_m=distances,geometry_counts=scene.distance_group_counts,grip_contacts=dict(left=sorted(lg),right=sorted(rg)),
                                  neutral_scene_clear=lc and rc,violations=(lb+rb)[:8]))
        best=max(pairs,key=lambda r:min(r['distances_m'].values())) if pairs else None
        full_left=next(f for f in summaries['left']['fixtures'] if f['name']==fx['name'])['full_hand_feasible']
        full_right=next(f for f in summaries['right']['fixtures'] if f['name']==fx['name'])['full_hand_feasible']
        row=dict(fixture=fx['name'],full_candidate_pairs=full_left*full_right,
                 grip_pairs_checked=grip_pairs_checked,verified_grip_pairs=len(verified_pairs),
                 verified_partial_controls=verified_pairs[0]['controls'] if verified_pairs else [],
                 verified_pair_witness=verified_pairs[0] if verified_pairs else None,
                 diagnostic_pair_count=len(pairs),diagnostic_best_spacing=best,
                 diagnostic_scene_clear_count=sum(x['neutral_scene_clear'] for x in pairs),
                 diagnostic_interpretation='Diagnostic spacing only. These are partial-coverage candidates, not successful grasps or full-control pairs.')
        if full_left and full_right:
            raise SystemExit('Full witnesses exist: exhaustive paired action-state verification required before accepting a full pair.')
        reports.append(row);print(json.dumps(row),flush=True)
    (args.output/'result.json').write_text(json.dumps(dict(fixtures=reports,sampling=dict(top_kinematic_palms_per_side=args.candidates,arm_seeds=6,arm_iterations=100),
        scope='Both arms and both hands placed together. All saved grip pairs checked, then partial-control witnesses rechecked against the other hand and arm. Full-control candidate set empty; separate diagnostics preserve failure.'),indent=2)+'\n')


if __name__=='__main__':main()
