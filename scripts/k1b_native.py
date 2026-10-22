#!/usr/bin/env python3
"""K1-b full native screening, collision repair and fixed-palm control checks."""
from __future__ import annotations
import argparse,json,time
from collections import Counter
from dataclasses import replace
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares

from gamepad_vla.feasibility import CollisionScene
from gamepad_vla.workstation.spec import FixtureSpec,pose_to_matrix,matrix_to_pose
from k1_height_common import MeshGeometry,CameraGate,SUPPORT_NAME,write_json
from k1b_search import Problem,column_box,cpu_seconds
from k1b_contact_fit import fit_goal_modes


class Native:
    def __init__(self,problem,work,final_spec=None):
        self.p=problem;work=Path(work);work.mkdir(parents=True,exist_ok=True)
        spec=problem.spec
        self.mutable_column=final_spec is None
        maxpost=FixtureSpec(SUPPORT_NAME,'box',(.45,.62,problem.top+.25,*matrix_to_pose(np.block([[np.array([[0.,1,0],[-1,0,0],[0,0,1]]),np.zeros((3,1))],[np.zeros((1,3)),np.ones((1,1))]]))[3:]),
            True,'design','K1-b mutable conservative screening envelope; final scene uses sloped-top mesh.',(.010,.012,.25))
        spec=replace(spec,fixtures=(*spec.fixtures,maxpost)) if final_spec is None else final_spec
        self.scene=CollisionScene(spec,problem.cfg['resource_root'],work,moving_fixtures=(SUPPORT_NAME,) if self.mutable_column else (),force_static_hand_pairs=True)
        self.geo=MeshGeometry(self.scene);self.camera=CameraGate(problem.camera,self.geo)
        letter='L' if problem.side=='left' else 'R'
        self.hand_group='H'+letter;self.arm_group='A'+letter
        self.robot_geoms=self.geo.groups[self.hand_group]+self.geo.groups[self.arm_group]
        self.robot_index={g:i for i,g in enumerate(self.robot_geoms)}

    def place(self,x,hand_override=None,gamepad_q=None):
        p=self.p;scene=self.scene
        pad,palm,qarm,qdict,frames=p.unpack(np.asarray(x))
        if hand_override is not None:qdict=hand_override
        pose=list(matrix_to_pose(pad));post=column_box(p.spec,pose)
        if self.mutable_column:
            gid=scene.model.geom(SUPPORT_NAME).id;mocap=scene.model.body(SUPPORT_NAME+'_mover').mocapid[0]
            scene.data.mocap_pos[mocap]=post.pose[:3];scene.data.mocap_quat[mocap]=post.pose[3:]
            scene.model.geom_size[gid]=post.half_extents
        scene.set_arm(p.side,qarm);scene.set_hand(p.side,palm,qdict);scene.set_pad(pose,gamepad_q or {})
        return pad,palm,qarm,qdict

    def collisions(self,x,hand_override=None,gamepad_q=None,goal=None):
        pad,palm,qarm,qdict=self.place(x,hand_override,gamepad_q)
        scene=self.scene;p=self.p
        clear,bad,grips,minimum=scene.check(p.side,goal['part'] if goal else None,goal['finger'] if goal else None)
        active=scene.last_active_contact;active_distance=scene.last_active_distance
        for c in scene.data.contact:
            g,h=map(int,c.geom)
            if {scene.groups[g],scene.groups[h]}=={self.arm_group,'pad'} and c.dist < -1e-6:
                bad.append(dict(a=scene.model.body(scene.model.geom_bodyid[g]).name,b=scene.model.body(scene.model.geom_bodyid[h]).name,
                    geom_a=scene.model.geom(g).name,geom_b=scene.model.geom(h).name,distance_m=float(c.dist)))
        # The native automatic contact table skips mocap-root/static pairs.
        # Native convex distances explicitly cover every hand/arm part vs post.
        column=scene.model.geom(SUPPORT_NAME).id;support_distance=.002
        for g in self.robot_geoms:
            d=float(scene.mj.mj_geomDistance(scene.model,scene.data,g,column,.002,None));support_distance=min(support_distance,d)
            if d < -1e-6:
                bad.append(dict(a=scene.model.body(scene.model.geom_bodyid[g]).name,b=SUPPORT_NAME,
                    geom_a=scene.model.geom(g).name,geom_b=SUPPORT_NAME,distance_m=d))
        return bad,grips,active,active_distance,support_distance,(pad,palm,qarm,qdict)

    def penalty(self,x):
        bad,grips,active,ad,support,frames=self.collisions(x)
        values=np.zeros(len(self.robot_geoms))
        for b in bad:
            for key in ('geom_a','geom_b'):
                try:g=self.scene.model.geom(b[key]).id
                except KeyError:continue
                if g in self.robot_index:values[self.robot_index[g]]=max(values[self.robot_index[g]],-b['distance_m'])
        return values*12

    def evaluate(self,x,hand_override=None,gamepad_q=None,goal=None,state=None,solution=None):
        p=self.p;x=np.asarray(x)
        bad,grips,active,ad,support,(pad,palm,qarm,qdict)=self.collisions(x,hand_override,gamepad_q,goal)
        gaps=self.geo.table_clearance(p.side);gate=self.camera.measure((p.side,))
        centre=self.geo.centroid('pad');m=p.feature.measure(qarm,centre)
        natural={k:bool(v[0]) if np.asarray(v).dtype.kind=='b' else float(v[0]) for k,v in m.items() if np.asarray(v).ndim==1}
        summary=p.summary(x);point=summary['grip_point_error_m'];reasons=[]
        if max(point)>.00025:reasons.append('grip_point_error')
        if not natural['natural']:reasons.append('natural_hard')
        if min(gaps.values())<.030-1e-10:reasons.append('table_clearance_30mm')
        if grips!={'ring','pinky'}:reasons.append('native_grip_missing:'+','.join(sorted({'ring','pinky'}-grips)))
        if bad:reasons.append('native_collision')
        reasons.extend(gate['reasons'])
        contact=None
        if goal:
            finger=p.fingers[goal['finger']];q=np.array([qdict[n] for n in finger.chain.active])
            fr=finger.chain.forward(q[None])[0];mode=solution['mode']
            cp=np.linalg.inv(pad)@palm@fr
            point_actual=cp[:3,:3]@finger.points[mode]+cp[:3,3]
            normal=cp[:3,:3]@finger.normals[mode]
            error=float(np.linalg.norm(point_actual-np.array(state['point'])))
            dot=float(np.dot(normal,-np.array(state['normal'])))
            normal_required=goal['name'] not in ('LS','RS','LT','RT')
            if error>.00025 or (normal_required and dot<np.cos(np.pi/6)):reasons.append('control_point_or_cone')
            if not active:reasons.append('native_active_contact_missing')
            contact=dict(point_error_m=error,normal_cos=dot,native_active_contact=active,native_active_distance_m=ad,mode=int(mode))
        return dict(passed=not reasons,reasons=reasons,grips=sorted(grips),violations=bad,
            table_clearance_m=gaps,camera=gate,natural=natural,grip_point_error_m=point,
            support_distance_lower_bound_m=support,active_contact=contact,
            gamepad_pose=list(matrix_to_pose(pad)),palm_world=list(matrix_to_pose(palm)),arm_q=qarm.tolist(),hand_q=qdict)

    def controls(self,x):
        p=self.p;pad,palm,qarm,qdict,frames=p.unpack(x);palm_in_pad=np.linalg.inv(pad)@palm
        controls=[];accepted=[];diagnostics={}
        for goal in p.goals:
            fits=fit_goal_modes(p.fingers[goal['finger']],goal,palm_in_pad)
            passed_states=[];attempts=Counter();state_diagnostics=[]
            for row in fits['states']:
                state=goal['states'][row['state']];winner=None
                for solution in row['solutions']:
                    hand=qdict.copy();hand.update(zip(p.fingers[goal['finger']].chain.active,solution['q']))
                    check=self.evaluate(x,hand,state['gamepad_q'],goal,state,solution)
                    if check['passed']:
                        winner=dict(control=goal['name'],state=row['state'],hand_q=hand,gamepad_q=state['gamepad_q'],check=check)
                        accepted.append(winner);break
                    attempts.update(check['reasons'])
                passed_states.append(winner is not None)
                state_diagnostics.append(dict(state=row['state'],point_ik_solutions=len(row['solutions']),native_passed=winner is not None))
            passed=any(passed_states) if goal.get('alternatives') else all(passed_states)
            if passed:controls.append(goal['name'])
            diagnostics[goal['name']]=dict(passed=passed,states=state_diagnostics,rejections=dict(attempts))
            print(json.dumps(dict(stage='control',side=p.side,control=goal['name'],passed=passed)),flush=True)
        return controls,accepted,diagnostics


def main():
    a=argparse.ArgumentParser(description=__doc__)
    a.add_argument('--task',type=Path,default=Path('configs/tasks/fixed_dpad_causal.json'))
    a.add_argument('--side',choices=['left','right'],required=True);a.add_argument('--proposals',type=Path,required=True)
    a.add_argument('--output',type=Path,required=True);a.add_argument('--polish',type=int,default=20)
    a.add_argument('--polish-nfev',type=int,default=55);a.add_argument('--controls-limit',type=int,default=16)
    args=a.parse_args();args.output.mkdir(parents=True,exist_ok=True);start=time.monotonic();initialcpu=cpu_seconds()
    p=Problem(args.task,args.side);native=Native(p,args.output/'model')
    proposed=json.loads(args.proposals.read_text())['proposals'];attempts=[];accepted=[];refinements=0
    for index,row in enumerate(proposed):
        x=np.array(row['x']);check=native.evaluate(x);was_polished=False
        initialcheck=check
        if not check['passed'] and refinements<args.polish and row['natural']['deviation']<1.25 and max(row['grip_point_error_m'])<.003:
            refinements+=1;was_polished=True
            result=least_squares(lambda z:np.r_[p.residual(z,x),native.penalty(z)],x,bounds=(p.lower,p.upper),
                x_scale=np.r_[.08,.08,.08,.3,np.ones(7)*.5,np.ones(len(p.lo))*.5],diff_step=1e-4,
                max_nfev=args.polish_nfev,ftol=1e-8,xtol=1e-8,gtol=1e-8)
            x=result.x;check=native.evaluate(x)
        attempts.append(dict(seed_index=row['seed_index'],polished=was_polished,initial=initialcheck,final=check))
        if check['passed']:
            accepted.append(dict(x=x.tolist(),source_seed_index=row['seed_index'],check=check,controls=[],states=[]))
        if (index+1)%8==0:
            write_json(args.output/'progress.json',dict(checked=index+1,accepted=accepted,attempts=attempts))
            print(json.dumps(dict(stage='native',side=args.side,checked=index+1,accepted=len(accepted),refinements=refinements,cpu_seconds=cpu_seconds()-initialcpu)),flush=True)
        if cpu_seconds()-initialcpu>6000:break
    # Keep geometry-valid candidates even when no operating control passes.
    for e in accepted[:args.controls_limit]:
        e['controls'],e['states'],e['control_diagnostics']=native.controls(np.array(e['x']))
    accepted.sort(key=lambda e:(-len(e['controls']),e['x'][2],e['check']['natural']['wrist_deviation_deg'],e['check']['camera']['margin_px']))
    out=dict(side=args.side,checked=len(attempts),refinements=refinements,accepted=accepted,attempts=attempts,
             elapsed_seconds=time.monotonic()-start,cpu_seconds=cpu_seconds()-initialcpu,
             acceptance='natural is hard; native grips, native collision, original full mesh 30mm, actual calibrated camera; separate recording; common placement still required')
    write_json(args.output/'native.json',out)
    print(json.dumps(dict(stage='native_done',side=args.side,accepted=len(accepted),best_controls=accepted[0]['controls'] if accepted else [],elapsed_seconds=out['elapsed_seconds'],cpu_seconds=out['cpu_seconds'])),flush=True)


if __name__=='__main__':main()
