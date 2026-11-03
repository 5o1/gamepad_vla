#!/usr/bin/env python3
"""Task Z bounded CPU experiment. Reuses X scenes and S native predicates."""
from __future__ import annotations
import argparse
from collections import Counter
import json
import os
from pathlib import Path
import time
import traceback

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from natural_grip_fixed import FixedProblem, FixedNative
from check_natural_grip_fixed import reach_diagnostics
from k1b_contact_fit import fit_goal_modes
from k1_height_common import write_json
from gamepad_vla.feasibility import transform
from gamepad_vla.workstation.spec import pose_to_matrix

DEADLINE = float('inf')


def tick():
    if time.time() >= DEADLINE:
        raise TimeoutError('independent point/side wall-clock limit')


class ZProblem(FixedProblem):
    def natural_residual(self, q):
        m = self.feature.measure(q, self.world_centre)
        up, pitch, heading = [m[k][0] for k in
            ('upper_tilt_deg', 'forearm_pitch_deg', 'forearm_heading_deg')]
        return np.array([max(0., 10-up), max(0., up-35),
            max(0., -10-pitch), max(0., pitch-25), max(0., heading-30),
            max(0., -(m['elbow']-m['shoulder'])[0, 0])*100])*.025


class ZNative(FixedNative):
    def evaluate(self, *args, **kwargs):
        c = super().evaluate(*args, **kwargs)
        c['soft_grip_reasons'] = [r for r in c['reasons'] if
            r == 'grip_point_error' or r.startswith('native_grip_missing:')]
        c['reasons'] = [r for r in c['reasons'] if r not in c['soft_grip_reasons']]
        c['passed'] = not c['reasons']
        return c

    def collision_vector(self, x, goal=None, state=None):
        bad, *_ = self.collisions(x, gamepad_q=state['gamepad_q'] if state else None, goal=goal)
        v = np.zeros(len(self.robot_geoms))
        for b in bad:
            for key in ('geom_a', 'geom_b'):
                g = self.scene.model.geom(b[key]).id
                if g in self.robot_index:
                    v[self.robot_index[g]] = max(v[self.robot_index[g]], -b['distance_m'])
        return v


def motion(p, x, anchor):
    a = p.feature.frames(np.asarray(anchor)[4:11])['palm'][0]
    b = p.feature.frames(np.asarray(x)[4:11])['palm'][0]
    ca = transform(p.feature.palm_point_local, a)
    cb = transform(p.feature.palm_point_local, b)
    return dict(centroid_displacement_mm=float(np.linalg.norm(cb-ca)*1000),
        root_displacement_mm=float(np.linalg.norm(b[:3, 3]-a[:3, 3])*1000),
        rotation_deg=float(np.degrees(Rotation.from_matrix(b[:3, :3]@a[:3, :3].T).magnitude())))


def motion_residual(p, x, anchor):
    m = motion(p, x, anchor)
    return np.array([max(0., m['centroid_displacement_mm']-19.95)/1000,
        max(0., m['rotation_deg']-14.95)*.001])*3000


def soft_residual(p, x, anchor=None):
    # Reuse S sampled surface/table/post residuals, with contact attraction 500x
    # weaker. Natural residual is zero throughout the permitted interior.
    r = p.residual(x, anchor, 'none')
    r[:6] *= .002
    r[6:12] *= .02
    # The inherited wrist residual is divided by ten (0.00005 * angle/180).
    # It precedes the len(x)-4 regularizer at the end of this residual.
    r[-(len(x)-4)-1] *= .1
    return r


def cloud_residual(p, x, goals):
    pp = np.linalg.inv(p.fixed_pad)@p.feature.frames(x[4:11])['palm'][0]
    residuals = []
    for g in goals:
        f = p.fingers[g['finger']]
        targets = (np.array([s['point'] for s in g['states']])-pp[:3, 3])@pp[:3, :3]
        normals = -np.array([s['normal'] for s in g['states']])@pp[:3, :3]
        if g['name'] in ('LS', 'RS', 'LT', 'RT'):
            _, ids = f.position_tree.query(targets)
        else:
            _, ids = f.tree.query(np.c_[targets, .015*normals])
        errors = targets-f.xyz[ids]
        if g.get('alternatives'):
            errors = errors[np.argmin(np.linalg.norm(errors, axis=1))][None]
        # Equal control weighting, independent of the eight stick endpoints.
        residuals.extend((errors*30/np.sqrt(len(errors))).ravel())
    return np.asarray(residuals)


def solve_base(p, native, start, goals, nfev=55, exact=False):
    x0 = np.array(start)
    def fun(z):
        tick()
        x = np.r_[p.fixed4, z]
        return np.r_[soft_residual(p, x, x0), p.natural_residual(x[4:11])*40,
            cloud_residual(p, x, goals), native.collision_vector(x)*3000 if exact else []]
    fit = least_squares(fun, np.clip(x0[4:], p.lower[4:]+1e-8, p.upper[4:]-1e-8),
        bounds=(p.lower[4:], p.upper[4:]), x_scale=.4, diff_step=1e-4,
        max_nfev=nfev, ftol=1e-7, xtol=1e-7, gtol=1e-7)
    return np.r_[p.fixed4, fit.x], dict(nfev=fit.nfev, cost=fit.cost, native_residual=exact)


def state_fit(p, native, base, goal, si, flexible, nfev=45, starts=2):
    """Joint arm/all-hand IK. Strict arm is excluded from decision variables."""
    state = goal['states'][si]
    finger = p.fingers[goal['finger']]
    x0 = np.array(base)
    pp = np.linalg.inv(p.fixed_pad)@p.feature.frames(x0[4:11])['palm'][0]
    target = (np.array(state['point'])-pp[:3, 3])@pp[:3, :3]
    inward = -np.array(state['normal'])@pp[:3, :3]
    cone = goal['name'] not in ('LS', 'RS', 'LT', 'RT')
    count = len(finger.q)
    _, near = finger.tree.query(np.r_[target, .015*inward], k=3) if cone else finger.position_tree.query(target, k=3)
    modes = list(dict.fromkeys(int(i)//count for i in np.atleast_1d(near)))
    modes += [m for m in range(len(finger.points)) if m not in modes]
    # Different surface modes are real K0 contact choices, not retrying bugs.
    diags = []
    best = None
    start_index = 4 if flexible else 11
    for mode in modes[:starts]:
        tick()
        section = slice(mode*count, (mode+1)*count)
        distances = np.linalg.norm(finger.xyz[section]-target, axis=1)
        if cone:
            distances += .015*np.maximum(0., np.cos(np.pi/6)-finger.nrm[section]@inward)
        fq = finger.q[int(np.argmin(distances))]
        seed = x0.copy()
        ids = np.array([11+p.names.index(n) for n in finger.chain.active])
        seed[ids] = fq
        def residual(z):
            tick()
            x = seed.copy(); x[start_index:] = z
            pad, palm, qa, hand, _ = p.unpack(x)
            frame = np.linalg.inv(pad)@palm@finger.chain.forward(x[ids][None])[0]
            point = transform(finger.points[mode], frame)
            normal = frame[:3, :3]@finger.normals[mode]
            # Aim 45 um inside the original surface; original point is checked.
            error = point-(np.array(state['point'])-.000045*np.array(state['normal']))
            c = max(0., np.cos(np.deg2rad(29.8))-normal@(-np.array(state['normal']))) if cone else 0.
            # No sampled contact attraction here: only weak finger anchoring and
            # exact native collision penalty. Every hand joint may move.
            return np.r_[error*1000, c*15, p.natural_residual(qa)*80,
                native.collision_vector(x, goal, state)*5000,
                motion_residual(p, x, x0) if flexible is True else [],
                (x[11:]-x0[11:])*.00001,
                p.feature.measure(qa, p.world_centre)['wrist_deviation_deg']*.00000028]
        fit = least_squares(residual, np.clip(seed[start_index:], p.lower[start_index:]+1e-8,
            p.upper[start_index:]-1e-8), bounds=(p.lower[start_index:], p.upper[start_index:]),
            x_scale=.35, diff_step=1e-4, max_nfev=nfev, ftol=1e-8, xtol=1e-8, gtol=1e-8)
        x = seed.copy(); x[start_index:] = fit.x
        c = native.evaluate(x, gamepad_q=state['gamepad_q'], goal=goal, state=state, solution=dict(mode=mode))
        m = motion(p, x, x0)
        if flexible is True and (m['centroid_displacement_mm']>20.+1e-7 or m['rotation_deg']>15.+1e-7):
            c['reasons'].append('palm_adjustment_limit');c['passed']=False
        row = dict(state=si, x=x.tolist(), mode=mode, check=c, motion=m, nfev=fit.nfev,
            gamepad_q=state['gamepad_q'])
        diags.append(dict(reasons=c['reasons'], point_error_mm=c['active_contact']['point_error_m']*1000,
            mode=mode, nfev=fit.nfev, motion=m))
        rank = (not c['passed'], len(c['violations']), c['active_contact']['point_error_m'])
        if best is None or rank < best[0]:best=(rank,row)
        if c['passed']:return row,diags
    return best[1],diags


def controls(p, native, x, flexible=False, strict=None, refine=True):
    out = dict(controls=[], states=[], diagnostics={})
    pp = np.linalg.inv(p.fixed_pad)@p.feature.frames(np.asarray(x)[4:11])['palm'][0]
    radial = reach_diagnostics(p, pp)
    for goal in p.goals:
        tick()
        name = goal['name']
        if strict and name in strict['controls']:
            out['controls'].append(name)
            out['states'] += [dict(s, motion=dict(centroid_displacement_mm=0.,root_displacement_mm=0.,rotation_deg=0.))
                for s in strict['states'] if s['control']==name]
            out['diagnostics'][name] = dict(passed=True,inherited_strict=True)
            continue
        if not flexible and radial[name]['radially_certified_unreachable']:
            out['diagnostics'][name]=dict(passed=False,reason='fixed_palm_chain_bound',radial=radial[name]);continue
        # Enlarge about the actual mesh centroid (the displacement constraint).
        # No cloud distance is used as an impossibility certificate.
        d = radial[name]
        allowance = .020+2*np.linalg.norm(np.array(d['finger_base_in_palm_m'])-p.feature.palm_point_local)*np.sin(np.deg2rad(7.5))
        if flexible and d['radial_point_error_lower_bound_m']-allowance>.00025:
            out['diagnostics'][name]=dict(passed=False,reason='adjustment_enlarged_chain_bound',
                enlarged_lower_bound_mm=1000*(d['radial_point_error_lower_bound_m']-allowance),radial=d);continue
        fits = fit_goal_modes(p.fingers[goal['finger']],goal,pp)
        successes=[];sd=[]
        for proposals in fits['states']:
            si=proposals['state'];state=goal['states'][si];winner=None;rejections=Counter()
            for solution in proposals['solutions']:
                z=np.array(x);z[11+np.arange(len(p.names))[p.slices[goal['finger']]]]=solution['q']
                c=native.evaluate(z,gamepad_q=state['gamepad_q'],goal=goal,state=state,solution=solution)
                if c['passed']:
                    winner=dict(state=si,x=z.tolist(),mode=solution['mode'],check=c,gamepad_q=state['gamepad_q'],
                        motion=dict(centroid_displacement_mm=0.,root_displacement_mm=0.,rotation_deg=0.));break
                rejections.update(c['reasons'])
            attempts=[]
            if winner is None and refine:
                row,attempts=state_fit(p,native,x,goal,si,flexible)
                if row['check']['passed']:winner=row
            sd.append(dict(state=si,passed=winner is not None,point_solutions=len(proposals['solutions']),
                native_rejections=dict(rejections),joint_attempts=attempts))
            if winner:
                winner['control']=name;successes.append(winner)
                if goal.get('alternatives'):break
            elif not goal.get('alternatives') and name in ('LS','RS'):
                # Failure of one required endpoint already rejects the control.
                break
        passed=bool(successes) if goal.get('alternatives') else len(successes)==len(goal['states'])
        if passed:out['controls'].append(name)
        out['states']+=successes
        out['diagnostics'][name]=dict(passed=passed,states=sd)
        print(json.dumps(dict(event='control',side=p.side,name=name,flexible=flexible,passed=passed)),flush=True)
    out['radial']=radial
    return out


def run(args):
    global DEADLINE
    started=time.time();DEADLINE=min(started+args.seconds,args.deadline)
    output=args.output/args.point/args.side;output.mkdir(parents=True,exist_ok=True)
    rank=args.rank
    p=ZProblem(args.source/'final'/f'rank-{rank}'/'task.json',args.side)
    p.natural_weight=40.
    native=ZNative(p,output/'model')
    source=json.loads((args.source/'controls'/f'{args.point}-{args.side}.json').read_text())
    sources=source.get('checked_candidates',[])
    print('SOURCE KEYS',list(source),flush=True)
    if not sources:
        result=json.loads((args.source/'results'/f'{args.point}-{args.side}.json').read_text())
        sources=result['attempts']
    candidates=[]
    def record(x,label,info=None,refine=False):
        tick();check=native.evaluate(x)
        row=dict(x=np.asarray(x).tolist(),label=label,check=check,optimizer=info)
        if check['passed']:
            row.update(controls(p,native,np.asarray(x),refine=refine))
        else:row['controls']=[]
        candidates.append(row);write_json(output/'candidates-progress.json',candidates)
        print(json.dumps(dict(event='candidate',point=args.point,side=args.side,label=label,
            valid=check['passed'],controls=row['controls'],reasons=check['reasons'],elapsed=time.time()-started)),flush=True)
        return row
    try:
        for i,s in enumerate(sources):
            if 'x' in s:record(np.array(s['x']),f'X-{i}',refine=False)
        # X selected baseline always included, even if controls file differs.
        selected=json.loads((args.source/'final'/f'rank-{rank}'/'candidates.json').read_text())['sides'][args.side]
        if not candidates:record(np.array(selected['x']),'X-selected')
        valid=[r for r in candidates if r['check']['passed']]
        anchor=max(valid,key=lambda r:len(r['controls'])) if valid else candidates[0]
        x0=np.array(anchor['x'])
        # All-controls, thumb controls, and shoulder/trigger subsets. Seed from
        # X and previously joint-optimized palms; no ideal-angle residual.
        subsets=[p.goals,[g for g in p.goals if g['finger']=='thumb'],
            [g for g in p.goals if g['finger']!='thumb']]
        for k,goals in enumerate(subsets):
            x,info=solve_base(p,native,x0,goals,nfev=args.nfev)
            c=native.evaluate(x)
            if not c['passed']:
                x,repair=solve_base(p,native,x,goals,nfev=35,exact=True);info['repair']=repair
            record(x,f'joint-cloud-{k}',info,refine=True)
        valid=[r for r in candidates if r['check']['passed']]
        # Joint arm/finger per-control solutions are further shared-palm seeds.
        # This lets the objective escape a pure shared distance compromise.
        current=max(valid,key=lambda r:len(r['controls']))
        for goal in p.goals:
            if goal['name'] in current['controls']:continue
            row,diag=state_fit(p,native,np.array(current['x']),goal,0,True,nfev=55,starts=3)
            # A valid operation may become a neutral common hold; recheck it.
            if row['check']['passed']:
                z=np.array(row['x'])
                if native.evaluate(z)['passed']:
                    rr=record(z,'joint-operation-'+goal['name'],dict(attempts=diag),refine=True)
                    if len(rr['controls'])>len(current['controls']):current=rr
        valid=[r for r in candidates if r['check']['passed']]
        strict=max(valid,key=lambda r:(len(r['controls']),len(r['check']['grips']),-r['check']['natural']['wrist_deviation_deg']))
        # Refine the selected strict candidate if it came from an unrefined X
        # seed. Other fingers may adjust; arm/palm are strictly unchanged.
        refined=controls(p,native,np.array(strict['x']),refine=True)
        strict.update(refined)
        write_json(output/'strict.json',strict)
        practical=[]
        alternatives=sorted(valid,key=lambda r:(-len(r['controls']),r['check']['natural']['wrist_deviation_deg']))[:2]
        if strict not in alternatives:alternatives.insert(0,strict)
        for base in alternatives[:2]:
            result=controls(p,native,np.array(base['x']),flexible=True,strict=base,refine=True)
            practical.append(dict(base_label=base['label'],x=base['x'],check=base['check'],**result))
            write_json(output/'practical-progress.json',practical)
        realistic=max(practical,key=lambda r:(len(r['controls']),len(r['check']['grips']),-r['check']['natural']['wrist_deviation_deg']))
        result=dict(point=args.point,side=args.side,strict=strict,realistic=realistic,
            candidates=len(candidates),valid_candidates=len(valid),completed=True,
            elapsed_seconds=time.time()-started,deadline_epoch=DEADLINE)
    except TimeoutError as e:
        result=dict(point=args.point,side=args.side,completed=False,error=str(e),
            elapsed_seconds=time.time()-started,deadline_epoch=DEADLINE,candidates=len(candidates))
        valid=[r for r in candidates if r['check']['passed']]
        if valid:result['strict']=max(valid,key=lambda r:len(r['controls']))
        if (output/'strict.json').exists():result['strict']=json.loads((output/'strict.json').read_text())
        if (output/'practical-progress.json').exists():
            result['realistic']=max(json.loads((output/'practical-progress.json').read_text()),key=lambda r:len(r['controls']))
    write_json(output/'result.json',result)
    print(json.dumps(dict(event='done',point=args.point,side=args.side,completed=result['completed'],
        strict=result.get('strict',{}).get('controls'),realistic=result.get('realistic',{}).get('controls'),
        elapsed=result['elapsed_seconds'])),flush=True)


if __name__=='__main__':
    a=argparse.ArgumentParser(description=__doc__)
    a.add_argument('--source',type=Path,required=True);a.add_argument('--output',type=Path,required=True)
    a.add_argument('--point',required=True);a.add_argument('--rank',type=int,required=True)
    a.add_argument('--side',choices=('left','right'),required=True)
    a.add_argument('--seconds',type=float,default=2200);a.add_argument('--deadline',type=float,default=float('inf'))
    a.add_argument('--nfev',type=int,default=65)
    run(a.parse_args())
