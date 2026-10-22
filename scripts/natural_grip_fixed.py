#!/usr/bin/env python3
"""Task S: fixed-scene natural-arm multi-start proposals and K0 native replay.

No placement variables, hardware, dynamics, training or dependency installation.
The workcell is read-only; candidates are separate files. Run on the compute node.
"""
from __future__ import annotations

import argparse
from collections import Counter
from concurrent.futures import ProcessPoolExecutor, as_completed
import json
import multiprocessing
import os
from pathlib import Path
import time

import numpy as np
from scipy.optimize import least_squares

from gamepad_vla.feasibility import transform
from gamepad_vla.workstation.spec import pose_to_matrix, matrix_to_pose
from k1b_search import Problem
from k1b_native import Native
from k1b_contact_fit import fit_goal_modes
from k1_natural_reference import solve_reference
from k1_height_common import write_json


POST = 'gamepad_support_post'
PROBLEMS = {}
NATIVES = {}
OUTPUT = None
DEADLINE = None


class FixedProblem(Problem):
    def __init__(self, task, side):
        super().__init__(task, side)
        self.fixed_pad = pose_to_matrix(self.spec.gamepad.root_pose)
        self.world_centre = transform(self.pad_centre, self.fixed_pad)
        self.post = next(f for f in self.spec.fixtures if f.name == POST)
        self.post_frame = pose_to_matrix(self.post.pose)
        # First four entries remain bookkeeping for the reused K1-b interface.
        # They are never optimized; unpack ignores them entirely.
        low = transform(np.c_[np.zeros(len(self.shell_yz)), self.shell_yz], self.fixed_pad)[:, 2].min()
        tilt = np.arctan2(self.fixed_pad[2, 1], self.fixed_pad[2, 2])
        self.fixed4 = np.r_[self.fixed_pad[:2, 3], low-self.top, tilt]
        self.thumb_base = self._finger_base('thumb')
        self.ring_base = self._finger_base('ring')
        self.human_palm_target_local = np.array([
            (-1 if side == 'left' else 1)*.055, -.03, -.045])
        self.palm_preference = 0.
        self.reach_scale = 1.
        self.natural_weight = 1.
        self.feasibility_nfev = 0

    def _finger_base(self, finger):
        chain = self.fingers[finger].chain
        frames = chain.link_poses(np.zeros((1, len(chain.active))))
        return frames[chain.joints[0]['child']][0, :3, 3]

    def unpack(self, x):
        qarm = np.asarray(x)[4:11]
        palm = self.feature.frames(qarm)['palm'][0]
        qdict = dict(zip(self.names, np.asarray(x)[11:]))
        return self.fixed_pad, palm, qarm, qdict, self.hand_surface.hand_frames(qdict)

    def natural_residual(self, q):
        m = self.feature.measure(q, self.world_centre)
        up, pitch, heading = [m[k][0] for k in
            ('upper_tilt_deg', 'forearm_pitch_deg', 'forearm_heading_deg')]
        return np.array([max(0., 10.15-up), max(0., up-34.85),
                         max(0., -9.85-pitch), max(0., pitch-24.85),
                         max(0., heading-29.85),
                         max(0., -(m['elbow']-m['shoulder'])[0, 0])*100])*.025

    def residual(self, x, regularize=None, reach='none'):
        pad, palm, qarm, qdict, frames = self.unpack(x)
        inv = np.linalg.inv(pad)
        pp = inv @ palm
        points = []
        normals = []
        for name in ('ring', 'pinky'):
            f = self.fingers[name]
            fr = frames[name+'_distal']
            points.append(transform(transform(f.points[0], fr), pp))
            normals.append(pp[:3, :3] @ fr[:3, :3] @ f.normals[0])
        gp = np.array(points)
        _, ids = self.grip_tree.query(gp)
        target = self.grip_p[ids]-.00012*self.grip_n[ids]
        contact = (gp-target).ravel()*90
        normal = (np.array(normals)+self.grip_n[ids]).ravel()*.001
        world, labels, centre = self.hand_surface.points(frames, palm)
        lp = transform(world, inv)
        distance, near = self.object_tree.query(lp)
        signed = np.einsum('ij,ij->i', lp-self.object_p[near], self.object_n[near])
        allowance = np.array([.0006 if n in ('ring_distal', 'pinky_distal') else -.001 for n in labels])
        penetration = np.where(distance < .022, np.maximum(0., -signed-allowance), 0.)*8
        armpts = self.arm_surface.arm_points(qarm)
        # Physical table collisions are hard in native replay. The 5 mm margin
        # here is a search preference, not an added Task S acceptance gate.
        table = np.maximum(0., self.top+.005-np.r_[world[:, 2], armpts[:, 2]])*8
        robotpts = np.r_[world, armpts]
        cp = transform(robotpts, np.linalg.inv(self.post_frame))
        delta = np.abs(cp)-np.array(self.post.half_extents)
        sd = np.linalg.norm(np.maximum(delta, 0.), axis=1)+np.minimum(np.max(delta, axis=1), 0.)
        column = np.maximum(0., .001-sd)*8
        natural = self.natural_residual(qarm)*self.natural_weight
        m = self.feature.measure(qarm, self.world_centre)
        # Human-style seed/preference: thumb side toward controller centre;
        # grip fingers curl from below. Native contacts alone decide acceptance.
        sign = 1 if self.side == 'left' else -1
        tb, rb = transform(np.array([self.thumb_base, self.ring_base]), pp)
        human = np.array([max(0., .012-sign*(tb[0]-rb[0]))])*1.5
        palm_centre = palm[:3, :3]@self.feature.palm_point_local+palm[:3, 3]
        palm_preference = (palm_centre-transform(self.human_palm_target_local, pad))*self.palm_preference
        soft = np.array([m['wrist_deviation_deg'][0]/180*.0005])
        reg = np.zeros(len(x)-4) if regularize is None else (np.asarray(x)[4:]-regularize[4:])*.00002
        reach_values = []
        for goal in self.goals:
            if reach == 'none' or (reach == 'thumb' and goal['finger'] != 'thumb'):
                continue
            f = self.fingers[goal['finger']]
            target = (np.array([s['point'] for s in goal['states']])-pp[:3, 3]) @ pp[:3, :3]
            outward = -np.array([s['normal'] for s in goal['states']]) @ pp[:3, :3]
            if goal['name'] in ('LS', 'RS', 'LT', 'RT'):
                _, ni = f.position_tree.query(target, workers=1)
            else:
                _, ni = f.tree.query(np.c_[target, .015*outward], workers=1)
            errors = target-f.xyz[ni]
            if goal.get('alternatives'):
                errors = errors[np.argmin(np.linalg.norm(errors, axis=1))][None]
            reach_values.extend(errors.ravel()*.5*self.reach_scale)
        return np.r_[contact, normal, penetration, table, column, natural, human, palm_preference, soft, reg, reach_values]


class FixedNative(Native):
    def __init__(self, problem, work):
        super().__init__(problem, work, final_spec=problem.spec)
        # Explicit distances include fixed roots/palms which MuJoCo's automatic
        # contact table may omit. Use every collidable fixture in the read-only
        # scene, including table frame, station column and actual 18 mm post.
        self.fixture_geoms = [self.scene.model.geom('table').id] + [
            self.scene.model.geom(f.name).id for f in problem.spec.fixtures
            if f.collision and f.type in ('box', 'mesh')]
        self.last_fixture_distances = {}

    def feasibility_residual(self, x, anchor):
        summary = self.p.summary(x)
        point = np.maximum(0., np.array(summary['grip_point_error_m'])-.00020)*1000
        return np.r_[point, self.p.natural_residual(np.asarray(x)[4:11])*10,
                     self.penalty(x)*100, (np.asarray(x)[4:]-np.asarray(anchor)[4:])*.0001]
        self.last_other_arm_home_violations = []

    def place(self, x, hand_override=None, gamepad_q=None):
        pad, palm, qarm, qdict, _ = self.p.unpack(np.asarray(x))
        if hand_override is not None:
            qdict = hand_override
        self.scene.set_arm(self.p.side, qarm)
        self.scene.set_hand(self.p.side, palm, qdict)
        self.scene.set_pad(self.p.spec.gamepad.root_pose, gamepad_q or {})
        return pad, palm, qarm, qdict

    def collisions(self, x, hand_override=None, gamepad_q=None, goal=None):
        frames = self.place(x, hand_override, gamepad_q)
        scene = self.scene
        _, bad, grips, _ = scene.check(self.p.side, goal['part'] if goal else None,
                                      goal['finger'] if goal else None)
        # Separate recordings do not constrain the unused arm to its scene home.
        # Keep those contacts visible without adding that extra parking gate.
        other_arm = 'AR' if self.p.side == 'left' else 'AL'
        self.last_other_arm_home_violations = [b for b in bad if any(
            scene.groups[scene.model.geom(b[key]).id] == other_arm
            for key in ('geom_a', 'geom_b'))]
        bad = [b for b in bad if not any(
            scene.groups[scene.model.geom(b[key]).id] == other_arm
            for key in ('geom_a', 'geom_b'))]
        for c in scene.data.contact:
            g, h = map(int, c.geom)
            if {scene.groups[g], scene.groups[h]} == {self.arm_group, 'pad'} and c.dist < -1e-6:
                bad.append(dict(a=scene.model.body(scene.model.geom_bodyid[g]).name,
                    b=scene.model.body(scene.model.geom_bodyid[h]).name,
                    geom_a=scene.model.geom(g).name, geom_b=scene.model.geom(h).name,
                    distance_m=float(c.dist)))
        self.last_fixture_distances = {}
        for fixture in self.fixture_geoms:
            name = scene.model.geom(fixture).name
            minimum = .1
            for g in self.robot_geoms:
                distance = float(scene.mj.mj_geomDistance(scene.model, scene.data, g, fixture, .1, None))
                minimum = min(minimum, distance)
                if distance < -1e-6:
                    bad.append(dict(a=scene.model.body(scene.model.geom_bodyid[g]).name,
                        b=name, geom_a=scene.model.geom(g).name, geom_b=name, distance_m=distance))
            self.last_fixture_distances[name] = minimum
        support_distance = self.last_fixture_distances[POST]
        return bad, grips, scene.last_active_contact, scene.last_active_distance, support_distance, frames

    def evaluate(self, x, hand_override=None, gamepad_q=None, goal=None, state=None, solution=None):
        check = super().evaluate(x, hand_override, gamepad_q, goal, state, solution)
        # Task S fixes the placement. K1-a's placement-selection camera/30mm
        # margins remain diagnostics; physical collisions remain hard K0 gates.
        auxiliary = {'table_clearance_30mm', 'camera_border', 'camera_distance', 'behind_camera'}
        check['placement_auxiliary_reasons'] = [r for r in check['reasons'] if r in auxiliary]
        check['reasons'] = [r for r in check['reasons'] if r not in auxiliary]
        check['passed'] = not check['reasons']
        f = self.p.feature.frames(check['arm_q'])
        check['natural_points_m'] = {k: f[k][0].tolist() for k in ('shoulder', 'elbow', 'wrist')}
        palm_centre = f['palm'][0, :3, :3]@self.p.feature.palm_point_local+f['palm'][0, :3, 3]
        target = transform(self.p.human_palm_target_local, self.p.fixed_pad)
        check['palm_mesh_centroid_world_m'] = palm_centre.tolist()
        check['palm_mesh_centroid_in_pad_m'] = transform(palm_centre, np.linalg.inv(self.p.fixed_pad)).tolist()
        check['human_seed_palm_target_distance_m'] = float(np.linalg.norm(palm_centre-target))
        check['human_seed_palm_target_local_m'] = self.p.human_palm_target_local.tolist()
        check['native_fixture_distance_lower_bound_m'] = dict(self.last_fixture_distances)
        check['other_arm_home_violations'] = list(self.last_other_arm_home_violations)
        # Diagnostic: minimum real native shell contact depth per grip finger.
        contacts = {}
        for c in self.scene.data.contact:
            g, h = map(int, c.geom)
            if {self.scene.groups[g], self.scene.groups[h]} != {self.hand_group, 'pad'}:
                continue
            hg = g if self.scene.groups[g] == self.hand_group else h
            pg = h if hg == g else g
            hb = self.scene.model.body(self.scene.model.geom_bodyid[hg]).name
            pb = self.scene.model.body(self.scene.model.geom_bodyid[pg]).name
            if pb.endswith('controller_body'):
                for finger in ('ring', 'pinky'):
                    if hb.endswith(finger+'_distal'):
                        contacts[finger] = min(contacts.get(finger, 0.), float(c.dist))
        check['native_grip_contact_depth_m'] = contacts
        return check


def seeds(problem, count, seed, reference_summary=None, style='root'):
    """Recompute natural references with this scene's mounts; no old arm angles."""
    rng = np.random.default_rng(seed)
    lo, hi = problem.feature.chain.active_limits
    refs = []
    diagnostics = [] if reference_summary is None else reference_summary['references']
    if reference_summary is None:
        for tilt, pitch in ((20, 10), (10.5, -9.5), (34.5, -9.5), (10.5, 24.5), (34.5, 24.5)):
            diagnostics.append(solve_reference(problem.feature, problem.world_centre, tilt, pitch))
    refs = [ref['solution']['q'] for ref in diagnostics if ref['solution']]
    if not refs:
        refs = [problem.arm.home_q]
    n = 120000
    cloud = np.array(refs)[rng.integers(len(refs), size=n)].copy()
    cloud[:, :4] += rng.normal(size=(n, 4))*[.25, .35, .4, .35]
    cloud[:, 4:] = rng.uniform(lo[4:], hi[4:], (n, 3))
    cloud[-n//5:] = rng.uniform(lo, hi, (n//5, 7))
    cloud = np.clip(cloud, lo, hi)
    measures = problem.feature.measure(cloud, problem.world_centre)
    cloud = cloud[measures['natural']]
    if not len(cloud):
        raise RuntimeError('No natural seeds for actual fixed-scene mounts')
    frames = problem.feature.frames(cloud)['palm']
    # Approximate desired human palm locations, only for ranking natural seeds.
    local = np.array([(-1 if problem.side == 'left' else 1)*.065, -.035, -.075])
    if style == 'palm':
        local = problem.human_palm_target_local
    desired = transform(local, problem.fixed_pad)
    positions = frames[:, :3, 3]
    if style == 'palm':
        positions = frames[:, :3, :3]@problem.feature.palm_point_local+positions
    position = np.linalg.norm(positions-desired, axis=1)
    # The desired normal points inward from the shell's back toward its face.
    facing = problem.fingers['ring'].normals[0, 0]*frames[:, :3, 0] @ problem.fixed_pad[:3, 2]
    ranking = position + np.maximum(0., .25-facing)*.08
    ids = np.argsort(ranking)[:min(len(cloud), max(600, count*8))]
    pool = []
    for i in range(max(count*12, 1000)):
        ci = ids[i % len(ids)]
        aq = cloud[ci]
        hq = problem.lo + rng.uniform(.1, .6, len(problem.lo))*(problem.hi-problem.lo)
        for name in ('ring', 'pinky'):
            sl = problem.slices[name]
            hq[sl] = np.clip([rng.uniform(-.2, .2), rng.uniform(.3, 1.3), rng.uniform(.4, 1.6)],
                             problem.lo[sl], problem.hi[sl])
        if i % 4 == 0:
            for name in ('thumb', 'index', 'middle'):
                hq[problem.slices[name]] = problem.lo[problem.slices[name]]
        x = np.r_[problem.fixed4, aq, hq]
        # Different wrist/arm states are retained before cost ranking.
        pool.append((float(np.linalg.norm(problem.residual(x))), x))
    pool.sort(key=lambda e: e[0])
    chosen = []
    for cost, x in pool:
        if not chosen or min(np.linalg.norm(x[4:11]-v[4:11]) for v in chosen) > .13:
            chosen.append(x)
        if len(chosen) >= count:
            break
    if len(chosen) < count:
        chosen.extend(x for _, x in pool[:count-len(chosen)])
    return chosen, dict(references=diagnostics, raw_cloud=n, natural_cloud=len(cloud),
                        seed_style=style, seed_position_local_m=local.tolist(),
                        seed=seed, fixed_pose=list(problem.spec.gamepad.root_pose),
                        base_pose=list(problem.arm.base_pose), home_q=list(problem.arm.home_q),
                        nearest_natural_palm_distance_m=float(position.min()))


def solve(item):
    side, index, xx, maxn, repairn, reach = item
    p = PROBLEMS[side]
    start = time.time()
    x = np.asarray(xx)
    def residual(z, native=False):
        if time.time() >= DEADLINE:
            raise TimeoutError('Task S wall clock deadline')
        v = np.r_[p.fixed4, z]
        r = p.residual(v, x, reach)
        return np.r_[r, NATIVES[side].penalty(v)*3] if native else r
    fit = least_squares(residual, x[4:], bounds=(p.lower[4:], p.upper[4:]),
                        x_scale=.5, max_nfev=maxn, diff_step=1e-4,
                        ftol=1e-8, xtol=1e-8, gtol=1e-8)
    xfinal = np.r_[p.fixed4, fit.x]
    if side not in NATIVES:
        NATIVES[side] = FixedNative(p, OUTPUT/'models'/str(os.getpid())/side)
    native = NATIVES[side]
    initial = native.evaluate(xfinal)
    check = initial
    repair = None
    # A bounded native collision polish is useful only near the contact manifold.
    if not check['passed'] and check['natural']['deviation'] < 1.4 and max(check['grip_point_error_m']) < .008:
        repaired = least_squares(lambda z: residual(z, True), xfinal[4:],
            bounds=(p.lower[4:], p.upper[4:]), x_scale=.5, diff_step=1e-4,
            max_nfev=repairn, ftol=1e-8, xtol=1e-8, gtol=1e-8)
        rx = np.r_[p.fixed4, repaired.x]
        rc = native.evaluate(rx)
        # Preserve an already better hard-gate result instead of blindly taking polish.
        def rank(c):
            return (not c['passed'], not c['natural']['natural'], len(c['reasons']),
                    sum(max(0., -b['distance_m']) for b in c['violations']), max(c['grip_point_error_m']))
        if rank(rc) < rank(check):
            xfinal, check = rx, rc
        repair = dict(nfev=repaired.nfev, check=rc, x=rx.tolist())
    feasibility = None
    if p.feasibility_nfev and not check['passed'] and check['natural']['deviation'] < 1.3 and max(check['grip_point_error_m']) < .005:
        anchor = xfinal.copy()
        fit_hard = least_squares(lambda z: native.feasibility_residual(np.r_[p.fixed4, z], anchor),
            xfinal[4:], bounds=(p.lower[4:], p.upper[4:]), x_scale=.5, diff_step=1e-3,
            max_nfev=p.feasibility_nfev, ftol=1e-9, xtol=1e-9, gtol=1e-9)
        hx = np.r_[p.fixed4, fit_hard.x]
        hc = native.evaluate(hx)
        feasibility = dict(x=hx.tolist(), check=hc, nfev=fit_hard.nfev)
        if hc['passed']:
            xfinal, check = hx, hc
    return dict(side=side, seed_index=index, reach_preference=reach, x=xfinal.tolist(),
                nfev=fit.nfev, optimizer_cost=float(fit.cost), initial_native=initial,
                check=check, repair=repair, feasibility_repair=feasibility,
                elapsed_seconds=time.time()-start)


def main():
    global PROBLEMS, OUTPUT, DEADLINE
    a = argparse.ArgumentParser(description=__doc__)
    a.add_argument('--task', type=Path, required=True)
    a.add_argument('--output', type=Path, required=True)
    a.add_argument('--starts', type=int, default=192)
    a.add_argument('--workers', type=int, default=4)
    a.add_argument('--nfev', type=int, default=180)
    a.add_argument('--repair-nfev', type=int, default=50)
    a.add_argument('--deadline', type=float, required=True)
    a.add_argument('--seed', type=int, default=231023)
    a.add_argument('--seed-style', choices=('root', 'palm'), default='root')
    a.add_argument('--reference-seeds', type=Path)
    a.add_argument('--palm-preference', type=float, default=0.)
    a.add_argument('--reach-scale', type=float, default=1.)
    a.add_argument('--natural-weight', type=float, default=1.)
    a.add_argument('--reach-mode', choices=('mixed', 'none', 'thumb', 'all'), default='mixed')
    a.add_argument('--feasibility-nfev', type=int, default=0)
    args = a.parse_args()
    if not 1 <= args.workers <= 64:
        a.error('workers must be 1..64')
    OUTPUT = args.output
    DEADLINE = args.deadline
    OUTPUT.mkdir(parents=True, exist_ok=True)
    started = time.time()
    items = []
    for side in ('left', 'right'):
        p = PROBLEMS[side] = FixedProblem(args.task, side)
        p.palm_preference, p.reach_scale = args.palm_preference, args.reach_scale
        p.natural_weight, p.feasibility_nfev = args.natural_weight, args.feasibility_nfev
        seed_path = OUTPUT/f'seeds-{side}.json'
        if seed_path.exists():
            saved = json.loads(seed_path.read_text())
            ss, summary = [np.array(x) for x in saved['seeds']], saved['summary']
        else:
            previous = None if args.reference_seeds is None else json.loads(
                (args.reference_seeds/f'seeds-{side}.json').read_text())['summary']
            ss, summary = seeds(p, args.starts, args.seed+(side == 'right'), previous, args.seed_style)
            write_json(seed_path, dict(summary=summary, seeds=[x.tolist() for x in ss]))
        print(json.dumps(dict(stage='seeds', side=side, count=len(ss),
                              natural_cloud=summary['natural_cloud'],
                              nearest_palm=summary['nearest_natural_palm_distance_m'])), flush=True)
        items.extend((side, i, x.tolist(), args.nfev, args.repair_nfev,
                      ('none', 'thumb', 'all')[i % 3] if args.reach_mode == 'mixed' else args.reach_mode)
                      for i, x in enumerate(ss[:args.starts]))
    # Interleave sides to ensure both produce results if interrupted by deadline.
    items.sort(key=lambda t: (t[1], t[0]))
    checkpoint = OUTPUT/'checkpoints'
    checkpoint.mkdir(exist_ok=True)
    done = {(r['side'], r['seed_index']) for f in checkpoint.glob('*.json')
            for r in [json.loads(f.read_text())]}
    with ProcessPoolExecutor(max_workers=args.workers,
                             mp_context=multiprocessing.get_context('fork')) as pool:
        futures = [pool.submit(solve, t) for t in items if (t[0], t[1]) not in done]
        completed = 0
        for future in as_completed(futures):
            row = future.result()
            write_json(checkpoint/f"{row['side']}-{row['seed_index']:04d}.json", row)
            completed += 1
            print(json.dumps(dict(stage='native_done', side=row['side'], seed=row['seed_index'],
                completed=completed, passed=row['check']['passed'], reasons=row['check']['reasons'],
                natural=row['check']['natural'], grip_error=row['check']['grip_point_error_m'])), flush=True)
    write_json(OUTPUT/'finished.json', dict(started_epoch=started, ended_epoch=time.time(),
        deadline_epoch=DEADLINE, requested_starts_per_side=args.starts, workers=args.workers,
        nfev=args.nfev, repair_nfev=args.repair_nfev))


if __name__ == '__main__':
    main()
