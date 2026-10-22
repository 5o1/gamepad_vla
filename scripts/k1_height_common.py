"""K1-a geometry helpers. K0 acceptance constants and native contacts are retained."""
from __future__ import annotations

from collections import Counter
from dataclasses import replace
from pathlib import Path
import json

import numpy as np
import trimesh

from gamepad_vla.feasibility import FINGERS, mesh_for_geom, transform
from gamepad_vla.workstation.spec import FixtureSpec, pose_to_matrix, matrix_to_pose

SIDES = ('left', 'right')
SUPPORT_NAME = 'gamepad_central_support'
EXCLUDED = {'View', 'Menu', 'Xbox'}
TABLE_GAP = .030
BORDER = 40.
MIN_DISTANCE = .30


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')


def support_point():
    path = Path('docs/reports/2026-10-21-two-hand-feasibility/fixture_envelopes.json')
    return np.asarray(json.loads(path.read_text())['support_point_m'])


def support_fixture(spec, root_pose, support, *, width=.020, depth=.024):
    """A vertical column in the pad's x/y axes; its top reaches the K0 shell point."""
    world = transform([0.,0.,support[2]], pose_to_matrix(root_pose))
    top = spec.table.pose[2]+spec.table.half_extents[2]
    height = float(world[2]-top)
    return FixtureSpec(SUPPORT_NAME, 'box',
        (*world[:2], top+height/2, *root_pose[3:]), True, 'design',
        'K1-a; K0 central rear-shell sample; 20x24mm full-height thin column; '
        'fixed root, no structural or clamp-load validation.',
        (width/2, depth/2, height/2))


def with_support(spec, root_pose, support):
    return replace(spec, gamepad=replace(spec.gamepad, root_pose=tuple(root_pose)),
                   fixtures=(*spec.fixtures, support_fixture(spec, root_pose, support)))


def set_placement(scene, root_pose, support):
    f = support_fixture(scene.spec, root_pose, support)
    geom = scene.model.geom(SUPPORT_NAME)
    mover = scene.model.body(SUPPORT_NAME+'_mover').mocapid[0]
    scene.data.mocap_pos[mover] = f.pose[:3]
    scene.data.mocap_quat[mover] = f.pose[3:]
    # Compile at the maximum height; only shrink narrow-phase box size.
    # The compiled local BVH/AABB/rbound stay conservative. The mocap frame
    # carries translations so no stale world-body BVH is used during the scan.
    if np.any(np.asarray(f.half_extents) > scene.model.geom_aabb[geom.id,3:]+1e-8):
        raise ValueError('support exceeds compiled conservative bounds')
    scene.model.geom_size[geom.id] = f.half_extents
    scene.set_pad(root_pose, {})


def relative_check(scene, side, part=None, finger=None):
    """K0 native hand/pad acceptance, with world-dependent pairs deferred."""
    scene.mj.mj_forward(scene.model, scene.data)
    hand = 'HL' if side == 'left' else 'HR'
    bad = []; grips = set(); active = False
    for c in scene.data.contact:
        a,b = map(int,c.geom); ga,gb = scene.groups[a],scene.groups[b]
        if {ga,gb} != {hand,'pad'}:
            continue
        ba = scene.model.body(scene.model.geom_bodyid[a]).name
        bb = scene.model.body(scene.model.geom_bodyid[b]).name
        hb,pb = (ba,bb) if ga == hand else (bb,ba)
        depth = float(c.dist)
        grip = next((f for f in ('ring','pinky') if hb.endswith(f+'_distal')),None)
        if grip and pb.endswith('controller_body'):
            grips.add(grip)
            if depth >= -.001:
                continue
        if part and finger and hb.endswith(finger+'_distal') and pb.endswith(part):
            if -.0001 <= depth <= 0:
                active = True
                continue
        if depth < -1e-6:
            bad.append(dict(a=ba,b=bb,geom_a=scene.model.geom(a).name,
                            geom_b=scene.model.geom(b).name,distance_m=depth))
    return not bad, bad, grips, active


def reject_reason(bad, grips, active=True):
    if bad:
        names = ' '.join(str(x.get(k,'')) for x in bad for k in ('a','b','geom_a','geom_b'))
        if SUPPORT_NAME in names: return 'support_collision'
        if 'table' in names: return 'table_collision'
        if 'column' in names: return 'station_column_collision'
        if any('AL_' in str(x.get('a','')) or 'AR_' in str(x.get('a','')) or
               'AL_' in str(x.get('b','')) or 'AR_' in str(x.get('b','')) for x in bad):
            return 'arm_or_cross_robot_collision'
        return 'hand_pad_or_other_collision'
    if grips != {'ring','pinky'}: return 'native_grip_missing'
    if not active: return 'native_active_contact_missing'
    return 'passed'


class MeshGeometry:
    """Original compiled triangle vertices, never convex-hull or AABB substitutes."""
    def __init__(self, scene):
        self.scene = scene
        self.meshes = {}
        for g in range(scene.model.ngeom):
            mid = int(scene.model.geom_dataid[g])
            if mid < 0:
                continue
            if int(scene.model.geom_type[g]) not in (int(scene.mj.mjtGeom.mjGEOM_MESH),
                                                      int(scene.mj.mjtGeom.mjGEOM_SDF)):
                continue
            m = scene.model
            va,vn = m.mesh_vertadr[mid],m.mesh_vertnum[mid]
            fa,fn = m.mesh_faceadr[mid],m.mesh_facenum[mid]
            vertices = np.array(m.mesh_vert[va:va+vn], dtype=float)
            faces = np.array(m.mesh_face[fa:fa+fn])
            mesh = trimesh.Trimesh(vertices,faces,process=False)
            self.meshes[g] = (vertices, faces, float(mesh.area), np.array(mesh.centroid))
        distance_groups = list(scene.groups)
        for g in self.meshes:
            mid = int(scene.model.geom_dataid[g])
            name = scene.model.mesh(mid).name
            distance_groups[g] = next((k for k in ('HL','HR','AL','AR') if name.startswith(k+'_')), distance_groups[g])
        self.groups = {name:[g for g in self.meshes if distance_groups[g] == name]
                       for name in ('HL','HR','AL','AR','pad')}

    def vertices(self, g):
        d = self.scene.data
        return self.meshes[g][0] @ d.geom_xmat[g].reshape(3,3).T + d.geom_xpos[g]

    def points(self, group):
        return np.concatenate([self.vertices(g) for g in self.groups[group]])

    def centroid(self, group):
        d = self.scene.data
        areas = [self.meshes[g][2] for g in self.groups[group]]
        centres = [self.meshes[g][3] @ d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g]
                   for g in self.groups[group]]
        return np.average(centres,axis=0,weights=areas)

    def table_clearance(self, side):
        letter = 'L' if side == 'left' else 'R'
        top = self.scene.spec.table.pose[2]+self.scene.spec.table.half_extents[2]
        return {kind:float(min(self.vertices(g)[:,2].min() for g in self.groups[kind+letter])-top)
                for kind in ('H','A')}

    def fingertips(self, side):
        """Mean of the original distal tip cap (local link +z, within 0.15 mm)."""
        letter = 'L' if side == 'left' else 'R'
        out = {}
        m,d = self.scene.model,self.scene.data
        for f in FINGERS:
            body = m.body('H'+letter+'_'+f+'_distal').id
            geoms = [g for g in self.groups['H'+letter] if m.geom_bodyid[g] == body]
            world = np.concatenate([self.vertices(g) for g in geoms])
            local = (world-d.xpos[body]) @ d.xmat[body].reshape(3,3)
            cap = local[:,2] >= local[:,2].max()-.00015
            out[f] = world[cap].mean(axis=0)
        return out

    def palm_root(self, side):
        letter = 'L' if side == 'left' else 'R'
        return self.scene.data.xpos[self.scene.model.body('H'+letter+'_hand_base_link').id].copy()


class CameraGate:
    def __init__(self, camera, geometry):
        self.camera,self.geometry = camera,geometry
        self.frame = pose_to_matrix(camera.pose_optical)
        self.k = camera.intrinsics
        self._pad_cache = {}

    def project(self, world):
        xyz = (np.asarray(world)-self.frame[:3,3]) @ self.frame[:3,:3]
        return np.c_[self.k.fx*xyz[:,0]/xyz[:,2]+self.k.cx,
                     self.k.fy*xyz[:,1]/xyz[:,2]+self.k.cy],xyz[:,2]

    def measure(self, sides=SIDES):
        g = self.geometry
        scene = g.scene
        padq = tuple(float(scene.data.qpos[scene.model.jnt_qposadr[j]]) for j in range(scene.model.njnt)
                     if scene.model.joint(j).name.startswith(scene.pad_prefix))
        key = (*scene.data.xpos[scene.padroot],*scene.data.xmat[scene.padroot],*padq)
        if key not in self._pad_cache:
            uv,z = self.project(g.points('pad'))
            pm = float(np.min(np.c_[uv[:,0],self.k.width-1-uv[:,0],uv[:,1],self.k.height-1-uv[:,1]]))
            self._pad_cache[key] = (pm,bool((z>0).all()),float(np.linalg.norm(g.centroid('pad')-self.frame[:3,3])))
        pm,positive,pdistance = self._pad_cache[key]
        margins = {'gamepad_mesh':pm};distances = {'gamepad':pdistance}
        for side in sides:
            tips = g.fingertips(side)
            for label,p in [(side+'_fingertips',np.array(list(tips.values()))),
                            (side+'_palm_root',g.palm_root(side)[None])]:
                uv,z = self.project(p)
                positive = positive and bool((z>0).all())
                margins[label] = float(np.min(np.c_[uv[:,0],self.k.width-1-uv[:,0],uv[:,1],self.k.height-1-uv[:,1]]))
            letter = 'L' if side == 'left' else 'R'
            distances[side] = float(np.linalg.norm(g.centroid('H'+letter)-self.frame[:3,3]))
        margin = min(margins.values());reasons = []
        if not positive: reasons.append('behind_camera')
        if margin < BORDER: reasons.append('camera_border')
        if min(distances.values()) < MIN_DISTANCE: reasons.append('camera_distance')
        return dict(passed=not reasons,reasons=reasons,margin_px=margin,
                    group_margins_px=margins,centroid_distances_m=distances)


def defer_pad_contacts(scene):
    """Native hand/pad contacts were proved in the relative bank; don't recompute.

    Whole-world final replay uses a fresh scene without this optimization.
    """
    for g,group in enumerate(scene.groups):
        if group=='pad':
            scene.model.geom_contype[g]=0;scene.model.geom_conaffinity[g]=0
    for b in range(scene.model.nbody):
        if scene.model.body(b).name.startswith(scene.pad_prefix):
            scene.model.body_contype[b]=0;scene.model.body_conaffinity[b]=0


def placements(spec, shell_low, heights=range(60,161,10)):
    camera = next(c for c in spec.cameras if c.name == 'camera_3')
    frame = pose_to_matrix(camera.pose_optical)
    axis = frame[:3,2]
    top = spec.table.pose[2]+spec.table.half_extents[2]
    out = []
    for height in heights:
        z = top+height/1000-shell_low
        centre = frame[:3,3]+axis*((z-frame[2,3])/axis[2])
        for dx in range(-60,61,20):
            for dy in range(-60,61,20):
                pose = list(spec.gamepad.root_pose)
                pose[:3] = [centre[0]+dx/1000,centre[1]+dy/1000,z]
                out.append(dict(name=f'h{height:03d}_x{dx:+03d}_y{dy:+03d}',
                    height_mm=height,offset_mm=[dx,dy],pose=pose,
                    optical_axis_plane_intersection_m=centre.tolist()))
    return out
