"""Kinematic check: can the G20 left thumb press all four D-pad directions
from one fixed palm pose, without the hand entering the gamepad?

The palm pose is free (the arm can place it), so the question is whether the
thumb's contact-point workspace, in the palm frame, contains the D-pad press
pattern with the contact normal facing into the pad.

1. Pattern search. For every candidate press direction ``n`` (outward D-pad
   normal, in the palm frame), in-plane rotation and D-pad centre, the four
   press points at ``--press-radius`` are tested at the cap surface and at full
   rocker travel. A point is reachable when its voxel holds a sampled thumb
   configuration whose contact normal is within ``--cone`` of ``-n``.
2. Palm collision. A random subset of pattern-feasible placements, each in all
   four up/down/left/right labellings, is placed against the gamepad collision
   hulls (CoACD, all components except the D-pad) and rejected if any palm
   surface sample lies inside them.
3. Thumb collision. For palm-free placements, every press point must have a
   reaching configuration whose thumb links stay out of the gamepad, allowing
   ``--thumb-tolerance`` of penetration for voxelisation and the D-pad well.
4. Other fingers. For thumb-clear placements, each of index/middle/ring/pinky
   must have at least one of ``--finger-postures`` sampled postures (plus fully
   open) clear of the gamepad, with the same tolerance. Fingers are held still
   while the thumb presses, so one posture per finger serves all press points.

Scope: kinematics only. Contact forces, arm IK, the table and thumb-finger
self-collision are not checked. D-pad dimensions default to the xbox_gamepad_physics
full_controls MJCF (frozen MuJoCo baseline, unchanged through v0.3.1): pivot
(-20.3, -1.8, 11.2) mm, cap top 18.4 mm, cross arm half-length 10.34 mm, rocker
limit 0.1745 rad.

Usage (the compute node Isaac env; needs numpy, scipy, trimesh). Hulls come from the
pinned release copy at workspace://assets/xbox_gamepad_physics_v0.3.1/
resources_inputs/assets/xbox_one_s/collision_coacd/meshes:
    python scripts/check_thumb_dpad_reach.py --urdf <g20_left.urdf> \
        --gamepad-coacd <collision_coacd/meshes> --report <json>
"""

import argparse
import json
import math
from collections import Counter
from pathlib import Path

import numpy as np
import trimesh
from scipy import ndimage
from scipy.spatial import ConvexHull

from gamepad_vla.kinematics import SerialChain

parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
parser.add_argument("--urdf", type=Path, required=True)
parser.add_argument("--gamepad-coacd", type=Path, required=True, help="dir of CoACD OBJ files")
parser.add_argument("--report", type=Path, required=True)
parser.add_argument("--contact-angle", type=float, default=45.0,
                    help="deg from the tip (0) toward the palmar pad (90)")
parser.add_argument("--samples", type=int, default=2_000_000)
parser.add_argument("--voxel", type=float, default=0.001, help="m")
parser.add_argument("--directions", type=int, default=400)
parser.add_argument("--rotations", type=int, default=12, help="in-plane angles over 90 deg")
parser.add_argument("--centre-stride", type=int, default=2, help="voxels between candidate centres")
parser.add_argument("--press-radius", type=float, default=0.007, help="m from D-pad centre")
parser.add_argument("--rocker-limit", type=float, default=0.17453241348266602, help="rad")
parser.add_argument("--dpad-top", type=float, nargs=3, default=(-0.0203, -0.0018, 0.0184),
                    help="cap top centre in the gamepad frame, m")
parser.add_argument("--cone", type=float, default=30.0, help="deg, used for the collision stages")
parser.add_argument("--palm-candidates", type=int, default=20_000)
parser.add_argument("--thumb-candidates", type=int, default=2_000)
parser.add_argument("--thumb-tolerance", type=float, default=0.001, help="m")
parser.add_argument("--finger-postures", type=int, default=64)
parser.add_argument("--seed", type=int, default=0)
parser.add_argument("--examples", type=int, default=5, help="all-clear placements to store in the report")
args = parser.parse_args()

TIP_LINK = "thumb_distal"
CONES_DEG = (30, 45, 60)


def contact_point(chain, mesh_path):
    """Contact point and outward normal in the distal-link frame.

    The normal is tilted ``--contact-angle`` from the link's +z (tip) toward the
    palmar side, i.e. the side the tip moves toward when the thumb flexes (found
    numerically, so per-side axis signs do not matter; the station mapping puts
    0 rad at open, so positive is flexion). The point is the centroid of the
    mesh's support region in that direction, where the surface normal is close
    to it on a convex fingertip.
    """
    vertices = trimesh.load(mesh_path).vertices
    lower, upper = chain.active_limits
    q = (lower + upper) / 2
    flex = q.copy()
    flex[chain.active.index("thumb_mcp")] += 1e-4
    poses = chain.forward(np.stack([q, flex]))
    tip = np.array([0.0, 0.0, vertices[:, 2].max(), 1.0])
    velocity = poses[0, :3, :3].T @ ((poses[1] @ tip)[:3] - (poses[0] @ tip)[:3])
    angle = math.radians(args.contact_angle)
    normal = np.array([math.sin(angle) * np.sign(velocity[0]), 0.0, math.cos(angle)])
    depth = vertices @ normal
    point = vertices[depth >= depth.max() - 0.0005].mean(axis=0)
    return point, normal


def fibonacci_sphere(n):
    i = np.arange(n) + 0.5
    phi = np.arccos(1 - 2 * i / n)
    theta = math.pi * (1 + 5 ** 0.5) * i
    return np.stack([np.cos(theta) * np.sin(phi), np.sin(theta) * np.sin(phi), np.cos(phi)], axis=1)


def in_plane_basis(n):
    helper = np.array([1.0, 0.0, 0.0]) if abs(n[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = np.cross(n, helper)
    u /= np.linalg.norm(u)
    return u, np.cross(n, u)


def dpad_axes(n, psi):
    """D-pad right and up axes in the palm frame; right x up = n."""
    u, v = in_plane_basis(n)
    return math.cos(psi) * u + math.sin(psi) * v, -math.sin(psi) * u + math.cos(psi) * v


class Grid:
    """Boolean voxel grid with point lookup."""

    def __init__(self, occupancy, origin, voxel):
        self.occupancy, self.origin, self.voxel = occupancy, origin, voxel

    def contains(self, points):
        cell = np.floor((points - self.origin) / self.voxel).astype(np.int64)
        inside = np.all((cell >= 0) & (cell < self.occupancy.shape), axis=-1)
        cell = np.clip(cell, 0, np.array(self.occupancy.shape) - 1)
        return inside & self.occupancy[cell[..., 0], cell[..., 1], cell[..., 2]]


def gamepad_grid(coacd_dir, voxel):
    """Solid occupancy of every CoACD hull except the D-pad, in the gamepad frame."""
    hulls = []
    for path in sorted(coacd_dir.glob("*.obj")):
        if path.stem == "dpad":
            continue
        groups, current = {}, None
        for line in path.read_text().splitlines():
            if line.startswith("o "):
                current = line.split()[1]
            elif line.startswith("v "):
                groups.setdefault(current, []).append([float(x) for x in line.split()[1:4]])
        hulls += [np.array(points) for points in groups.values()]
    lo = np.min([h.min(axis=0) for h in hulls], axis=0) - 2 * voxel
    hi = np.max([h.max(axis=0) for h in hulls], axis=0) + 2 * voxel
    shape = np.ceil((hi - lo) / voxel).astype(int)
    occupancy = np.zeros(shape, dtype=bool)
    for points in hulls:
        equations = ConvexHull(points).equations
        a = np.floor((points.min(axis=0) - lo) / voxel).astype(int)
        b = np.ceil((points.max(axis=0) - lo) / voxel).astype(int) + 1
        axes = [lo[k] + (np.arange(a[k], b[k]) + 0.5) * voxel for k in range(3)]
        centres = np.stack(np.meshgrid(*axes, indexing="ij"), axis=-1)
        inside = np.all(centres @ equations[:, :3].T + equations[:, 3] <= 0, axis=-1)
        occupancy[a[0]:b[0], a[1]:b[1], a[2]:b[2]] |= inside
    return Grid(occupancy, lo, voxel), len(hulls)


def surface_points(mesh_path, count, seed):
    mesh = trimesh.load(mesh_path)
    points, _ = trimesh.sample.sample_surface(mesh, count, seed=seed)
    return np.vstack([points, mesh.vertices[::max(1, len(mesh.vertices) // count)]])


def to_gamepad(points, centre, right, up, n, top):
    """Palm-frame points -> gamepad frame, given the D-pad placement in the palm frame."""
    rotation = np.stack([right, up, n], axis=-1)  # columns: gamepad x, y, z in palm frame
    return top + (points - centre[..., None, :]) @ rotation


def main():
    chain = SerialChain(args.urdf, TIP_LINK)
    if chain.root_link != "hand_base_link":
        raise ValueError(f"unexpected root link {chain.root_link}")
    meshes = args.urdf.parent / "meshes"
    point, normal = contact_point(chain, meshes / f"{TIP_LINK}.STL")
    top = np.array(args.dpad_top)

    rng = np.random.default_rng(args.seed)
    lower, upper = chain.active_limits
    configs = (lower + rng.random((args.samples, len(lower))) * (upper - lower)).astype(np.float32)
    positions, normals = [], []
    for start in range(0, args.samples, 200_000):
        pose = chain.forward(configs[start:start + 200_000])
        positions.append(pose[:, :3, :3] @ point + pose[:, :3, 3])
        normals.append(pose[:, :3, :3] @ normal)
    positions, normals = np.concatenate(positions), np.concatenate(normals)

    # Voxelise the contact-point workspace in the palm (hand_base_link) frame.
    origin = positions.min(axis=0) - 2 * args.voxel
    cells = np.floor((positions - origin) / args.voxel).astype(np.int64)
    shape = cells.max(axis=0) + 3
    linear = np.ravel_multi_index(cells.T, shape)
    order = np.argsort(linear, kind="stable")
    occupied, starts = np.unique(linear[order], return_index=True)
    ends = np.append(starts[1:], len(order))
    grid = np.full(int(np.prod(shape)), -1, dtype=np.int64)
    grid[occupied] = np.arange(len(occupied))

    def voxel_of(points):
        cell = np.floor((points - origin) / args.voxel).astype(np.int64)
        inside = np.all((cell >= 0) & (cell < shape), axis=-1)
        index = grid[np.ravel_multi_index(np.moveaxis(np.clip(cell, 0, shape - 1), -1, 0), shape)]
        return np.where(inside, index, -1)

    # best[v, d]: max over samples in voxel v of cos(angle between contact normal and -n_d).
    directions = fibonacci_sphere(args.directions)
    sorted_normals = normals[order].astype(np.float32)
    best = np.empty((len(occupied), len(directions)), dtype=np.float32)
    for d0 in range(0, len(directions), 50):
        align = -(sorted_normals @ directions[d0:d0 + 50].T.astype(np.float32))
        best[:, d0:d0 + 50] = np.maximum.reduceat(align, starts, axis=0)

    occupancy = (grid >= 0).reshape(shape)
    inscribed = ndimage.distance_transform_edt(ndimage.binary_closing(occupancy)) * args.voxel

    centres = np.argwhere(occupancy[::args.centre_stride, ::args.centre_stride, ::args.centre_stride])
    centres = origin + (centres * args.centre_stride + 0.5) * args.voxel
    travel = args.press_radius * math.sin(args.rocker_limit)
    angles = np.arange(args.rotations) * (math.pi / 2) / args.rotations
    cone_cos = math.cos(math.radians(args.cone))

    def press_points(centre, right, up, n):
        """(..., 8, 3): up, down, left, right at the surface and at full travel."""
        arms = np.stack([up, -up, -right, right], axis=-2)
        offsets = args.press_radius * arms[..., :, None, :] - np.array([0.0, travel])[:, None] * n[..., None, None, :]
        return centre[..., None, :] + offsets.reshape(*offsets.shape[:-3], 8, 3)

    feasible = {cone: 0 for cone in CONES_DEG}
    candidates = []  # (direction, centre index, rotation index) passing --cone
    best_pattern = dict(score=-2.0)
    total = 0
    for d, n in enumerate(directions):
        axes = [dpad_axes(n, psi) for psi in angles]
        right = np.array([a[0] for a in axes])[None]  # (1, R, 3)
        up = np.array([a[1] for a in axes])[None]
        targets = press_points(centres[:, None, :], right, up, np.broadcast_to(n, (1, len(angles), 3)))
        index = voxel_of(targets)
        score = np.where(index >= 0, best[np.maximum(index, 0), d], -2.0).min(axis=-1)  # (C, R)
        total += score.size
        for cone in CONES_DEG:
            feasible[cone] += int((score >= math.cos(math.radians(cone))).sum())
        c, r = np.nonzero(score >= cone_cos)
        candidates.append(np.stack([np.full_like(c, d), c, r], axis=1))
        c, r = np.unravel_index(np.argmax(score), score.shape)
        if score[c, r] > best_pattern["score"]:
            best_pattern = dict(score=float(score[c, r]),
                                worst_normal_angle_deg=math.degrees(math.acos(min(1.0, float(score[c, r])))))
    candidates = np.concatenate(candidates)

    # Stage 2: palm (hand_base_link) against the gamepad, in all four labellings.
    gamepad, hull_count = gamepad_grid(args.gamepad_coacd, args.voxel)
    tolerant = Grid(ndimage.binary_erosion(gamepad.occupancy,
                                           iterations=max(1, round(args.thumb_tolerance / args.voxel))),
                    gamepad.origin, gamepad.voxel)
    palm = surface_points(meshes / "hand_base_link.STL", 4000, args.seed)
    pick = rng.choice(len(candidates), min(args.palm_candidates, len(candidates)), replace=False)
    placements = []  # (centre, right, up, n)
    for d, c, r in candidates[pick]:
        for k in range(4):
            right, up = dpad_axes(directions[d], angles[r] + k * math.pi / 2)
            placements.append((centres[c], right, up, directions[d]))
    placement_arrays = [np.array(p) for p in zip(*placements)]
    palm_free = np.zeros(len(placements), dtype=bool)
    for s in range(0, len(placements), 500):
        chunk = [a[s:s + 500] for a in placement_arrays]
        inside = gamepad.contains(to_gamepad(palm[None], *chunk, top))
        palm_free[s:s + 500] = ~inside.any(axis=-1)

    # Stage 3: thumb links, per press point, using the sampled configurations.
    thumb_links = [j["child"] for j in chain.joints]
    thumb_points = {link: surface_points(meshes / f"{link}.STL", 300, args.seed) for link in thumb_links}
    free_index = np.flatnonzero(palm_free)
    free_index = rng.choice(free_index, min(args.thumb_candidates, len(free_index)), replace=False)
    passed = []
    # Why a placement failed: the first failing press point (0-7: up, down, left, right x
    # {surface, full travel}) and, if it was a collision, the links that collide in every
    # reaching configuration.
    failed_point, blocking = Counter(), Counter()
    for i in free_index:
        centre, right, up, n = (a[i] for a in placement_arrays)
        ok_all = True
        for point_index, target in enumerate(press_points(centre, right, up, n)):
            v = int(voxel_of(target))
            samples = order[starts[v]:ends[v]] if v >= 0 else order[:0]
            samples = samples[-(normals[samples] @ n) >= cone_cos]
            if len(samples) == 0:
                ok_all = False
                failed_point[f"{point_index}:no_reach"] += 1
                break
            poses = chain.link_poses(configs[samples])
            collide = np.zeros(len(samples), dtype=bool)
            always = []
            for link, local in thumb_points.items():
                world = np.einsum("nij,pj->npi", poses[link][:, :3, :3], local) + poses[link][:, None, :3, 3]
                hit = tolerant.contains(to_gamepad(world, centre[None], right[None], up[None], n[None],
                                                   top)).any(axis=-1)
                collide |= hit
                if hit.all():
                    always.append(link)
            if collide.all():
                ok_all = False
                failed_point[f"{point_index}:collision"] += 1
                blocking["+".join(always) or "mixed"] += 1
                break
        if ok_all:
            passed.append(i)

    # Stage 4: the other four fingers, each needing one clear posture.
    finger_points, finger_q = {}, {}
    for finger in ("index", "middle", "ring", "pinky"):
        finger_chain = SerialChain(args.urdf, f"{finger}_distal")
        low, high = finger_chain.active_limits
        q = low + rng.random((args.finger_postures, len(low))) * (high - low)
        q = np.vstack([np.clip(0.0, low, high), q])
        finger_q[finger] = (finger_chain.active, q)
        poses = finger_chain.link_poses(q)
        finger_points[finger] = np.concatenate([
            np.einsum("nij,pj->npi", poses[j["child"]][:, :3, :3],
                      surface_points(meshes / f"{j['child']}.STL", 100, args.seed))
            + poses[j["child"]][:, None, :3, 3]
            for j in finger_chain.joints], axis=1)  # (postures, points, 3)
    finger_blocked, examples, clear = Counter(), [], 0
    for i in passed:
        centre, right, up, n = (a[i] for a in placement_arrays)
        hits = {finger: tolerant.contains(to_gamepad(points, centre[None], right[None], up[None], n[None],
                                                     top)).any(axis=-1)
                for finger, points in finger_points.items()}  # (postures,) per finger
        blocked = [finger for finger, hit in hits.items() if hit.all()]
        finger_blocked["+".join(blocked) or "none"] += 1
        if not blocked:
            clear += 1
            if len(examples) < args.examples:
                # First clear posture per finger; index 0 is fully open, so open wins when clear.
                postures = {}
                for finger, hit in hits.items():
                    names, q = finger_q[finger]
                    postures.update(zip(names, q[int(np.argmin(hit))].round(4).tolist()))
                examples.append(dict(dpad_centre_m=centre.round(4).tolist(), dpad_outward_normal=n.round(4).tolist(),
                                     dpad_up_axis=up.round(4).tolist(), dpad_right_axis=right.round(4).tolist(),
                                     clear_finger_q=postures))

    report = dict(
        urdf=str(args.urdf),
        gamepad_coacd=str(args.gamepad_coacd),
        frame="hand_base_link",
        chain=chain.active,
        mimic={j["name"]: j["mimic"] for j in chain.joints if j["mimic"]},
        contact_angle_deg=args.contact_angle,
        contact_point_in_distal_m=point.round(5).tolist(),
        contact_normal_in_distal=normal.tolist(),
        samples=args.samples,
        voxel_m=args.voxel,
        workspace=dict(
            occupied_voxels=int(len(occupied)),
            volume_cm3=float(len(occupied) * args.voxel ** 3 * 1e6),
            bbox_min_m=positions.min(axis=0).round(4).tolist(),
            bbox_max_m=positions.max(axis=0).round(4).tolist(),
            max_inscribed_radius_m=float(inscribed.max()),
        ),
        dpad=dict(top_centre_gamepad_m=list(args.dpad_top), press_radius_m=args.press_radius,
                  rocker_limit_rad=args.rocker_limit, full_travel_at_press_radius_m=travel,
                  points_per_placement="4 directions x {surface, full travel}"),
        pattern_search=dict(directions=args.directions, rotations_over_90deg=args.rotations,
                            centres=int(len(centres)), placements=total,
                            feasible={f"cone_{cone}deg": count for cone, count in feasible.items()},
                            best=best_pattern),
        collision=dict(
            cone_deg=args.cone,
            gamepad_hulls=hull_count,
            gamepad_solid_voxels=int(gamepad.occupancy.sum()),
            # Expected: body centre solid; 5 mm above the cap free; cap top free (D-pad excluded).
            grid_probes=dict(zip(("body_centre", "above_cap_5mm", "cap_top"),
                                 gamepad.contains(np.array([[0.0, 0.0, 0.0], top + [0, 0, 0.005], top])).tolist())),
            palm_tested=len(placements),
            palm_free=int(palm_free.sum()),
            thumb_tested=int(len(free_index)),
            thumb_tolerance_m=args.thumb_tolerance,
            thumb_clear=len(passed),
            first_failed_point=dict(failed_point.most_common()),
            always_colliding_links=dict(blocking.most_common()),
            finger_postures=args.finger_postures + 1,
            fingers_blocked=dict(finger_blocked.most_common()),
            all_clear=clear,
            examples=examples,
        ),
        scope="Kinematics only. Palm, thumb and finger links vs gamepad hulls (D-pad excluded); "
              "contact force, arm IK, table and self-collision not checked; "
              "D-pad dimensions from the MuJoCo baseline.",
    )
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


main()
