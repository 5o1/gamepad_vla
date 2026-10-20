"""Choose the gamepad pose and left-arm home_q for a workcell spec by IK.

For every candidate gamepad pose (fixed lateral position, a few distances from the
table's back edge, yaws about world z) and every all-clear palm placement of a
thumb-reach report, the palm pose follows from the placement; the left arm must
then reach ``palm * mount^-1`` within its joint limits. IK is batched damped
least squares on ``gamepad_vla.kinematics`` FK of the spec's arm URDF, re-rooted
at the spec's (measured) base pose.

Rejected kinematically:
- placements whose open G20 hand (mesh vertices, all fingers at 0) dips below the
  table top plus ``--hand-clearance`` (independent of the gamepad yaw and position);
- arm link mesh vertices below ``table top + --table-clearance``;
- arm link origins inside the column keep-out box. Survivors are ranked by the smallest
joint-limit margin; the best per (distance, yaw) are written to the report, with
the overall choice preferring ``--preferred-yaw`` among near-best margins.

Kinematics only: final contact checks belong in the SAPIEN backend.

Usage (any env with numpy):
    python scripts/place_gamepad_ik.py --spec <v3 draft> --reach-report <report_angle0.json> --output <json>
"""

import argparse
import json
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from gamepad_vla.kinematics import SerialChain, rpy_to_matrix
from gamepad_vla.workstation.spec import load_spec, matrix_to_pose, pose_to_matrix

parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
parser.add_argument("--spec", type=Path, required=True)
parser.add_argument("--reach-report", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--dpad-top", type=float, nargs=3, default=(-0.0203, -0.0018, 0.0184))
parser.add_argument("--gamepad-bottom", type=float, default=0.0299, help="m below controller_body (v0.3.1)")
parser.add_argument("--table-gap", type=float, default=0.001, help="m between gamepad bottom and table top")
parser.add_argument("--lateral-y", type=float, default=None, help="world y of the gamepad (default: table centre)")
parser.add_argument("--from-back", type=float, nargs="+", default=(0.35, 0.38, 0.41),
                    help="m from the table's back edge to the gamepad centre")
parser.add_argument("--yaws", type=float, nargs="+", default=list(range(-180, 180, 15)), help="deg about world z")
parser.add_argument("--preferred-yaw", type=float, default=-90.0,
                    help="deg; -90 puts the bumpers away from the robot, like a player holding it")
parser.add_argument("--table-clearance", type=float, default=0.02, help="m, arm meshes above the table top")
parser.add_argument("--hand-clearance", type=float, default=0.005, help="m, open hand above the table top")
parser.add_argument("--vertices", type=int, default=300, help="mesh vertices kept per link")
parser.add_argument("--column", type=float, nargs=6, default=None,
                    help="column keep-out box xmin xmax ymin ymax zmin zmax (world, m)")
parser.add_argument("--seeds", type=int, default=12)
parser.add_argument("--iterations", type=int, default=150)
parser.add_argument("--max-placements", type=int, default=2000)
args = parser.parse_args()


def rotz(deg):
    c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def log_rotation(r):
    """Batched rotation-vector of ``(N, 3, 3)``."""
    cos = np.clip((np.trace(r, axis1=1, axis2=2) - 1) / 2, -1, 1)
    angle = np.arccos(cos)
    vee = np.stack([r[:, 2, 1] - r[:, 1, 2], r[:, 0, 2] - r[:, 2, 0], r[:, 1, 0] - r[:, 0, 1]], axis=1)
    sin = np.sin(angle)
    scale = np.where(sin > 1e-6, angle / (2 * np.maximum(sin, 1e-12)), 0.5)
    return vee * scale[:, None]


def stl_vertices(path):
    data = Path(path).read_bytes()
    if data[:5] == b"solid" and b"facet" in data[:400]:
        return np.array([[float(v) for v in line.split()[1:4]]
                         for line in data.decode(errors="ignore").splitlines() if line.strip().startswith("vertex")])
    count = struct.unpack("<I", data[80:84])[0]
    tri = np.frombuffer(data[84:84 + 50 * count], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return np.unique(tri["v"].reshape(-1, 3).astype(float), axis=0)


def link_vertices(urdf, links, keep, seed=0):
    """``{link: (k, 3)}`` visual-mesh vertices in each link frame (origin and scale applied)."""
    rng = np.random.default_rng(seed)
    root = ET.parse(urdf).getroot()
    out = {}
    for link in root.findall("link"):
        if link.get("name") not in links:
            continue
        points = []
        for visual in link.findall("visual"):
            mesh = visual.find("geometry/mesh")
            if mesh is None:
                continue
            v = stl_vertices(Path(urdf).parent / mesh.get("filename")) * np.array(
                [float(x) for x in (mesh.get("scale") or "1 1 1").split()])
            origin = visual.find("origin")
            if origin is not None:
                r = rpy_to_matrix([float(x) for x in origin.get("rpy", "0 0 0").split()])
                v = v @ r.T + np.array([float(x) for x in origin.get("xyz", "0 0 0").split()])
            points.append(v)
        if points:
            v = np.concatenate(points)
            out[link.get("name")] = v[rng.choice(len(v), min(keep, len(v)), replace=False)]
    return out


def open_hand_points(urdf, root_link, keep):
    """Vertices of the whole open hand in the hand root frame."""
    points = []
    vertices = link_vertices(urdf, _all_links(urdf), keep)
    if root_link in vertices:
        points.append(vertices[root_link])
    for tip in ("thumb_distal", "index_distal", "middle_distal", "ring_distal", "pinky_distal"):
        chain = SerialChain(urdf, tip)
        poses = chain.link_poses(np.zeros((1, len(chain.active))))
        for link, pose in poses.items():
            if link in vertices:
                points.append(vertices[link] @ pose[0, :3, :3].T + pose[0, :3, 3])
    return np.concatenate(points)


def _all_links(urdf):
    return {link.get("name") for link in ET.parse(urdf).getroot().findall("link")}


def main():
    spec = load_spec(args.spec)
    arm = spec.left_arm
    chain = SerialChain(arm.urdf, arm.tip_link)
    if chain.active != list(arm.joint_names):
        raise ValueError(f"chain joints {chain.active} differ from spec {arm.joint_names}")
    lower, upper = chain.active_limits
    zero_links = chain.link_poses(np.zeros((1, 7)))
    # FK runs in the URDF's design frame; map targets into it from the spec's measured base.
    design_from_world = zero_links[arm.base_link][0] @ np.linalg.inv(pose_to_matrix(arm.base_pose))
    world_from_design = np.linalg.inv(design_from_world)
    mount_inv = np.linalg.inv(pose_to_matrix(spec.left_hand.mount))
    joint_links = [j["child"] for j in chain.joints if j["name"] in chain.active]
    axes = np.array([j["axis"] for j in chain.joints if j["name"] in chain.active])

    report = json.loads(args.reach_report.read_text())
    examples = report["collision"]["examples"][:args.max_placements]
    palm_gamepad = []
    for e in examples:
        m = np.eye(4)
        m[:3, :3] = np.stack([e["dpad_right_axis"], e["dpad_up_axis"], e["dpad_outward_normal"]], axis=1)
        u, _, vt = np.linalg.svd(m[:3, :3])  # rounded axes -> nearest rotation
        m[:3, :3] = u @ vt
        m[:3, 3] = np.array(e["dpad_centre_m"]) - m[:3, :3] @ np.array(args.dpad_top)
        palm_gamepad.append(m)
    gamepad_palm = np.linalg.inv(np.array(palm_gamepad))  # (P, 4, 4)

    # The table plane is z = -(bottom + gap) in the face-up gamepad frame, whatever the yaw.
    hand = open_hand_points(spec.left_hand.urdf, spec.left_hand.root_link, args.vertices)
    hand_low = (np.einsum("pij,nj->pni", gamepad_palm[:, :3, :3], hand)
                + gamepad_palm[:, None, :3, 3])[..., 2].min(axis=1)
    hand_ok = hand_low > -(args.gamepad_bottom + args.table_gap) + args.hand_clearance
    print(f"open hand above the table in {int(hand_ok.sum())}/{len(hand_ok)} placements")

    table = pose_to_matrix(spec.table.pose)
    half = np.array(spec.table.half_extents)
    top = table[2, 3] + half[2]
    back_edge = table[0, 3] - half[0]
    lateral = table[1, 3] if args.lateral_y is None else args.lateral_y
    z = top + args.table_gap + args.gamepad_bottom

    candidates = []  # (from_back, yaw, placement, world_gamepad)
    for from_back in args.from_back:
        for yaw in args.yaws:
            g = np.eye(4)
            g[:3, :3] = rotz(yaw)
            g[:3, 3] = [back_edge + from_back, lateral, z]
            for p in range(len(gamepad_palm)):
                candidates.append((from_back, yaw, p, g))
    gw = np.array([c[3] for c in candidates])
    palm_world = gw @ gamepad_palm[[c[2] for c in candidates]]
    target = design_from_world @ palm_world @ mount_inv  # tcp targets in the design frame
    n = len(target)
    print(f"{len(examples)} placements x {len(args.from_back)} distances x {len(args.yaws)} yaws = {n} targets")

    rng = np.random.default_rng(0)
    best_q = np.zeros((n, 7))
    best_err = np.full(n, np.inf)
    for seed in range(args.seeds):
        q = np.zeros((n, 7)) if seed == 0 else lower + rng.random((n, 7)) * (upper - lower)
        for _ in range(args.iterations):
            links = chain.link_poses(q)
            tip = links[arm.tip_link]
            dp = target[:, :3, 3] - tip[:, :3, 3]
            dr = log_rotation(target[:, :3, :3] @ np.transpose(tip[:, :3, :3], (0, 2, 1)))
            err = np.concatenate([dp, dr], axis=1)
            jac = np.empty((n, 6, 7))
            for i, link in enumerate(joint_links):
                axis = links[link][:, :3, :3] @ axes[i]
                jac[:, :3, i] = np.cross(axis, tip[:, :3, 3] - links[link][:, :3, 3])
                jac[:, 3:, i] = axis
            jjt = jac @ np.transpose(jac, (0, 2, 1)) + 1e-3 * np.eye(6)
            step = np.einsum("nji,nj->ni", jac, np.linalg.solve(jjt, err[..., None])[..., 0])
            q = np.clip(q + np.clip(step, -0.3, 0.3), lower, upper)
        tip = chain.forward(q)
        pos = np.linalg.norm(target[:, :3, 3] - tip[:, :3, 3], axis=1)
        rot = np.linalg.norm(log_rotation(target[:, :3, :3] @ np.transpose(tip[:, :3, :3], (0, 2, 1))), axis=1)
        err = pos + 0.05 * rot
        better = err < best_err
        best_err[better], best_q[better] = err[better], q[better]
        print(f"seed {seed}: solved {int(np.sum(best_err < 1e-3))}/{n}")

    tip = chain.forward(best_q)
    pos = np.linalg.norm(target[:, :3, 3] - tip[:, :3, 3], axis=1)
    rot = np.linalg.norm(log_rotation(target[:, :3, :3] @ np.transpose(tip[:, :3, :3], (0, 2, 1))), axis=1)
    solved = (pos < 5e-4) & (rot < 5e-3)
    links = chain.link_poses(best_q)
    origins = np.stack([(world_from_design @ links[l])[:, :3, 3] for l in joint_links], axis=1)  # (n, 7, 3)
    arm_vertices = link_vertices(arm.urdf, set(joint_links), args.vertices)
    arm_low = np.full(n, np.inf)
    for link, v in arm_vertices.items():
        world = world_from_design @ links[link]
        arm_low = np.minimum(arm_low, (np.einsum("nj,pj->np", world[:, 2, :3], v) + world[:, 2, 3:4]).min(axis=1))
    above_table = arm_low > top + args.table_clearance
    hand_clear = hand_ok[[c[2] for c in candidates]]
    outside_column = np.ones(n, dtype=bool)
    if args.column:
        lo, hi = np.array(args.column[0::2]), np.array(args.column[1::2])
        inside = np.all((origins[:, 1:] > lo) & (origins[:, 1:] < hi), axis=2)
        outside_column = ~inside.any(axis=1)
    margin = np.minimum(best_q - lower, upper - best_q).min(axis=1)
    ok = solved & above_table & outside_column & hand_clear

    groups = {}
    for i, (from_back, yaw, p, _) in enumerate(candidates):
        key = (from_back, yaw)
        g = groups.setdefault(key, dict(from_back_m=from_back, yaw_deg=yaw, targets=0, solved=0, feasible=0,
                                        best=None))
        g["targets"] += 1
        g["solved"] += int(solved[i])
        g["feasible"] += int(ok[i])
        if ok[i] and (g["best"] is None or margin[i] > g["best"]["margin_rad"]):
            g["best"] = dict(index=i, placement=p, margin_rad=float(margin[i]))
    rows = []
    for g in groups.values():
        if g["best"] is not None:
            i = g["best"]["index"]
            g["best"].update(home_q=best_q[i].round(5).tolist(),
                             gamepad_root_pose=[round(v, 6) for v in matrix_to_pose(candidates[i][3])],
                             palm_world_pose=[round(v, 6) for v in matrix_to_pose(palm_world[i])],
                             ik_error=dict(pos_m=float(pos[i]), rot_rad=float(rot[i])),
                             arm_lowest_above_table_m=float(arm_low[i] - top),
                             hand_lowest_above_table_m=float(hand_low[candidates[i][2]]
                                                             + args.gamepad_bottom + args.table_gap),
                             clear_finger_q=examples[candidates[i][2]].get("clear_finger_q"))
        rows.append(g)
    feasible = [g for g in rows if g["best"] is not None]
    if not feasible:
        raise SystemExit("no feasible placement")
    top_margin = max(g["best"]["margin_rad"] for g in feasible)
    # Among placements within 0.1 rad of the best margin, prefer the player-like yaw, then margin.
    near = [g for g in feasible if g["best"]["margin_rad"] >= top_margin - 0.1]
    choice = min(near, key=lambda g: (abs((g["yaw_deg"] - args.preferred_yaw + 180) % 360 - 180),
                                      -g["best"]["margin_rad"]))
    out = dict(spec=str(args.spec), reach_report=str(args.reach_report), placements=len(examples),
               table_top_m=top, back_edge_x_m=back_edge, lateral_y_m=lateral, gamepad_z_m=z,
               table_clearance_m=args.table_clearance, hand_clearance_m=args.hand_clearance,
               placements_hand_clear=int(hand_ok.sum()), column_keepout=args.column,
               joint_limits=dict(lower=lower.tolist(), upper=upper.tolist()),
               solved=int(solved.sum()), feasible=int(ok.sum()), targets=n,
               choice=choice, groups=sorted(rows, key=lambda g: (g["from_back_m"], g["yaw_deg"])),
               scope="Kinematics only: IK, joint limits, link-origin table/column keep-out; no meshes or contacts.")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(out, indent=2))
    print(json.dumps({k: out[k] for k in ("solved", "feasible", "targets", "choice")}, indent=2))


main()
