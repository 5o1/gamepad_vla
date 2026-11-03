"""Reuse SerialChain FK and task Z's axis-line definition of natural posture.

The axis-line feature equations are extracted from k1_natural_reference.py;
that script remains untouched while task Z is running. No mesh-centroid proxy
is needed for the four hard posture bounds used here.
"""
import numpy as np

from gamepad_vla.kinematics import SerialChain
from gamepad_vla.workstation.episode_schema import FINGERS
from gamepad_vla.workstation.g20_mapping import G20_JOINT_NAMES
from gamepad_vla.workstation.spec import matrix_to_pose, pose_to_matrix

# Neutral mesh centroid used by task Z, transformed back from the public
# 2026-10-22-gamepad-height/selected-pair.json placement. It is not the root
# origin. Kept explicit so source/target use the same heading convention without
# needing MuJoCo just to construct a geometric reference point.
NATURAL_HEADING_POINT_M = (-.00037975017865455045, .012404610563098554, .0017678735770016996)


def heading_centre(gamepad_pose, point=NATURAL_HEADING_POINT_M):
    frame = pose_to_matrix(gamepad_pose)
    return frame[:3, :3] @ np.asarray(point)+frame[:3, 3]


def unit(v):
    return v / np.maximum(np.linalg.norm(v, axis=-1, keepdims=True), 1e-12)


def closest_lines(p, a, q, b):
    c = np.sum(a*b, axis=-1)
    d = q-p
    ad, bd = np.sum(a*d, axis=-1), np.sum(b*d, axis=-1)
    den = 1-c*c
    safe = np.maximum(den, 1e-12)
    t = np.where(den > 1e-10, (ad-c*bd)/safe, ad/2)
    u = np.where(den > 1e-10, (c*ad-bd)/safe, -bd/2)
    return p+t[..., None]*a, q+u[..., None]*b


def relative_frames(world_frames, gamepad_pose):
    """T_gamepad_frame = inverse(T_world_gamepad) @ T_world_frame."""
    return np.linalg.inv(pose_to_matrix(gamepad_pose)) @ np.asarray(world_frames)


def world_frames(object_frames, gamepad_pose):
    return pose_to_matrix(gamepad_pose) @ np.asarray(object_frames)


def interpolate_frames(a, b, fractions):
    """Linear position and shortest-arc quaternion SLERP, including endpoints."""
    pa, pb = np.array(matrix_to_pose(a)), np.array(matrix_to_pose(b))
    qa, qb = pa[3:], pb[3:]
    dot = float(qa @ qb)
    if dot < 0:
        qb, dot = -qb, -dot
    theta = np.arccos(np.clip(dot, -1, 1))
    result = []
    for x in fractions:
        q = ((1-x)*qa+x*qb if theta < 1e-6 else
             (np.sin((1-x)*theta)*qa+np.sin(x*theta)*qb)/np.sin(theta))
        result.append(pose_to_matrix([*((1-x)*pa[:3]+x*pb[:3]), *q]))
    return np.asarray(result)


class ArmGeometry:
    def __init__(self, arm, hand):
        self.arm, self.hand = arm, hand
        self.chain = SerialChain(arm.urdf, arm.tip_link)
        if self.chain.active != list(arm.joint_names) or len(self.chain.active) != 7:
            raise ValueError("demogen expects the configured seven-joint arm chain")
        self.joints = [j for j in self.chain.joints if j["name"] in self.chain.active]
        poses = self.chain.link_poses(np.zeros((1, 7)))
        zero = np.eye(4) if self.chain.root_link == arm.base_link else poses[arm.base_link][0]
        self.world_from_design = pose_to_matrix(arm.base_pose) @ np.linalg.inv(zero)
        self.mount = pose_to_matrix(hand.mount)
        self.lower, self.upper = self.chain.active_limits

    def frames(self, q):
        poses = {k: self.world_from_design @ v for k, v in
                 self.chain.link_poses(np.atleast_2d(q)).items()}
        origins = np.stack([poses[j["child"]][:, :3, 3] for j in self.joints], axis=1)
        axes = np.stack([poses[j["child"]][:, :3, :3] @ j["axis"] for j in self.joints], axis=1)
        p1, p2 = closest_lines(origins[:, 0], axes[:, 0], origins[:, 1], axes[:, 1])
        shoulder = (p1+p2)/2
        pp, aa = origins[:, 4:7], axes[:, 4:7]
        projectors = np.eye(3)-aa[:, :, :, None]*aa[:, :, None, :]
        mat = projectors.sum(axis=1)
        rhs = np.einsum("nkij,nkj->ni", projectors, pp)
        anchor = pp.mean(axis=1)
        wrist = anchor + np.einsum("nij,nj->ni", np.linalg.pinv(mat),
                                  rhs-np.einsum("nij,nj->ni", mat, anchor))
        elbow, _ = closest_lines(origins[:, 3], axes[:, 3], shoulder, unit(wrist-shoulder))
        return dict(shoulder=shoulder, elbow=elbow, wrist=wrist,
                    palm=poses[self.arm.tip_link] @ self.mount)

    def measure(self, q, centre):
        f = self.frames(q)
        upper, fore = unit(f["elbow"]-f["shoulder"]), unit(f["wrist"]-f["elbow"])
        tilt = np.degrees(np.arccos(np.clip(-upper[:, 2], -1, 1)))
        pitch = np.degrees(np.arcsin(np.clip(fore[:, 2], -1, 1)))
        target = unit((np.atleast_2d(centre)-f["elbow"])[:, :2])
        heading = np.degrees(np.arccos(np.clip(np.sum(target*unit(fore[:, :2]), axis=1), -1, 1)))
        forward = (f["elbow"]-f["shoulder"])[:, 0]
        natural = (forward > 0) & (tilt >= 10) & (tilt <= 35) & (pitch >= -10) & (pitch <= 25) & (heading <= 30)
        return dict(upper_tilt_deg=tilt, forearm_pitch_deg=pitch, forearm_heading_deg=heading,
                    elbow_forward_m=forward, natural=natural)

    def natural_residual(self, q, centre):
        m = self.measure(q, centre)
        tilt, pitch, heading = [m[k] for k in
            ("upper_tilt_deg", "forearm_pitch_deg", "forearm_heading_deg")]
        return np.stack([np.maximum(0, 10.1-tilt), np.maximum(0, tilt-34.9),
                         np.maximum(0, -9.9-pitch), np.maximum(0, pitch-24.9),
                         np.maximum(0, heading-29.9),
                         np.maximum(0, .0001-m["elbow_forward_m"])*100], axis=1)*.002

    def solve(self, target, initial, centre, *, iterations=80, position_tolerance_m=.0005,
              rotation_tolerance_rad=.01):
        """Deterministic bounded damped least squares, with a hard final check.

        A local solve can fail even when another branch is feasible. Never label
        such a failure as a proof of unreachable space.
        """
        q = np.clip(np.asarray(initial, float), self.lower, self.upper)

        def residual(batch):
            frames = self.frames(batch)["palm"]
            return np.c_[frames[:, :3, 3]-target[:3, 3],
                         .08*(frames[:, :3, :3]-target[:3, :3]).reshape(-1, 9),
                         self.natural_residual(batch, centre)]

        for _ in range(iterations):
            r = residual(q)[0]
            batch = q + np.eye(7)*1e-5
            jac = (residual(batch)-r).T/1e-5
            delta = np.linalg.solve(jac.T @ jac+np.eye(7)*1e-6, -jac.T @ r)
            delta = np.clip(delta, -.15, .15)
            best = q
            cost = float(r @ r)
            for scale in (1., .5, .25):
                candidate = np.clip(q+scale*delta, self.lower, self.upper)
                trial = residual(candidate)[0]
                if trial @ trial < cost:
                    best = candidate
                    break
            if np.max(np.abs(best-q)) < 1e-8:
                break
            q = best
            frame = self.frames(q)["palm"][0]
            if (np.linalg.norm(frame[:3, 3]-target[:3, 3]) < position_tolerance_m*.1 and
                    np.linalg.norm(frame[:3, :3]-target[:3, :3]) < rotation_tolerance_rad*.1 and
                    self.measure(q, centre)["natural"][0]):
                break
        frame = self.frames(q)["palm"][0]
        position_error = float(np.linalg.norm(frame[:3, 3]-target[:3, 3]))
        angle = float(np.arccos(np.clip((np.trace(frame[:3, :3].T @ target[:3, :3])-1)/2, -1, 1)))
        natural = bool(self.measure(q, centre)["natural"][0])
        return q, dict(passed=position_error <= position_tolerance_m and angle <= rotation_tolerance_rad and natural,
                       position_error_m=position_error, rotation_error_rad=angle, natural=natural)


class RobotKinematics:
    def __init__(self, spec, side, tip_offsets):
        self.side = side
        self.arm = ArmGeometry(getattr(spec, side+"_arm"), getattr(spec, side+"_hand"))
        hand = getattr(spec, side+"_hand")
        self.fingers = {f: SerialChain(hand.urdf, f+"_distal") for f in FINGERS}
        self.offsets = tip_offsets[side]
        self.indices = {f: [G20_JOINT_NAMES.index(n) for n in c.active] for f, c in self.fingers.items()}

    def forward(self, arm_q, hand_q):
        palm = self.arm.frames(arm_q)["palm"]
        return palm, palm[:, None] @ self.hand_frames(hand_q)

    def hand_frames(self, hand_q):
        """Fingertip frames relative to the palm, via the existing hand URDF."""
        tips = []
        for finger, chain in self.fingers.items():
            frame = chain.forward(np.atleast_2d(hand_q)[:, self.indices[finger]])
            frame[:, :3, 3] += frame[:, :3, :3] @ np.asarray(self.offsets[finger])
            tips.append(frame)
        return np.stack(tips, axis=1)

    def object_trajectory(self, stream):
        palm, tips = self.forward(stream.arm_q, stream.hand_q)
        return relative_frames(palm, stream.spec.gamepad.root_pose), relative_frames(tips, stream.spec.gamepad.root_pose)
