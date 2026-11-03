"""A declared synthetic v4 fixture assembled from reviewed static joint states.

Signals are scripted labels, not measured contacts. This fixture can exercise
rejection; it is never itself placed in the accepted generated dataset.
"""
import json
import hashlib
import time

import numpy as np

from gamepad_vla.signals import SDL3_NAMES
from gamepad_vla.workstation.episode_schema import SIDES, Observation, fingertip_offsets
from gamepad_vla.workstation.g20_mapping import G20_JOINT_NAMES, apply_mimic_joint_positions
from gamepad_vla.workstation.spec import matrix_to_pose
from .kinematics import RobotKinematics
from .output import write_episode


def _source_metadata(candidates_path):
    from gamepad_vla.provenance import code_provenance
    from gamepad_vla.site import repository_root
    return dict(candidate_source=str(candidates_path),
                candidate_sha256=hashlib.sha256(candidates_path.read_bytes()).hexdigest(),
                code=code_provenance(repository_root()))


def from_candidates(spec, candidates_path, output, *, fps=30, repetitions=3):
    if fps < 10 or repetitions < 1:
        raise ValueError("synthetic fixture needs fps >= 10 and repetitions >= 1")
    candidates = json.loads(candidates_path.read_text())
    sides = candidates["sides"]
    offsets = fingertip_offsets(spec)
    arms = np.array([sides[s]["home_q_candidate"] for s in SIDES])
    grip = np.array([sides[s]["hand_q_all_candidate"] for s in SIDES])
    states = [r for r in sides["left"]["states"] if r["control"] in ("Dpad_up", "Dpad_down", "Dpad_left")]
    if not states:
        raise ValueError("candidate archive has no left D-pad states")
    hands, signals = [], []

    def hold(q, signal, seconds):
        for _ in range(round(seconds*fps)):
            hands.append(q.copy())
            signals.append(signal.copy())

    neutral = np.zeros(32)
    hold(grip, neutral, .6)
    for state in states:
        pressed = grip.copy()
        pressed[0] = apply_mimic_joint_positions("left", [state["hand_q"].get(n, 0) for n in G20_JOINT_NAMES])
        for _ in range(repetitions):
            for x in np.linspace(0, 1, max(2, round(.2*fps))):
                hold((1-x)*grip+x*pressed, neutral, 1/fps)
            active = neutral.copy()
            active[SDL3_NAMES.index(state["control"].lower())] = 1.
            hold(pressed, active, .25)
            for x in np.linspace(0, 1, max(2, round(.2*fps))):
                hold((1-x)*pressed+x*grip, neutral, 1/fps)
            hold(grip, neutral, .6)
    hands = np.asarray(hands)
    n = len(hands)
    fk = [RobotKinematics(spec, side, offsets).forward(np.repeat(arms[i][None], n, axis=0), hands[:, i])
          for i, side in enumerate(SIDES)]
    observations, commands = [], []
    from .validate import command_state
    from gamepad_vla.workstation.commands import ArmCommand, HandCommand
    start_ns = time.time_ns()
    for i in range(n):
        palm = np.array([matrix_to_pose(item[0][i]) for item in fk])
        tips = np.array([[matrix_to_pose(tip) for tip in item[1][i]] for item in fk])
        t = (i+1)/fps
        observations.append(Observation(t, start_ns+round(t*1e9), hands[i], arms, palm, tips,
                                        signals[i], np.ones(32, bool), {}, False, "synthetic"))
        cmd = []
        for side_index, side in enumerate(SIDES):
            cmd += [ArmCommand(side, tuple(arms[side_index]), i/fps),
                    HandCommand(side, dict(zip(G20_JOINT_NAMES, hands[i, side_index])), i/fps)]
        commands.append(command_state(cmd))
    return write_episode(output, observations, commands, spec=spec, fps=fps, tip_offsets=offsets,
        metadata=dict(synthetic=True, synthetic_kind="static-candidate interpolation with scripted signal labels",
                      **_source_metadata(candidates_path), physics_validated=False,
                      evidence_limit="not a human demonstration; scripted signal is not contact success"))


def physical_from_candidates(spec, candidates_path, output, *, fps=30, repetitions=3, target_value=.34,
                             environment_collisions=True):
    """Generate a short D-pad down demonstration by native signal feedback.

    Non-operating fingers are folded and the other arm is parked naturally.
    This is a fixed-controller contact test, not a certificate of hand grasp.
    The controller stops increasing depth once the minimum signal is reached.
    """
    from dataclasses import replace
    from .kinematics import ArmGeometry, heading_centre
    from .validate import command_state, contact_samples, step_native_tick
    from gamepad_vla.workstation.collection import create_backend
    from gamepad_vla.workstation.commands import ArmCommand, HandCommand
    from gamepad_vla.workstation.g20_mapping import G20_JOINT_LIMITS
    if not 10 <= fps <= 60 or not 1 <= repetitions <= 5 or not .30 < target_value <= .4:
        raise ValueError("physical fixture requires 10-60 fps, 1-5 repetitions and closure target in (0.30, 0.4]")
    candidates = json.loads(candidates_path.read_text())["sides"]
    arms = np.array([candidates[s]["home_q_candidate"] for s in SIDES])
    hands = np.array([candidates[s]["hand_q_all_candidate"] for s in SIDES])
    geometry = ArmGeometry(spec.right_arm, spec.right_hand)
    park = geometry.frames(arms[1])["palm"][0].copy()
    park[:3, 3] += [-.02, -.07, .02]
    arms[1], check = geometry.solve(park, arms[1], heading_centre(spec.gamepad.root_pose), iterations=120)
    if not check["passed"]:
        raise ValueError("synthetic inactive-arm parking IK failed")
    for index, side in enumerate(SIDES):
        for j, name in enumerate(G20_JOINT_NAMES):
            if index == 0 and name.startswith("thumb_"):
                continue
            hands[index, j] = G20_JOINT_LIMITS[side][name][1]
        hands[index] = apply_mimic_joint_positions(side, hands[index])
    state = next(s for s in candidates["left"]["states"] if s["control"] == "Dpad_down")
    pressed = np.array(apply_mimic_joint_positions("left", [state["hand_q"].get(n, 0) for n in G20_JOINT_NAMES]))
    changes = {}
    for index, side in enumerate(SIDES):
        changes[side+"_arm"] = replace(getattr(spec, side+"_arm"), home_q=tuple(arms[index]))
        changes[side+"_hand"] = replace(getattr(spec, side+"_hand"), home_q=tuple(hands[index]))
    spec = replace(spec, **changes)
    backend = create_backend("sapien", spec, render=False,
                             environment_collisions=environment_collisions, gamepad_collisions=True)
    observations, commands = [], []
    min_distances = {}
    frame = ticks = 0
    alphas = []

    def advance(alpha):
        nonlocal frame, ticks
        target = hands.copy()
        target[0, :5] = (1-alpha)*hands[0, :5]+alpha*pressed[:5]
        cmd = []
        for i, side in enumerate(SIDES):
            cmd += [ArmCommand(side, tuple(arms[i]), frame/fps),
                    HandCommand(side, dict(zip(G20_JOINT_NAMES, target[i])), frame/fps)]
        backend.apply(cmd)
        frame += 1
        stop = round(frame/fps/.001)
        while ticks < stop:
            step_native_tick(backend, ticks, .001)
            ticks += 1
            for distance, pair in contact_samples(backend):
                key = "|".join(sorted(pair))
                min_distances[key] = min(min_distances.get(key, 0.), distance)
        observation = backend.observe()
        observations.append(observation)
        commands.append(command_state(cmd))
        return float(observation.gamepad_signal[SDL3_NAMES.index("dpad_down")])

    try:
        for _ in range(round(.6*fps)):
            advance(0.)
        for _ in range(repetitions):
            for alpha in np.linspace(0, .94, round(1.5*fps)):
                value = advance(float(alpha))
            alpha = .94
            for _ in range(round(1.5*fps)):
                alpha = float(np.clip(alpha+np.clip((target_value-value)*.01, -.002, .002), 0, .97))
                value = advance(alpha)
                if value >= target_value:
                    break
            else:
                raise ValueError("physical synthetic did not reach minimum closure by feedback")
            alphas.append(alpha)
            for _ in range(round(.35*fps)):
                advance(alpha)
            for alpha in np.linspace(alpha, 0, round(1.5*fps)):
                advance(float(alpha))
            for _ in range(round(.6*fps)):
                advance(0.)
    finally:
        backend.close()
    return write_episode(output, observations, commands, spec=spec, fps=fps, tip_offsets=fingertip_offsets(spec),
        metadata=dict(synthetic=True, synthetic_kind="native feedback Dpad_down; folded fingers and natural inactive-arm park",
                      **_source_metadata(candidates_path),
                      physics_timestep_s=.001,
                      source_scene_variant="full" if environment_collisions else "gamepad-contact-calibration-only",
                      physics_generated=True, physics_validated=False, signal_source="post_physics_contact",
                      feedback=dict(target_signal=target_value, press_alphas=alphas, max_alpha=.97,
                                    approach_s=1.5, release_s=1.5,
                                    source_min_distances_m=min_distances),
                      evidence_limit="fixed gamepad, single thumb operation; not a human grasp demonstration"))
