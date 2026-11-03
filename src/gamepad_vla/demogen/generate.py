"""Seeded selection, grip bridges, resampling and object-relative arm IK."""
from dataclasses import asdict, dataclass

import numpy as np

from gamepad_vla.workstation.episode_schema import SIDES
from gamepad_vla.workstation.g20_mapping import (
    G20_JOINT_NAMES, G20_JOINT_LIMITS, G20_MIMIC_JOINTS, apply_mimic_joint_positions,
)
from .kinematics import NATURAL_HEADING_POINT_M, RobotKinematics, heading_centre, interpolate_frames, world_frames


@dataclass(frozen=True)
class GenerateConfig:
    heading_point_gamepad_m: tuple = NATURAL_HEADING_POINT_M
    speed_range: tuple = (1., 1.)
    amplitude_range: tuple = (1., 1.)
    joint_noise_rad: float = 0.
    max_joint_noise_rad: float = .01
    transition_s: float = .25
    grip_s: float = .20
    max_duration_s: float = 120.
    ik_iterations: int = 80
    position_tolerance_m: float = .0005
    rotation_tolerance_rad: float = .01
    fingertip_tolerance_m: float = .001

    def __post_init__(self):
        point = np.asarray(self.heading_point_gamepad_m, float)
        if point.shape != (3,) or not np.isfinite(point).all():
            raise ValueError("heading point must be finite xyz in the gamepad frame")
        for name in ("speed_range", "amplitude_range"):
            v = np.asarray(getattr(self, name), float)
            if v.shape != (2,) or not np.isfinite(v).all() or not 0 < v[0] <= v[1]:
                raise ValueError("invalid " + name)
        if self.amplitude_range[0] < .8 or self.amplitude_range[1] > 1.2:
            raise ValueError("amplitude scaling must stay in [0.8, 1.2]")
        values = [self.joint_noise_rad, self.max_joint_noise_rad, self.transition_s,
                  self.grip_s, self.max_duration_s, self.position_tolerance_m,
                  self.rotation_tolerance_rad, self.fingertip_tolerance_m]
        if (not np.isfinite(values).all() or self.joint_noise_rad < 0 or
                not self.joint_noise_rad <= self.max_joint_noise_rad <= .05 or
                min(values[2:]) <= 0 or self.ik_iterations < 1 or self.fingertip_tolerance_m > .001):
            raise ValueError("invalid perturbation limits, duration or IK settings")


@dataclass
class Trajectory:
    side: str
    fps: float
    arm_q: np.ndarray             # N, 2, 7; inactive side held at source grip
    hand_q: np.ndarray            # N, 2, 21
    target_signal: np.ndarray     # N, 32; never written as measured signal
    palm_object: np.ndarray
    tips_object: np.ndarray
    events: list
    provenance: dict
    spec: object
    tip_offsets: dict


class GenerationFailure(ValueError):
    def __init__(self, reason, details=None):
        super().__init__(reason)
        self.details = details or {}


def resample(values, coordinates):
    values = np.asarray(values)
    flat = values.reshape(len(values), -1)
    out = np.stack([np.interp(coordinates, np.arange(len(values)), column) for column in flat.T], axis=1)
    return out.reshape((len(coordinates),)+values.shape[1:])


def resample_frames(frames, coordinates):
    result = []
    for x in coordinates:
        i = min(int(x), len(frames)-1)
        j = min(i+1, len(frames)-1)
        result.append(interpolate_frames(frames[i], frames[j], [x-i])[0])
    return np.array(result)


def identity_trajectory(stream, library):
    """Exact applied source targets at the original pose; no IK or interpolation."""
    offsets = stream.meta["fingertip_offsets_m"]
    kin = RobotKinematics(stream.spec, stream.side, offsets)
    palm, tips = kin.object_trajectory(stream)
    events = [dict(control=s.control, start=s.start, stop=s.stop,
                   channels=s.stats["channels"], kind=s.stats["kind"])
              for s in library if s.control != "grip"]
    return Trajectory(stream.side, float(stream.meta["fps"]),
        command_targets(stream, "arm"), command_targets(stream, "hand"), stream.signal.copy(), palm, tips,
        events, dict(identity_replay=True, source=stream.source,
                     source_synthetic=bool(stream.meta.get("synthetic", False)),
                     natural_heading_point_world_m=heading_centre(stream.spec.gamepad.root_pose).tolist()),
        stream.spec, offsets)


def command_targets(stream, kind):
    """Replay known applied joint targets; measured values are an explicit fallback."""
    result = []
    for row in stream.rows:
        values = []
        for index, side in enumerate(SIDES):
            command = row["commands"].get(side+"_"+kind)
            if command is None:
                values.append(row[kind+"_q"][index])
            elif kind == "arm":
                values.append(command["positions"])
            else:
                values.append([command["joint_positions"][n] for n in G20_JOINT_NAMES])
        result.append(values)
    return np.asarray(result, float)


def generate(stream, library, target_spec, sequence, *, seed=0, config=None, source_streams=None):
    config = config or GenerateConfig()
    if not sequence:
        raise GenerationFailure("empty control sequence")
    if not target_spec.gamepad.fixed_root:
        raise GenerationFailure("target gamepad must have a fixed root")
    rng = np.random.default_rng(seed)
    sources_by_path = {s.source: s for s in (source_streams or [stream])}
    grips = [s for s in library if s.control == "grip" and s.source == stream.source]
    if not grips:
        raise GenerationFailure("no stable neutral grip candidate")
    grip = grips[int(rng.integers(len(grips)))]
    grip_index = (grip.start+grip.stop)//2
    side_index = SIDES.index(stream.side)
    offsets = stream.meta.get("fingertip_offsets_m")
    if offsets is None:
        raise GenerationFailure("source lacks fingertip_offsets_m")
    source_kin = RobotKinematics(stream.spec, stream.side, offsets)
    target_kin = RobotKinematics(target_spec, stream.side, offsets)
    relative_palm, _ = source_kin.object_trajectory(stream)
    source_cache = {stream.source: (stream, source_kin, relative_palm)}
    fps = float(stream.meta["fps"])
    nbridge, nhold = max(2, round(config.transition_s*fps)), max(2, round(config.grip_s*fps))
    hand_targets = command_targets(stream, "hand")[:, side_index]
    grip_palm, grip_hand = relative_palm[grip_index], hand_targets[grip_index]
    palms, hands, signals = [], [], []
    events, sources = [], []
    tip_references = []

    def extend(p, h, s):
        palms.extend(p)
        hands.extend(h)
        signals.extend(s)
        if len(palms)/fps > config.max_duration_s:
            raise GenerationFailure("generated duration exceeds configured limit")

    def bridge(p, h):
        fractions = np.linspace(0, 1, nbridge+1)[1:]
        extend(interpolate_frames(palms[-1], p, fractions),
               (1-fractions[:, None])*hands[-1]+fractions[:, None]*h,
               np.zeros((nbridge, 32)))

    extend(np.repeat(grip_palm[None], nhold, axis=0), np.repeat(grip_hand[None], nhold, axis=0),
           np.zeros((nhold, 32)))
    for control in sequence:
        choices = [s for s in library if s.control == control and not s.truncated and not s.overlaps
                   and s.stats.get("available", False)]
        if not choices:
            raise GenerationFailure("no complete independent segment for " + control)
        segment = choices[int(rng.integers(len(choices)))]
        if segment.source not in source_cache:
            selected = sources_by_path[segment.source]
            kin = RobotKinematics(selected.spec, selected.side, selected.meta["fingertip_offsets_m"])
            source_cache[segment.source] = (selected, kin, kin.object_trajectory(selected)[0])
        selected, kin, selected_palm = source_cache[segment.source]
        speed = float(rng.uniform(*config.speed_range))
        amplitude = float(rng.uniform(*config.amplitude_range))
        size = max(2, round((segment.stop-segment.start-1)*fps/float(selected.meta["fps"])/speed)+1)
        coordinates = np.linspace(0, segment.stop-segment.start-1, size)
        p = resample_frames(selected_palm[segment.start:segment.stop], coordinates)
        h = resample(command_targets(selected, "hand")[segment.start:segment.stop, side_index], coordinates)
        original_targets = h.copy()
        measured_hand = resample(selected.hand_q[segment.start:segment.stop], coordinates)
        s = resample(selected.signal[segment.start:segment.stop], coordinates)
        # Independent single-control snippets cannot carry another hand's input
        # or an unannotated simultaneous control into the requested sequence.
        other = [i for i in range(32) if i not in segment.stats["channels"]]
        if (np.abs(s[:, other]) >= .15).any():
            raise GenerationFailure("source fragment contains additional control activity")
        # Scale only this control's operating finger; retain the other fingers.
        finger = "index" if "trigger" in control or "shoulder" in control else "thumb"
        indices = [i for i, name in enumerate(G20_JOINT_NAMES) if name.startswith(finger+"_")]
        h[:, indices] = grip_hand[indices]+amplitude*(h[:, indices]-grip_hand[indices])
        # Smooth bounded joint perturbation vanishes at segment endpoints.
        bounds = np.array([G20_JOINT_LIMITS[stream.side][n] for n in G20_JOINT_NAMES])
        noise_indices = [i for i in indices if G20_JOINT_NAMES[i] not in G20_MIMIC_JOINTS[stream.side]]
        lower_room = np.maximum(0, h[:, noise_indices].min(axis=0)-bounds[noise_indices, 0])
        upper_room = np.maximum(0, bounds[noise_indices, 1]-h[:, noise_indices].max(axis=0))
        perturbation = rng.uniform(-np.minimum(config.joint_noise_rad, lower_room),
                                   np.minimum(config.joint_noise_rad, upper_room), (1, len(noise_indices)))
        h[:, noise_indices] += np.sin(np.linspace(0, np.pi, size))[:, None]*perturbation
        if np.any(h < bounds[:, 0]-1e-8) or np.any(h > bounds[:, 1]+1e-8):
            raise GenerationFailure("amplitude or noise violates hand joint limits")
        if amplitude != 1 or config.joint_noise_rad:
            h = np.array([apply_mimic_joint_positions(stream.side, row) for row in h])
        if not np.allclose(kin.hand_frames(h), target_kin.hand_frames(h), atol=1e-7):
            raise GenerationFailure("source and target hand kinematics differ; rigid fingertip transfer is invalid")
        # Do not silently repair an unmodified human measurement into a command.
        # The backend applies the existing mimic policy when targets are replayed.
        if segment.stats["kind"] in ("stick", "trigger"):
            s[:, segment.stats["channels"]] *= amplitude
            if (np.abs(s[:, :4]) > 1).any() or (s[:, 4:6] > 1).any():
                raise GenerationFailure("scaled analog target exceeds signal range")
        bridge(p[0], h[0])
        start = len(palms)
        extend(p, h, s)
        stop = len(palms)
        # Preserve measured source fingertip references. Applied targets can
        # differ from measurements under contact load; comparing only target FK
        # with itself would miss that error. Intentional variation is added to
        # the measured reference before checking reproduction.
        reference_hand = measured_hand+(h-original_targets)
        tip_references.append((start, stop, p[:, None] @ kin.hand_frames(reference_hand)))
        events.append(dict(control=control, start=start, stop=stop,
                           channels=segment.stats["channels"], kind=segment.stats["kind"]))
        sources.append(dict(segment=segment.to_dict(), output_frames=[start, stop],
                            source_synthetic=bool(selected.meta.get("synthetic", False)),
                            source_scene_sha256=selected.meta["scene_spec_sha256"],
                            speed=speed, amplitude=amplitude, perturbation_rad=perturbation.tolist(),
                            perturbation_joints=[G20_JOINT_NAMES[i] for i in noise_indices]))
        bridge(grip_palm, grip_hand)
        extend(np.repeat(grip_palm[None], nhold, axis=0), np.repeat(grip_hand[None], nhold, axis=0),
               np.zeros((nhold, 32)))
    palms, hands, signals = np.asarray(palms), np.asarray(hands), np.asarray(signals)
    targets = world_frames(palms, target_spec.gamepad.root_pose)
    centre = heading_centre(target_spec.gamepad.root_pose, config.heading_point_gamepad_m)
    solved, diagnostics = [], []
    previous = stream.arm_q[grip_index].copy()
    for index, target in enumerate(targets):
        if index and np.array_equal(target, targets[index-1]):
            solved.append(previous.copy())
            continue
        previous, result = target_kin.arm.solve(target, previous, centre,
            iterations=config.ik_iterations, position_tolerance_m=config.position_tolerance_m,
            rotation_tolerance_rad=config.rotation_tolerance_rad)
        if not result["passed"]:
            raise GenerationFailure("arm IK failed", dict(frame=index, **result))
        diagnostics.append(result)
        solved.append(previous.copy())
    arm_q = np.repeat(np.asarray(stream.rows[grip_index]["arm_q"])[None], len(hands), axis=0)
    hand_q = np.repeat(command_targets(stream, "hand")[grip_index][None], len(hands), axis=0)
    arm_q[:, side_index], hand_q[:, side_index] = solved, hands
    # Compute all fingertip references by existing FK; finger joints are unchanged
    # at unit amplitude/no noise. Under rigid relocation each tip follows the palm.
    tips_object = palms[:, None] @ target_kin.hand_frames(hands)
    for start, stop, reference in tip_references:
        tips_object[start:stop] = reference
    from .kinematics import relative_frames
    _, actual_tips = target_kin.forward(arm_q[:, side_index], hands)
    tip_error = np.linalg.norm(relative_frames(actual_tips, target_spec.gamepad.root_pose)[..., :3, 3]
                               - tips_object[..., :3, 3], axis=-1)
    if tip_error.max() > config.fingertip_tolerance_m:
        frame, finger = np.unravel_index(tip_error.argmax(), tip_error.shape)
        raise GenerationFailure("IK fingertip reproduction error exceeds tolerance", dict(
            frame=int(frame), finger=int(finger), error_m=float(tip_error.max()),
            tolerance_m=config.fingertip_tolerance_m))
    provenance = dict(seed=int(seed), config=asdict(config), source=stream.source,
                      source_scene_sha256=stream.meta["scene_spec_sha256"],
                      source_synthetic=bool(stream.meta.get("synthetic", False)) or
                                       any(s["source_synthetic"] for s in sources),
                      grip_segment=grip.to_dict(), grip_frame=grip_index, fragments=sources,
                      source_gamepad_pose=list(stream.spec.gamepad.root_pose),
                      target_gamepad_pose=list(target_spec.gamepad.root_pose),
                      natural_heading_point_world_m=centre.tolist(),
                      ik=dict(solves=len(diagnostics), max_position_error_m=max(
                          r["position_error_m"] for r in diagnostics),
                          max_fingertip_error_m=float(tip_error.max())),
                      inactive_side_policy="held at measured source grip; both arms validated",
                      hand_targets="known applied joint targets; measured radians fallback; no invented motor messages")
    return Trajectory(stream.side, fps, arm_q, hand_q, signals, palms, tips_object,
                      events, provenance, target_spec, offsets)
