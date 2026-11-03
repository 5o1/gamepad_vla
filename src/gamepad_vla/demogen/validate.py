"""Native collection-backend replay. Every physical substep participates."""
from dataclasses import asdict, dataclass, replace
import importlib.metadata
import math

import numpy as np

from gamepad_vla.workstation.collection import create_backend
from gamepad_vla.workstation.commands import ArmCommand, HandCommand
from gamepad_vla.workstation.episode_schema import SIDES
from gamepad_vla.workstation.g20_mapping import G20_JOINT_NAMES
from .kinematics import ArmGeometry


@dataclass(frozen=True)
class ValidationConfig:
    penetration_tolerance_m: float = 1e-6
    control_contact_tolerance_m: float = 1e-6
    signal_timing_tolerance_s: float = .08
    analog_tolerance: float = .15
    digital_threshold: float = .30
    min_hit_s: float = .03

    def __post_init__(self):
        values = list(asdict(self).values())
        if (not np.isfinite(values).all() or min(values) < 0 or
                self.penetration_tolerance_m > 1e-6 or self.control_contact_tolerance_m > 3e-5 or
                self.signal_timing_tolerance_s > .2 or
                not 0 < self.digital_threshold < 1 or not 0 < self.min_hit_s or
                not 0 <= self.analog_tolerance <= .3):
            raise ValueError("invalid physical gate tolerances")


def trajectory_commands(trajectory, index):
    stamp = index/trajectory.fps
    result = []
    for side_index, side in enumerate(SIDES):
        result.extend([
            ArmCommand(side, tuple(trajectory.arm_q[index, side_index]), stamp),
            HandCommand(side, dict(zip(G20_JOINT_NAMES, trajectory.hand_q[index, side_index])), stamp),
        ])
    return result


def command_state(commands):
    return {c.side+("_arm" if isinstance(c, ArmCommand) else "_hand"): asdict(c) for c in commands}


def contact_samples(backend):
    """Distances for robot contacts, including full table/fixture/post geometry."""
    if backend.name == "mujoco":
        for c in backend.data.contact:
            if any(int(backend.model.geom_bodyid[g]) in backend._robot_bodies for g in c.geom):
                yield float(c.dist), [backend.model.geom(g).name for g in c.geom]
    elif backend.name == "sapien":
        for contact in backend.scene.get_contacts():
            if any(body.entity in backend._robot_entities for body in contact.bodies):
                names = [body.entity.name for body in contact.bodies]
                for point in contact.points:
                    yield float(point.separation), names
    else:
        raise ValueError("physical gate requires a native collection backend")


def contact_limit(pair, trajectory, config):
    """Only the calibrated left thumb/moving D-pad pair has normal compliance.

    Controller shell, other fingers, hand/hand and fixtures retain the 1 um
    geometric limit. Contact offset itself is never used as penetration budget.
    """
    if (trajectory.side == "left" and set(pair) == {"thumb_distal", "dpad"} and trajectory.events and
            all(e["control"] == "dpad_down:press" for e in trajectory.events)):
        return max(config.penetration_tolerance_m, config.control_contact_tolerance_m)
    return config.penetration_tolerance_m


def arm_positions(backend):
    if backend.name == "mujoco":
        return np.array([backend.data.qpos[backend._arm_indices[s]] for s in SIDES])
    result = []
    for side in SIDES:
        q = dict(zip(backend.joint_order[side], backend.robots[side].get_qpos()))
        result.append([q[n] for n in getattr(backend.spec, side+"_arm").joint_names])
    return np.asarray(result)


def step_native_tick(backend, completed_ticks, physics_dt):
    """Advance one native tick without accumulating control-clock roundoff.

    The existing backends accumulate requested time internally. Supplying the
    difference to an absolute integer-tick boundary keeps their public step
    clock aligned during long 1 ms sequences; no state or solver setting changes.
    """
    target = (completed_ticks+1)*physics_dt
    requested = getattr(backend, "_requested_time", completed_ticks*physics_dt)
    backend.step(target-requested)


def initialized_spec(trajectory):
    changes = {}
    for index, side in enumerate(SIDES):
        for kind in ("arm", "hand"):
            changes[side+"_"+kind] = replace(getattr(trajectory.spec, side+"_"+kind),
                                           home_q=tuple(getattr(trajectory, kind+"_q")[0, index]))
    return replace(trajectory.spec, **changes)


def replay(trajectory, *, backend_name="sapien", config=None, resource_root=None):
    config = config or ValidationConfig()
    centre = np.asarray(trajectory.provenance["natural_heading_point_world_m"])
    versions = {p: importlib.metadata.version(p) for p in (backend_name, "xbox-gamepad-physics")}
    if versions["xbox-gamepad-physics"] != "0.4.1":
        raise ValueError("demogen requires existing xbox-gamepad-physics 0.4.1")
    if backend_name == "sapien" and versions["sapien"] != "3.0.3":
        raise ValueError("demogen requires existing SAPIEN 3.0.3")
    if backend_name != "sapien" and config.control_contact_tolerance_m > 1e-6:
        raise ValueError("normal D-pad contact calibration is only validated for SAPIEN 3.0.3")
    spec = initialized_spec(trajectory)
    geometries = [ArmGeometry(getattr(spec, s+"_arm"), getattr(spec, s+"_hand")) for s in SIDES]
    # Full collision policy is mandatory. Neither collection's free-space nor
    # gamepad-contact acceptance shortcuts are offered by this entrypoint.
    backend = create_backend(backend_name, spec, render=False, environment_collisions=True,
                             resource_root=resource_root)
    observations, commands_out = [], []
    actual, masks, sample_frames, arm_samples = [], [], [], []
    reasons = []
    counts = dict(penetration_substeps=0, signal_mismatch_substeps=0, natural_invalid_samples=0)
    counts.update(raw_negative_contact_substeps=0, normal_control_contact_substeps=0)
    min_distance = 0.
    depth_pair = None
    # PhysX returns the configured 1 ms tick as float32. Recover its decimal
    # interval for scheduling, as CollectionSapienBackend's own tick clock does;
    # otherwise a 100 ms frame boundary can incorrectly schedule only 99 ticks.
    physics_dt = (float(backend.model.opt.timestep) if backend.name == "mujoco"
                  else round(float(backend.scene.get_timestep()), 9))
    if physics_dt > 1/trajectory.fps:
        backend.close()
        raise ValueError("physics timestep exceeds trajectory sampling period")
    try:
        arm_samples.append(arm_positions(backend))
        for distance, pair in contact_samples(backend):
            if not np.isfinite(distance):
                raise FloatingPointError("nonfinite contact distance")
            if distance < min_distance:
                min_distance, depth_pair = distance, pair
            if distance < -contact_limit(pair, trajectory, config):
                counts["penetration_substeps"] += 1
        ticks = 0
        for i in range(len(trajectory.arm_q)):
            commands = trajectory_commands(trajectory, i)
            backend.apply(commands)
            next_tick = math.floor((i+1)/trajectory.fps/physics_dt+1e-7)
            while ticks < next_tick:
                step_native_tick(backend, ticks, physics_dt)
                ticks += 1
                arm_samples.append(arm_positions(backend))
                penetration = False
                raw_negative = normal_contact = False
                for distance, pair in contact_samples(backend):
                    if not np.isfinite(distance):
                        raise FloatingPointError("nonfinite contact distance")
                    if distance < min_distance:
                        min_distance, depth_pair = distance, pair
                    limit = contact_limit(pair, trajectory, config)
                    penetration |= distance < -limit
                    raw_negative |= distance < 0
                    normal_contact |= limit > config.penetration_tolerance_m and distance < 0
                counts["penetration_substeps"] += int(penetration)
                counts["raw_negative_contact_substeps"] += int(raw_negative)
                counts["normal_control_contact_substeps"] += int(normal_contact)
                state = backend.gamepad.gamepad_state()
                values, available = np.asarray(state.values, float), np.asarray(state.available_mask, bool)
                if not np.isfinite(values).all():
                    raise FloatingPointError("nonfinite physical gamepad signal")
                actual.append(values.copy())
                masks.append(available.copy())
                sample_frames.append(i)
            observation = backend.observe()
            observation.validate()
            observations.append(observation)
            commands_out.append(command_state(commands))
        # Vectorized FK checks every measured physics sample, including reset.
        arm_samples = np.asarray(arm_samples)
        posture = {}
        for side_index, side in enumerate(SIDES):
            q = arm_samples[:, side_index]
            geometry = geometries[side_index]
            m = geometry.measure(q, centre)
            within_limits = ((q >= geometry.lower-1e-8) & (q <= geometry.upper+1e-8)).all(axis=1)
            bad = ~m["natural"] | ~within_limits
            counts["natural_invalid_samples"] += int(bad.sum())
            posture[side] = dict(invalid_samples=int(bad.sum()), total_samples=len(q),
                ranges={k: [float(v.min()), float(v.max())] for k, v in m.items() if k != "natural"})
        actual, masks, sample_frames = np.asarray(actual), np.asarray(masks), np.asarray(sample_frames)
        # Exact temporal envelope of the zero-order-held target, rather than
        # rounding tolerance up to an extra control frame. End-of-step samples
        # belong to the command interval just integrated at a grid boundary.
        times = (np.arange(len(actual))+1)*physics_dt-physics_dt*1e-6
        first = np.floor((times-config.signal_timing_tolerance_s)*trajectory.fps).astype(int)
        last = np.floor((times+config.signal_timing_tolerance_s)*trajectory.fps).astype(int)+1
        first = np.clip(first, 0, len(trajectory.arm_q)-1)
        last = np.clip(last, 1, len(trajectory.arm_q))
        intervals, inverse = np.unique(np.c_[first, last], axis=0, return_inverse=True)
        lower = np.array([trajectory.target_signal[a:b].min(axis=0) for a, b in intervals])[inverse]
        upper = np.array([trajectory.target_signal[a:b].max(axis=0) for a, b in intervals])[inverse]
        analog_bad = ((actual[:, :6] < lower[:, :6]-config.analog_tolerance) |
                      (actual[:, :6] > upper[:, :6]+config.analog_tolerance)) & masks[:, :6]
        digital = actual[:, 6:] > config.digital_threshold
        digital_bad = ((digital & ~(upper[:, 6:] > config.digital_threshold)) |
                       (~digital & (lower[:, 6:] > config.digital_threshold))) & masks[:, 6:]
        mismatch = analog_bad.any(axis=1) | digital_bad.any(axis=1)
        counts["signal_mismatch_substeps"] = int(mismatch.sum())
        event_results = []
        for event in trajectory.events:
            a, b = event["start"], event["stop"]
            channels = event["channels"]
            selected = (sample_frames >= a) & (sample_frames < b)
            target = trajectory.target_signal[a:b, channels]
            measured = actual[selected][:, channels]
            available = masks[selected][:, channels].all()
            if event["kind"] == "digital":
                hits = (measured > config.digital_threshold).all(axis=1)
                target_on = (target > config.digital_threshold).all(axis=1)
            else:
                peak = float(np.linalg.norm(target, axis=1).max())
                hits = np.linalg.norm(measured, axis=1) >= max(config.digital_threshold, peak-config.analog_tolerance)
                target_on = np.linalg.norm(target, axis=1) >= config.digital_threshold
            passed = bool(available and target_on.any() and hits.sum()*physics_dt >= config.min_hit_s)
            event_results.append(dict(**event, passed=passed, hit_duration_s=float(hits.sum()*physics_dt),
                                      channels_available=bool(available)))
        if counts["penetration_substeps"]:
            reasons.append("robot penetration exceeds numerical tolerance")
        if counts["natural_invalid_samples"]:
            reasons.append("natural posture or arm joint limits violated")
        if counts["signal_mismatch_substeps"]:
            reasons.append("actual gamepad signal mismatches target or has unintended input")
        if not event_results or any(not e["passed"] for e in event_results):
            reasons.append("required control event missing or unavailable")
        return dict(passed=not reasons, reasons=reasons, counts=counts,
                    backend=backend_name, packages=versions, physics_timestep_s=physics_dt,
                    substeps=ticks, frames=len(observations), config=asdict(config),
                    min_robot_contact_distance_m=float(min_distance), deepest_pair=depth_pair,
                    posture=posture, events=event_results,
                    natural_heading_point_world_m=centre.tolist(),
                    normal_contact_policy=("sapien-dpad-down-minimum-closure-v1; left thumb_distal/dpad only; "
                                           "other pairs retain geometric limit" if config.control_contact_tolerance_m > 1e-6
                                           else "uniform geometric numerical tolerance"),
                    collision_scope="full collection scene; native robot self-collision filters retained"), observations, commands_out
    finally:
        backend.close()
