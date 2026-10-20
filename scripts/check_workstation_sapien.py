"""Integration check of the SAPIEN workcell backend (the compute node, no hardware).

1. Hold: no commands for 2 s; arm stays at home, hand stays open, gamepad neutral.
2. UDP: a local sender emits station-relay ``command`` packets (G20 motor values)
   sweeping the left thumb; the workstation loop follows them and records an episode.
3. Press: the gamepad is placed from a thumb-reach report placement (palm frame),
   or kept at the spec pose with ``--keep-spec-gamepad``; the thumb tip is moved
   above and then into each D-pad press point (``--directions``), and the
   32-channel signal is read back.
4. With ``--keep-spec-gamepad``, robot contacts with the table or gamepad at the
   home pose (open hand) are listed.

Only simulation; the D-pad geometry and all drive gains are unmeasured placeholders.

Usage (SAPIEN env):
    python scripts/check_workstation_sapien.py --spec configs/workcell/station_left_g20_v3.json \
        --reach-report <report_angle0.json> --keep-spec-gamepad --directions up right down left \
        --output <run dir>
"""

import argparse
import json
import socket
import threading
import time
from dataclasses import replace
from pathlib import Path

import numpy as np

from gamepad_vla import signals
from gamepad_vla.kinematics import SerialChain
from gamepad_vla.workstation.commands import HandCommand
from gamepad_vla.workstation.g20_mapping import (
    G20_JOINT_NAMES,
    apply_mimic_joint_positions,
    joint_positions_to_motor_positions,
)
from gamepad_vla.workstation.loop import run_loop
from gamepad_vla.workstation.recorder import EpisodeRecorder, read_episode
from gamepad_vla.workstation.sapien_backend import SapienBackend
from gamepad_vla.workstation.sources import ScriptedSource, UdpSource
from gamepad_vla.workstation.spec import compose, load_spec, matrix_to_pose, pose_to_matrix

parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
parser.add_argument("--spec", type=Path, required=True)
parser.add_argument("--reach-report", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--dpad-top", type=float, nargs=3, default=(-0.0203, -0.0018, 0.0184))
parser.add_argument("--press-radius", type=float, default=0.007)
parser.add_argument("--press-depth", type=float, default=0.0015)
parser.add_argument("--example", type=int, default=0)
parser.add_argument("--keep-spec-gamepad", action="store_true",
                    help="use the spec's gamepad pose and home_q instead of re-posing the gamepad")
parser.add_argument("--directions", nargs="+", default=["up"], choices=["up", "right", "down", "left"])
parser.add_argument("--finger-q", type=Path, help="JSON {joint: rad} held by the non-thumb fingers while pressing")
args = parser.parse_args()

DPAD = {name: signals.SDL3_NAMES.index(name) for name in signals.DPAD_NAMES}


class Clock:
    monotonic = staticmethod(time.monotonic)
    sleep = staticmethod(time.sleep)


def hand_command(q, stamp=0.0):
    return HandCommand("left", dict(zip(G20_JOINT_NAMES, apply_mimic_joint_positions("left", q))), stamp)


def hold_check(spec):
    backend = SapienBackend(spec, work_dir=args.output / "hold")
    for _ in range(120):
        backend.step(1 / 60)
    obs = backend.observe()
    arm_error = np.max(np.abs(np.array(obs.arm_joint_positions) - spec.left_arm.home_q))
    hand_max = max(abs(v) for v in obs.hand_joint_positions.values())
    available = obs.gamepad_mask
    right_error = 0.0
    if backend.right_robot is not None:
        target = dict(zip(spec.right_arm.joint_names, spec.right_arm.home_q))
        target.update(dict(zip(("right_" + n for n in G20_JOINT_NAMES), spec.right_hand.home_q)))
        right_error = float(np.max(np.abs(backend.right_robot.get_qpos() -
                          np.array([target[n] for n in backend.right_joint_order]))))
    return dict(arm_max_error_rad=float(arm_error), hand_max_abs_rad=float(hand_max),
                right_max_error_rad=right_error,
                gamepad_max_abs_signal=float(np.max(np.abs(obs.gamepad_signal[available]))))


def udp_check(spec):
    backend = SapienBackend(spec, work_dir=args.output / "udp")
    source = UdpSource("127.0.0.1", 0, stale_after_s=0.25)
    mcp = G20_JOINT_NAMES.index("thumb_mcp")
    stop = threading.Event()

    def sender():
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        start = time.monotonic()
        while not stop.is_set():
            q = [0.0] * 21
            q[mcp] = 1.2 * min(1.0, (time.monotonic() - start) / 2.0)
            motor = joint_positions_to_motor_positions("left", apply_mimic_joint_positions("left", q))
            # Same field set and encoding as the station's g20_manus_ros2_udp_relay.py.
            payload = {"type": "command", "hand": "left", "format": "motor",
                       "name": [f"motor_{i}" for i in range(20)],
                       "position": [float(min(max(round(v), 0), 255)) for v in motor],
                       "velocity": [], "effort": [], "stamp": time.time()}
            sock.sendto(json.dumps(payload, separators=(",", ":")).encode(), source.bound_address)
            time.sleep(1 / 60)

    thread = threading.Thread(target=sender, daemon=True)
    thread.start()
    with EpisodeRecorder(args.output / "episodes", "udp_thumb_sweep", spec=spec, backend_names=["sapien"],
                         source_description="local synthetic station-relay packets",
                         rates={"control_hz": 60}, provenance={"check": "check_workstation_sapien"}) as rec:
        summary = run_loop(source, backend, rec, rate_hz=60, duration_s=3.0, clock=Clock())
    stop.set()
    thread.join()
    source.close()
    _, arrays = read_episode(args.output / "episodes" / "udp_thumb_sweep")
    ip = G20_JOINT_NAMES.index("thumb_ip")
    final = arrays["hand_q"][-1]
    return dict(loop=summary, rows=int(len(arrays["t"])), final_thumb_mcp=float(final[mcp]),
                final_thumb_ip=float(final[ip]), ip_over_mcp=float(final[ip] / final[mcp]),
                decode_errors=source.decode_errors)


def press_check(spec):
    report = json.loads(args.reach_report.read_text())
    probe = SapienBackend(spec, work_dir=args.output / "press_probe")
    palm_world = probe.observe().hand_root_pose_world
    if args.keep_spec_gamepad:
        gamepad_world = spec.gamepad.root_pose
        palm_gamepad = np.linalg.inv(pose_to_matrix(palm_world)) @ pose_to_matrix(gamepad_world)
    else:
        example = report["collision"]["examples"][args.example]
        centre = np.array(example["dpad_centre_m"])
        right, up, n = (np.array(example[k]) for k in ("dpad_right_axis", "dpad_up_axis", "dpad_outward_normal"))
        # Gamepad frame in the palm frame: axes (right, up, n), cap-top centre at `centre`.
        palm_gamepad = np.eye(4)
        palm_gamepad[:3, :3] = np.stack([right, up, n], axis=1)
        palm_gamepad[:3, 3] = centre - palm_gamepad[:3, :3] @ np.array(args.dpad_top)
        gamepad_world = compose(palm_world, matrix_to_pose(palm_gamepad))
        spec = replace(spec, gamepad=replace(spec.gamepad, root_pose=tuple(gamepad_world)))
    rotation = palm_gamepad[:3, :3]
    centre = palm_gamepad[:3, :3] @ np.array(args.dpad_top) + palm_gamepad[:3, 3]
    n = rotation[:, 2]
    axis = {"up": rotation[:, 1], "down": -rotation[:, 1], "right": rotation[:, 0], "left": -rotation[:, 0]}

    # Thumb configurations by sampled FK in the palm frame (contact model of the report).
    chain = SerialChain(spec.left_hand.urdf, "thumb_distal")
    point = np.array(report["contact_point_in_distal_m"])
    normal = np.array(report["contact_normal_in_distal"])
    lower, upper = chain.active_limits
    q = lower + np.random.default_rng(0).random((400_000, 4)) * (upper - lower)
    pose = chain.forward(q)
    tips = pose[:, :3, :3] @ point + pose[:, :3, 3]
    aligned = -(pose[:, :3, :3] @ normal) @ n >= np.cos(np.radians(30))

    def solve(target):
        error = np.linalg.norm(tips - target, axis=1) + np.where(aligned, 0.0, 1.0)
        best = int(np.argmin(error))
        return q[best], float(error[best])

    fingers = json.loads(args.finger_q.read_text()) if args.finger_q else {}

    def full(thumb):
        values = [0.0] * 21
        for name, value in fingers.items():
            values[G20_JOINT_NAMES.index(name)] = float(value)
        for name, value in zip(chain.active, thumb):
            values[G20_JOINT_NAMES.index(name)] = float(value)
        return hand_command(values)

    # Per direction: hover 4 mm above its press point for 1 s, press for 1.5 s, hover 1 s.
    schedule, windows, ik_error, t = [], {}, {}, 0.0
    for name in args.directions:
        press_point = centre + args.press_radius * axis[name]
        above, above_err = solve(press_point + 0.004 * n)
        pressed, pressed_err = solve(press_point - args.press_depth * n)
        ik_error[name] = dict(above=above_err, pressed=pressed_err)
        schedule += [(t, full(above)), (t + 1.0, full(pressed)), (t + 2.5, full(above))]
        windows[name] = dict(approach=(t + 0.5, t + 1.0), pressed=(t + 2.0, t + 2.5), released=(t + 3.2, t + 3.5))
        t += 3.5

    backend = SapienBackend(spec, work_dir=args.output / "press")
    episode = "dpad_press_" + "_".join(args.directions)
    with EpisodeRecorder(args.output / "episodes", episode, spec=spec, backend_names=["sapien"],
                         source_description="scripted thumb press", rates={"control_hz": 60},
                         provenance={"reach_report": str(args.reach_report), "example": args.example,
                                     "keep_spec_gamepad": args.keep_spec_gamepad}) as rec:
        run_loop(ScriptedSource(schedule), backend, rec, rate_hz=60, duration_s=t, clock=_VirtualClock())
    _, arrays = read_episode(args.output / "episodes" / episode)
    times, sig = arrays["t"], arrays["gamepad_signal"]
    result = {}
    for name, spans in windows.items():
        result[name] = {w: {d: float(sig[(times > a) & (times < b), idx].max()) for d, idx in DPAD.items()}
                        for w, (a, b) in spans.items()}
    return dict(gamepad_root_world=[round(v, 5) for v in gamepad_world], ik_error_m=ik_error, dpad_max=result)


def home_contacts(spec):
    """Robot-vs-table/gamepad contacts after 0.5 s at home with the hand open."""
    backend = SapienBackend(spec, work_dir=args.output / "contacts")
    robot = {link.name for link in backend.robot.get_links()}
    found = {}
    right = {link.name for link in backend.right_robot.get_links()} if backend.right_robot else set()
    right_pairs = {}
    # Capture from the first physical substep, so an initial penetration cannot
    # disappear before the old 0.5-second snapshot.
    for _ in range(120):
        backend.step(1 / 240)
        for contact in backend.scene.get_contacts():
            a, b = (body.entity.name for body in contact.bodies)
            if not contact.points:
                continue
            key = " | ".join(sorted((a, b)))
            depth = min(point.separation for point in contact.points)
            if (a in robot) != (b in robot):
                found[key] = min(found.get(key, 0.0), float(depth))
            if (a in right) != (b in right):
                right_pairs[key] = min(right_pairs.get(key, 0.0), float(depth))
    return dict(pairs=found, right_pairs=right_pairs,
                palm_world=[round(v, 5) for v in backend.observe().hand_root_pose_world])


class _VirtualClock:
    """Simulation time instead of wall time: the scripted press need not run in real time."""

    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


def main():
    args.output.mkdir(parents=True, exist_ok=False)
    spec = load_spec(args.spec)
    report = dict(spec=str(args.spec), hold=hold_check(spec), udp=udp_check(spec))
    if args.keep_spec_gamepad:
        report["home_contacts"] = home_contacts(spec)
    report.update(press=press_check(spec),
                  scope="SAPIEN 3.0.3 simulation only; placeholder gains, unmeasured D-pad and mount.")
    hold = report["hold"]
    passed = (hold["arm_max_error_rad"] < 1e-6 and hold["hand_max_abs_rad"] < 1e-6
              and hold["right_max_error_rad"] < 1e-6 and hold["gamepad_max_abs_signal"] == 0)
    if args.keep_spec_gamepad:
        passed &= not report["home_contacts"]["pairs"]
    for direction, windows in report["press"]["dpad_max"].items():
        for window, values in windows.items():
            passed &= all((v > .3 if window == "pressed" and d == "dpad_" + direction else v == 0)
                          for d,v in values.items())
    report["passed"] = bool(passed)
    (args.output / "report.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
    if not passed:
        raise SystemExit(1)


main()
