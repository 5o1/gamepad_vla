"""CPU contracts for slicing, object frames, deterministic generation and IO."""
from dataclasses import replace
import json
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from gamepad_vla.demogen.episode import load_demonstration
from gamepad_vla.demogen.generate import GenerateConfig, GenerationFailure, generate, identity_trajectory
from gamepad_vla.demogen.kinematics import (
    ArmGeometry, RobotKinematics, heading_centre, relative_frames, world_frames,
)
from gamepad_vla.demogen.output import data_directory, write_episode
from gamepad_vla.demogen.pipeline import run
from gamepad_vla.demogen.segments import SliceConfig, slice_stream
from gamepad_vla.demogen.validate import ValidationConfig, trajectory_commands, command_state
from gamepad_vla.signals import SDL3_NAMES
from gamepad_vla.workstation.episode_schema import SIDES, FINGERS, Observation, read_episode
from gamepad_vla.workstation.spec import (
    ArmSpec, GamepadSpec, HandSpec, TableSpec, WorkcellSpec, dump_spec, matrix_to_pose,
)


def make_robot(tmp_path):
    """Small SI-unit URDF, natural at q=0, using production FK (no simulator)."""
    arm = ET.Element("robot", name="test_arm")
    ET.SubElement(arm, "link", name="base")
    origins = ["0 0 0", "0 0 0", ".12 0 -.30", "0 0 0", ".3 0 .05", "0 0 0", "0 0 0"]
    axes = ["0 0 1", "0 1 0", "1 0 0", "0 1 0", "1 0 0", "0 1 0", "0 0 1"]
    parent = "base"
    names = []
    for i, (origin, axis) in enumerate(zip(origins, axes)):
        child, name = f"link{i}", f"joint{i}"
        ET.SubElement(arm, "link", name=child)
        j = ET.SubElement(arm, "joint", name=name, type="revolute")
        ET.SubElement(j, "parent", link=parent)
        ET.SubElement(j, "child", link=child)
        ET.SubElement(j, "origin", xyz=origin)
        ET.SubElement(j, "axis", xyz=axis)
        ET.SubElement(j, "limit", lower="-2.5", upper="2.5", effort="100", velocity="2")
        parent = child
        names.append(name)
    arm_path = tmp_path / "arm.urdf"
    ET.ElementTree(arm).write(arm_path)
    from gamepad_vla.workstation.g20_mapping import G20_JOINT_NAMES, G20_MIMIC_JOINTS
    hand = ET.Element("robot", name="test_hand")
    ET.SubElement(hand, "link", name="hand_base_link")
    for finger in FINGERS:
        parent = "hand_base_link"
        finger_names = [n for n in G20_JOINT_NAMES if n.startswith(finger+"_")]
        for i, name in enumerate(finger_names):
            child = finger+"_distal" if i == len(finger_names)-1 else name+"_link"
            ET.SubElement(hand, "link", name=child)
            j = ET.SubElement(hand, "joint", name=name, type="revolute")
            ET.SubElement(j, "parent", link=parent)
            ET.SubElement(j, "child", link=child)
            ET.SubElement(j, "origin", xyz="0 0 .01")
            ET.SubElement(j, "axis", xyz="0 1 0")
            ET.SubElement(j, "limit", lower="-2", upper="2", effort="10", velocity="2")
            if name in G20_MIMIC_JOINTS["left"]:
                source, multiplier = G20_MIMIC_JOINTS["left"][name]
                ET.SubElement(j, "mimic", joint=source, multiplier=str(multiplier))
            parent = child
    hand_path = tmp_path / "hand.urdf"
    ET.ElementTree(hand).write(hand_path)
    pose = (0., 0., 1., 1., 0., 0., 0.)
    arm_spec = ArmSpec(str(arm_path), "base", "link6", pose, (0.,)*7, tuple(names))
    hand_spec = HandSpec(str(hand_path), "hand_base_link", (0., 0., 0., 1., 0., 0., 0.), "unverified", (0.,)*21)
    spec = WorkcellSpec("test", "world", TableSpec((0., 0., -.1, 1., 0., 0., 0.), (.5, .5, .05)),
                        arm_spec, hand_spec, GamepadSpec("xbox-gamepad-physics==0.4.1", "full_controls", "coacd",
                        (.8, 0., .8, 1., 0., 0., 0.), True), (), right_arm=arm_spec, right_hand=hand_spec)
    offsets = {s: {f: [0., 0., .01] for f in FINGERS} for s in SIDES}
    return spec, offsets


def make_episode(tmp_path):
    spec, offsets = make_robot(tmp_path)
    n, fps = 100, 30
    arms, hands = np.zeros((n, 2, 7)), np.zeros((n, 2, 21))
    signals = np.zeros((n, 32))
    for a, b in ((20, 30), (55, 65)):
        signals[a:b, SDL3_NAMES.index("dpad_up")] = 1
        hands[a:b, 0, 3] = .15
        hands[a:b, 0, 4] = .153
    fk = [RobotKinematics(spec, side, offsets).forward(arms[:, i], hands[:, i]) for i, side in enumerate(SIDES)]
    observations, commands = [], []
    for i in range(n):
        palm = np.array([matrix_to_pose(item[0][i]) for item in fk])
        tips = np.array([[matrix_to_pose(tip) for tip in item[1][i]] for item in fk])
        observations.append(Observation((i+1)/fps, 1_000_000_000+round((i+1)/fps*1e9), hands[i], arms[i],
            palm, tips, signals[i], np.ones(32, bool), {}, False, "synthetic"))
        trajectory = SimpleNamespace(fps=fps, arm_q=arms, hand_q=hands)
        commands.append(command_state(trajectory_commands(trajectory, i)))
    output = tmp_path / "source"
    write_episode(output, observations, commands, spec=spec, fps=fps, tip_offsets=offsets,
                  metadata=dict(synthetic=True, physics_validated=False))
    return output, spec, offsets


@pytest.fixture
def demo(tmp_path):
    path, spec, offsets = make_episode(tmp_path)
    stream = load_demonstration(path)["left"]
    return stream, slice_stream(stream), spec, offsets


def signal_stream(signals, fps=30):
    n = len(signals)
    return SimpleNamespace(source="test", side="left", time=np.arange(n)/fps,
        signal=signals, mask=np.ones((n, 32), bool), arm_q=np.zeros((n, 7)), hand_q=np.zeros((n, 21)))


def test_slicing_jitter_overlap_and_boundaries():
    signals = np.zeros((100, 32))
    up, down = (SDL3_NAMES.index(n) for n in ("dpad_up", "dpad_down"))
    signals[:5, up] = 1
    signals[20:35, up] = 1
    signals[25, up] = 0        # One-frame dropout bridged.
    signals[30:40, down] = 1  # Simultaneous controls are retained and marked.
    signals[60, up] = 1       # One-frame pulse removed by debounce.
    signals[-5:, up] = 1
    segments = slice_stream(signal_stream(signals))
    actions = [s for s in segments if s.control != "grip"]
    assert len(actions) == 4
    assert actions[0].truncated and actions[-1].truncated
    assert actions[1].active_start == 20 and actions[1].active_stop == 35
    assert actions[1].overlaps == ["dpad_down:press"]
    assert actions[2].overlaps == ["dpad_up:press"]
    assert all(0 <= s.start < s.stop <= 100 for s in segments)


def test_adjacent_padding_and_grip(demo):
    stream, library, _, _ = demo
    actions = [s for s in library if s.control == "dpad_up:press"]
    assert len(actions) == 2 and not any(s.truncated or s.overlaps for s in actions)
    assert actions[0].stop <= actions[1].start
    assert [s for s in library if s.control == "grip"]
    moving = replace(stream, arm_q=np.tile(np.arange(len(stream.time))[:, None]*.1, (1, 7)))
    assert not any(s.control == "grip" for s in slice_stream(moving))


def test_stable_grip_after_neutral_approach():
    stream = signal_stream(np.zeros((60, 32)))
    stream.hand_q[:20, 0] = np.linspace(0, .8, 20)
    stream.hand_q[20:, 0] = .8
    library = slice_stream(stream)
    grips = [s for s in library if s.control == "grip"]
    assert len(grips) == 1 and grips[0].start >= 19 and grips[0].stop == 60


@pytest.mark.parametrize("direction", range(8))
def test_stick_directions_and_amplitude(direction):
    signals = np.zeros((60, 32))
    angle = direction*np.pi/4
    signals[15:35, :2] = [.8*np.cos(angle), .8*np.sin(angle)]
    actions = [s for s in slice_stream(signal_stream(signals)) if s.stats["kind"] == "stick"]
    assert len(actions) == 1 and actions[0].control == f"left_stick:dir{direction}"
    assert actions[0].stats["peak"] == pytest.approx(.8)


def test_circle_trigger_and_unavailable():
    signals = np.zeros((100, 32))
    angles = np.linspace(0, 2*np.pi, 40)
    signals[10:50, :2] = np.c_[.8*np.cos(angles), .8*np.sin(angles)]
    signals[65:85, 4] = np.r_[np.linspace(.35, 1, 10), np.linspace(1, .35, 10)]
    stream = signal_stream(signals)
    library = slice_stream(stream)
    assert any(s.control == "left_stick:circle_positive" for s in library)
    trigger = next(s for s in library if s.control == "left_trigger:travel")
    assert trigger.stats["travel_range"] == pytest.approx([.35, 1])
    stream.mask[:, 4] = False
    assert not any(s.control == trigger.control for s in slice_stream(stream))


def test_frames_roundtrip_and_joint_fk(demo):
    stream, _, spec, offsets = demo
    kin = RobotKinematics(spec, "left", offsets)
    palm, tips = kin.forward(stream.arm_q, stream.hand_q)
    obj_palm, obj_tips = kin.object_trajectory(stream)
    assert np.allclose(world_frames(obj_palm, spec.gamepad.root_pose), palm, atol=1e-12)
    assert np.allclose(world_frames(obj_tips, spec.gamepad.root_pose), tips, atol=1e-12)
    assert np.allclose([matrix_to_pose(p) for p in palm], stream.palm_world)
    moved = replace(spec.gamepad, root_pose=(.81, .01, .8, .9238795325, 0., 0., .3826834324))
    assert np.allclose(relative_frames(world_frames(obj_tips, moved.root_pose), moved.root_pose), obj_tips)


def test_natural_geometry_and_ik_failure(demo):
    _, _, spec, _ = demo
    geometry = ArmGeometry(spec.left_arm, spec.left_hand)
    measurement = geometry.measure(np.zeros(7), spec.gamepad.root_pose[:3])
    assert measurement["natural"][0]
    assert measurement["upper_tilt_deg"][0] == pytest.approx(np.degrees(np.arctan2(.12, .3)))
    assert measurement["forearm_pitch_deg"][0] == pytest.approx(np.degrees(np.arctan2(.05, .3)))
    target = geometry.frames(np.zeros(7))["palm"][0].copy()
    target[:3, 3] += [5, 5, 5]
    _, result = geometry.solve(target, np.zeros(7), spec.gamepad.root_pose[:3], iterations=3)
    assert not result["passed"] and result["position_error_m"] > 1


def test_natural_heading_reference_matches_reviewed_geometry():
    root = Path(__file__).resolve().parents[1]
    placement = json.loads((root / "docs/reports/2026-10-22-gamepad-height/selected-pair.json").read_text())["placement"]
    assert np.allclose(heading_centre(placement["pose"]), placement["gamepad_centroid_world_m"], atol=1e-12)


def test_generate_seed_bridges_and_relocation(demo):
    stream, library, spec, _ = demo
    target = replace(spec, gamepad=replace(spec.gamepad, root_pose=(.805, 0., .8, 1., 0., 0., 0.)))
    config = GenerateConfig(speed_range=(.9, 1.1), amplitude_range=(.97, 1.03), joint_noise_rad=.002)
    first = generate(stream, library, target, ["dpad_up:press"]*2, seed=42, config=config)
    second = generate(stream, library, target, ["dpad_up:press"]*2, seed=42, config=config)
    assert np.array_equal(first.arm_q, second.arm_q)
    assert np.array_equal(first.hand_q, second.hand_q)
    assert first.provenance == second.provenance
    other = generate(stream, library, target, ["dpad_up:press"]*2, seed=43, config=config)
    assert first.provenance["fragments"] != other.provenance["fragments"]
    assert np.max(np.linalg.norm(np.diff(first.arm_q, axis=0), axis=-1)) < .03
    assert np.allclose(first.hand_q[0], first.hand_q[-1])
    a, b = first.events
    assert np.allclose(first.target_signal[a["stop"]:b["start"]], 0)
    assert np.allclose(first.hand_q[a["stop"]+8, 0], first.hand_q[0, 0])


def test_no_grip_or_no_segment_is_failure(demo):
    stream, library, spec, _ = demo
    with pytest.raises(GenerationFailure, match="grip"):
        generate(stream, [s for s in library if s.control != "grip"], spec, ["dpad_up:press"])
    with pytest.raises(GenerationFailure, match="complete independent"):
        generate(stream, library, spec, ["dpad_down:press"])


def test_segments_can_be_selected_across_episodes(demo):
    stream, library, spec, _ = demo
    second = replace(stream, source="second-v4-episode")
    first_library = [s for s in library if s.control == "grip"]
    second_library = [replace(s, source=second.source) for s in library if s.control != "grip"]
    trajectory = generate(stream, first_library+second_library, spec, ["dpad_up:press"],
                          source_streams=[stream, second])
    assert trajectory.provenance["fragments"][0]["segment"]["source"] == second.source


def test_identity_replay_preserves_exact_source(demo):
    stream, library, spec, _ = demo
    trajectory = identity_trajectory(stream, library)
    assert trajectory.spec == spec
    assert np.array_equal(trajectory.arm_q, [r["arm_q"] for r in stream.rows])
    assert np.array_equal(trajectory.hand_q, [r["hand_q"] for r in stream.rows])
    assert np.array_equal(trajectory.target_signal, stream.signal)
    assert trajectory.provenance["identity_replay"]


def test_fingertip_gate_rechecks_fk_after_ik(demo, monkeypatch):
    stream, library, spec, _ = demo
    original = ArmGeometry.solve
    def bad_solver(self, *args, **kwargs):
        q, result = original(self, *args, **kwargs)
        q[0] += .05
        result["passed"] = True
        return q, result
    monkeypatch.setattr(ArmGeometry, "solve", bad_solver)
    with pytest.raises(GenerationFailure, match="fingertip reproduction"):
        generate(stream, library, spec, ["dpad_up:press"])


def test_fingertip_reference_is_source_measurement_not_command_fk(demo):
    stream, library, spec, _ = demo
    for row in stream.rows:
        command = row["commands"]["left_hand"]["joint_positions"]
        command["thumb_mcp"] += .1
        command["thumb_ip"] += .102
    with pytest.raises(GenerationFailure, match="fingertip reproduction"):
        generate(stream, library, spec, ["dpad_up:press"])


def test_normal_contact_tolerance_is_calibrated_pair_and_task_only():
    from gamepad_vla.demogen.validate import contact_limit
    trajectory = SimpleNamespace(side="left", events=[dict(control="dpad_down:press")])
    config = ValidationConfig(control_contact_tolerance_m=3e-5)
    assert contact_limit(["thumb_distal", "dpad"], trajectory, config) == 3e-5
    for pair in (["thumb_distal", "controller_body"], ["pinky_distal", "dpad"],
                 ["thumb_distal", "right_thumb_distal"], ["thumb_distal", "table"]):
        assert contact_limit(pair, trajectory, config) == 1e-6
    trajectory.events = [dict(control="dpad_up:press")]
    assert contact_limit(["thumb_distal", "dpad"], trajectory, config) == 1e-6
    with pytest.raises(ValueError):
        ValidationConfig(control_contact_tolerance_m=3.1e-5)


def test_long_native_clock_stays_on_integer_ticks():
    import math
    from gamepad_vla.demogen.validate import step_native_tick
    class ExistingClock:
        _requested_time = 0.
        ticks = 0
        def step(self, dt):
            self._requested_time += dt
            self.ticks = math.floor(self._requested_time/.001+1e-9)
    clock = ExistingClock()
    for i in range(20000):
        step_native_tick(clock, i, .001)
        assert clock.ticks == i+1


@pytest.mark.parametrize("damage", ["version", "incomplete", "count", "time", "sidecar", "discard", "hash"])
def test_strict_v4_integrity(demo, damage):
    stream, _, _, _ = demo
    path = Path(stream.source)
    meta = json.loads((path / "meta.json").read_text())
    if damage == "version":
        meta["schema_version"] = 3
    elif damage == "incomplete":
        meta["status"] = "recording"
    elif damage == "count":
        meta["received_input_messages"] = 1
    elif damage == "time":
        meta["recording_end_sim_time_s"] += 1
    elif damage == "hash":
        meta["scene_spec_sha256"] = "0"*64
    elif damage == "sidecar":
        (path / "integrity.json").write_text('{"state":"running","passed":null}')
    else:
        (path / "discarded.json").write_text('{}')
    (path / "meta.json").write_text(json.dumps(meta))
    with pytest.raises(ValueError):
        load_demonstration(path)


def test_output_uses_site_data_root(tmp_path, monkeypatch):
    config = tmp_path / "site.json"
    config.write_text(json.dumps(dict(station=dict(data_root=str(tmp_path / "data")))))
    monkeypatch.setenv("GAMEPAD_VLA_SITE_CONFIG", str(config))
    assert data_directory("test/run") == tmp_path / "data/demogen/test/run"
    with pytest.raises(ValueError):
        data_directory("../../elsewhere")


def test_pipeline_refuses_rejected_physics(demo, tmp_path, monkeypatch):
    stream, _, spec, _ = demo
    target = tmp_path / "target.json"
    dump_spec(spec, target)
    from gamepad_vla.demogen import pipeline
    monkeypatch.setattr(pipeline, "replay", lambda *a, **k: (
        dict(passed=False, reasons=["missed control", "penetration"]), [], []))
    output = tmp_path / "rejected"
    result = run([stream.source], [target], ["dpad_up:press"], output, backend="sapien")
    assert result["accepted"] == 0 and result["rejected"] == 1
    assert result["attempts"][0]["stage"] == "physics"
    assert not list(output.glob("episode-*"))
    assert json.loads((output / "manifest.json").read_text())["status"] == "complete"


def test_pipeline_accepted_serialization_contract(demo, tmp_path, monkeypatch):
    """Stub gate checks IO only; this test is not a physical success claim."""
    stream, _, spec, _ = demo
    target = tmp_path / "target.json"
    dump_spec(spec, target)
    from gamepad_vla.demogen import pipeline
    def stub_replay(trajectory, **kwargs):
        n = len(trajectory.arm_q)
        fk = [RobotKinematics(trajectory.spec, side, trajectory.tip_offsets).forward(
            trajectory.arm_q[:, j], trajectory.hand_q[:, j]) for j, side in enumerate(SIDES)]
        obs = [Observation((i+1)/trajectory.fps, 1_000_000_000+i, trajectory.hand_q[i], trajectory.arm_q[i],
            np.array([matrix_to_pose(item[0][i]) for item in fk]),
            np.array([[matrix_to_pose(tip) for tip in item[1][i]] for item in fk]),
            np.zeros(32), np.ones(32, bool), {}, False, "test_stub") for i in range(n)]
        return dict(passed=True, reasons=[], physics_timestep_s=.001), obs, [
            command_state(trajectory_commands(trajectory, i)) for i in range(n)]
    monkeypatch.setattr(pipeline, "replay", stub_replay)
    output = tmp_path / "accepted"
    manifest = run([stream.source], [target], ["dpad_up:press"], output, backend="sapien")
    assert manifest["accepted"] == 1
    meta, rows = read_episode(output / "episode-000000")
    assert meta["generated"] and meta["synthetic"]
    assert meta["code"]["source_sha256"]
    assert all(row["commands"]["left_hand"]["motor_positions"] is None for row in rows)
    assert load_demonstration(output / "episode-000000")["left"].meta["status"] == "complete"
    from gamepad_vla.workstation.collection_sources import CollectionReplay
    replay = CollectionReplay(output / "episode-000000")
    assert len(replay.poll(0)) == 4


@pytest.mark.parametrize("factory,kwargs", [
    (SliceConfig, dict(on=.1, off=.2)), (GenerateConfig, dict(amplitude_range=(.5, 1))),
    (GenerateConfig, dict(joint_noise_rad=.02)), (ValidationConfig, dict(penetration_tolerance_m=.001)),
])
def test_invalid_settings(factory, kwargs):
    with pytest.raises(ValueError):
        factory(**kwargs)


@pytest.mark.parametrize("fault", [None, "miss", "unintended", "penetration", "posture", "unavailable"])
def test_gate_checks_native_substeps(demo, monkeypatch, fault):
    """Controlled test adapter; spikes are absent at the recorded frame end."""
    stream, library, spec, _ = demo
    trajectory = generate(stream, library, spec, ["dpad_up:press"])
    from gamepad_vla.demogen import validate

    class ControlledBackend:
        name = "sapien"
        def __init__(self):
            self.spec = spec
            self.tick = self.index = 0
            self.scene = SimpleNamespace(get_timestep=lambda: float(np.float32(.01)))
            self.gamepad = SimpleNamespace(gamepad_state=self.state)
            self.closed = False

        def apply(self, commands):
            self.index = round(commands[0].stamp*trajectory.fps)

        def step(self, dt):
            self.tick += 1

        def state(self):
            signal = trajectory.target_signal[self.index].copy()
            mask = np.ones(32, bool)
            if fault == "miss":
                signal[:] = 0
            if fault == "unintended" and self.tick == 2:
                signal[SDL3_NAMES.index("south")] = 1
            if fault == "unavailable":
                mask[SDL3_NAMES.index("dpad_up")] = False
            return SimpleNamespace(values=signal, available_mask=mask)

        def observe(self):
            row = stream.rows[0]
            state = self.state()
            return Observation(self.tick*.01, 123456+self.tick, trajectory.hand_q[self.index],
                trajectory.arm_q[self.index], np.asarray(row["palm_pose_world"]),
                np.asarray(row["fingertip_pose_world"]), state.values, state.available_mask,
                {}, False, "controlled_test_adapter")

        def close(self):
            self.closed = True

    backend = ControlledBackend()
    monkeypatch.setattr(validate, "create_backend", lambda *a, **k: backend)
    monkeypatch.setattr(validate, "contact_samples", lambda b: [
        (-.001 if fault == "penetration" and b.tick == 2 else .001, ["hand", "table"])])
    def qpos(b):
        q = trajectory.arm_q[b.index].copy()
        if fault == "posture" and b.tick == 2:
            q[0, 1] = 2
        return q
    monkeypatch.setattr(validate, "arm_positions", qpos)
    monkeypatch.setattr(validate.importlib.metadata, "version", lambda name: "3.0.3" if name == "sapien" else "0.4.1")
    result, observations, commands = validate.replay(trajectory)
    assert result["passed"] == (fault is None)
    assert result["substeps"] > len(observations) == len(commands)
    assert result["substeps"] == int(len(observations)/trajectory.fps/.01+1e-7)
    assert backend.closed
    if fault == "penetration":
        assert result["counts"]["penetration_substeps"] == 1
    if fault == "posture":
        assert result["counts"]["natural_invalid_samples"] == 1
    if fault == "unintended":
        assert result["counts"]["signal_mismatch_substeps"] == 1
