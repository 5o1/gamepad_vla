"""Opt-in short native replay in an existing station/compute environment."""
import importlib.util
import json
import os
from pathlib import Path

import pytest
import numpy as np

from gamepad_vla.demogen.generate import GenerateConfig, identity_trajectory
from gamepad_vla.demogen.episode import load_demonstration
from gamepad_vla.demogen.segments import slice_stream
from gamepad_vla.demogen.validate import ValidationConfig, replay
from gamepad_vla.demogen.pipeline import run
from gamepad_vla.demogen.synthetic import from_candidates, physical_from_candidates
from gamepad_vla.workstation.spec import load_spec
from gamepad_vla.workstation.episode_schema import read_episode


@pytest.mark.skipif(os.environ.get("GAMEPAD_VLA_TEST_DEMOGEN_PHYSICS") != "1",
                    reason="set GAMEPAD_VLA_TEST_DEMOGEN_PHYSICS=1 in an existing simulation environment")
def test_synthetic_native_pipeline(tmp_path):
    backend = os.environ.get("GAMEPAD_VLA_DEMOGEN_BACKEND", "sapien")
    if importlib.util.find_spec(backend) is None:
        pytest.skip(backend+" is not installed; no dependencies installed by tests")
    root = Path(__file__).resolve().parents[1]
    spec_path = root / "configs/workcell/station_g20_v4_4_raised_interim.json"
    spec = load_spec(spec_path)
    if not Path(spec.left_arm.urdf).is_file():
        pytest.skip("existing workcell assets unavailable")
    source = tmp_path / "synthetic"
    from_candidates(spec, root / "docs/reports/2026-10-23-natural-grip-fixed/candidates.json",
                    source, repetitions=1)
    result = run([source], [root / "configs/workcell/placement_trials/d050-h220-t+10.json",
                           root / "configs/workcell/placement_trials/d075-h190-t+10.json"],
                 ["dpad_up:press", "dpad_down:press", "dpad_left:press"], tmp_path / "generated",
                 count=2, seed=42, backend=backend, generation=GenerateConfig(),
                 resource_root=os.environ.get("XBOX_GAMEPAD_PHYSICS_RESOURCES"))
    assert result["status"] == "complete" and len(result["attempts"]) == 2
    assert all(entry["stage"] in ("physics", "output") and "validation" in entry for entry in result["attempts"])
    for entry in result["attempts"]:
        physical = entry["validation"]
        assert physical["substeps"] > physical["frames"] > 0
        if entry["status"] == "accepted":
            assert physical["passed"] and not physical["reasons"]
        else:
            assert physical["reasons"]
    # Success here means native validation ran and the retention rule held.
    # Zero accepted trajectories is a valid, explicitly reported outcome.
    assert len(list((tmp_path / "generated").glob("episode-*"))) == result["accepted"]


@pytest.mark.skipif(os.environ.get("GAMEPAD_VLA_TEST_DEMOGEN_PHYSICS") != "1",
                    reason="set GAMEPAD_VLA_TEST_DEMOGEN_PHYSICS=1 in an existing simulation environment")
def test_feedback_source_identity_and_accepted_relocation(tmp_path):
    if importlib.util.find_spec("sapien") is None:
        pytest.skip("SAPIEN not installed; no dependencies installed by tests")
    root = Path(__file__).resolve().parents[1]
    spec = load_spec(root / "configs/workcell/station_g20_v4_4_raised_interim.json")
    if not Path(spec.left_arm.urdf).is_file():
        pytest.skip("existing workcell assets unavailable")
    source = tmp_path / "feedback-source"
    physical_from_candidates(spec, root / "docs/reports/2026-10-23-natural-grip-fixed/candidates.json",
                             source, repetitions=1)
    calibration = tmp_path / "gamepad-contact-calibration"
    physical_from_candidates(spec, root / "docs/reports/2026-10-23-natural-grip-fixed/candidates.json",
                             calibration, repetitions=1, environment_collisions=False)
    full_meta, full_rows = read_episode(source)
    contact_meta, contact_rows = read_episode(calibration)
    assert np.allclose([r["gamepad_signal"] for r in full_rows],
                       [r["gamepad_signal"] for r in contact_rows], atol=1e-8)
    key = "dpad|thumb_distal"
    assert full_meta["feedback"]["source_min_distances_m"][key] == pytest.approx(
        contact_meta["feedback"]["source_min_distances_m"][key], abs=1e-8)
    stream = load_demonstration(source)["left"]
    identity = identity_trajectory(stream, slice_stream(stream))
    strict, _, _ = replay(identity)
    assert not strict["passed"] and strict["counts"]["signal_mismatch_substeps"] == 0
    assert strict["counts"]["natural_invalid_samples"] == 0
    assert strict["min_robot_contact_distance_m"] > -3e-5
    validation = ValidationConfig(control_contact_tolerance_m=3e-5)
    calibrated, _, _ = replay(identity, config=validation)
    assert calibrated["passed"]
    settings = json.loads((root / "configs/demogen/physical_check.json").read_text())
    result = run([source], [root / "configs/workcell/placement_trials/d050-h220-t+10.json",
                           root / "configs/workcell/placement_trials/d075-h190-t+10.json"],
                 ["dpad_down:press", "dpad_down:press"], tmp_path / "positive-generated",
                 count=2, seed=42, backend="sapien", generation=GenerateConfig(**settings["generation"]),
                 validation=validation)
    assert result["accepted"] >= 1
    for entry in result["attempts"]:
        if entry["status"] == "accepted":
            assert entry["validation"]["passed"]
            assert entry["transforms"]["ik"]["max_fingertip_error_m"] <= .001
            episode = tmp_path / "positive-generated" / entry["episode"]
            assert load_demonstration(episode)["left"].meta["physics_validated"]
