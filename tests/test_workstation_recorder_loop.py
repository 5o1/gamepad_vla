from pathlib import Path

import numpy as np
import pytest

from gamepad_vla.targets import TargetSequence
from gamepad_vla.workstation.backends import NullBackend, Observation
from gamepad_vla.workstation.commands import ArmCommand, HandCommand
from gamepad_vla.workstation.g20_mapping import G20_JOINT_NAMES
from gamepad_vla.workstation.loop import run_loop
from gamepad_vla.workstation.recorder import EpisodeRecorder, read_episode
from gamepad_vla.workstation.sources import ReplaySource, ScriptedSource
from gamepad_vla.workstation.spec import load_spec


CONFIG = Path(__file__).parents[1] / "configs/workcell/station_left_g20_v3.json"


def _hand(stamp=1.0):
    return HandCommand("left", {name: 0.1 for name in G20_JOINT_NAMES}, stamp)


def test_episode_layout_arrays_and_replay(tmp_path):
    recorder = EpisodeRecorder(
        tmp_path, "episode", spec=load_spec(CONFIG), backend_names=["null"],
        source_description="test", rates={"loop_hz": 60}, provenance={"case": "unit"})
    observation = Observation(1.0, _hand().joint_positions, None, None, None, None, "null")
    commands = [_hand(), ArmCommand("left", (0.0,) * 7, 1.0)]
    recorder.record(0.0, observation, commands)
    recorder.close()
    meta, arrays = read_episode(recorder.path)
    assert meta["schema_version"] == 1 and meta["provenance"] == {"case": "unit"}
    assert arrays["hand_q"].shape == (1, 21)
    assert arrays["hand_cmd_motor"].shape == (1, 20)
    assert arrays["arm_q"].shape == (1, 7) and np.isnan(arrays["arm_q"]).all()
    assert arrays["gamepad_mask"].dtype == bool
    assert arrays["target_report"].shape == (1, 20)
    replay = ReplaySource(recorder.path)
    assert replay.poll(5.0) == commands
    with pytest.raises(FileExistsError):
        EpisodeRecorder(tmp_path, "episode", spec=load_spec(CONFIG), backend_names=[],
                        source_description="", rates={}, provenance={})


class FakeClock:
    def __init__(self):
        self.now = 20.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class CaptureRecorder:
    def __init__(self):
        self.rows = []

    def record(self, t, observation, commands, target):
        self.rows.append((t, observation, commands, target))


def test_fixed_rate_loop_uses_fake_clock_and_target_hold():
    clock = FakeClock()
    source = ScriptedSource([(0.0, _hand())])
    recorder = CaptureRecorder()
    reports = np.zeros((2, 20), dtype=np.float32)
    reports[1, 0] = 1
    sequence = TargetSequence(np.array([0.0, 0.15]), reports, "test")
    summary = run_loop(source, NullBackend(), recorder, rate_hz=10, duration_s=0.3,
                       clock=clock, target_sequence=sequence)
    assert summary["ticks"] == 3
    np.testing.assert_allclose([row[0] for row in recorder.rows], [0.0, 0.1, 0.2])
    assert recorder.rows[-1][3][0] == 1
    assert clock.now == pytest.approx(20.2)
