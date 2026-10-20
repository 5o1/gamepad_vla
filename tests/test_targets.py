import json

import numpy as np
import pytest

from gamepad_vla import signals, targets

CLOCK = "monotonic-local:test"


def _tick(tick, scheduled_ns, pressed=(), clock=CLOCK):
    report = [0.0] * 20
    for name in pressed:
        report[signals.XINPUT_NAMES.index(name)] = 1.0
    # Field set copied from resnake_gym realtime._executed_tick.
    return {"type": "tick", "tick": tick, "clock_id": clock, "scheduled_ns": scheduled_ns,
            "applied_ns": scheduled_ns + 800_000, "state_captured_ns": scheduled_ns + 1_500_000,
            "sequence": None, "chunk_index": None, "report": report, "command_received_ns": None,
            "observation_captured_ns": None, "reward": 0.0, "perturbation_events": []}


def _write(path, rows):
    path.write_text("".join(json.dumps(row) + "\n" for row in rows))
    return path


@pytest.fixture
def timeline(tmp_path):
    t0 = 2_119_326_070_577_913
    return _write(tmp_path / "timeline.jsonl", [
        _tick(0, t0),
        {"type": "command", "received_ns": t0 + 1, "packet": {}, "result": {"accepted": True}},
        _tick(1, t0 + 100_000_000, ["dpad_up"]),
        _tick(2, t0 + 200_000_000, ["dpad_left"]),
        {"type": "summary", "kind": "finished", "clock_id": CLOCK},
    ])


def test_load_and_hold(timeline):
    sequence = targets.load_timeline(timeline)
    np.testing.assert_allclose(sequence.times_s, [0.0, 0.1, 0.2])
    held = signals.dpad_targets(sequence.hold_at([-0.01, 0.05, 0.1, 0.199, 0.5]))
    np.testing.assert_array_equal(held.argmax(axis=-1) + held.any(axis=-1), [0, 0, 1, 1, 3])
    assert sequence.hold_at(0.15).shape == (20,)


def test_resample_covers_last_tick_period(timeline):
    grid, reports = targets.load_timeline(timeline).resample(60)
    assert len(grid) == 18  # 0.3 s at 60 Hz
    assert signals.dpad_targets(reports[-1])[2]


def test_dpad_only():
    reports = np.zeros((3, 20), dtype=np.float32)
    reports[1, signals.XINPUT_NAMES.index("dpad_up")] = 1.0
    reports[2, signals.XINPUT_NAMES.index("a")] = 1.0
    np.testing.assert_array_equal(targets.dpad_only(reports), [True, True, False])


@pytest.mark.parametrize("rows, message", [
    ([_tick(0, 10), _tick(1, 10)], "strictly increasing"),
    ([_tick(0, 10), _tick(1, 20, clock="other")], "several clocks"),
    ([{"type": "summary"}], "no tick rows"),
])
def test_rejects_bad_timelines(tmp_path, rows, message):
    with pytest.raises(ValueError, match=message):
        targets.load_timeline(_write(tmp_path / "bad.jsonl", rows))
