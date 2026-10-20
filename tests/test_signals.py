import importlib.util

import numpy as np
import pytest

from gamepad_vla import signals


def test_dpad5_reports_map_by_name():
    reports = np.zeros((5, 20), dtype=np.float32)
    for category, button in enumerate(range(4), start=1):
        reports[category, button] = 1.0
    out, mask = signals.xinput_to_sdl3(reports, sdl_y_down_positive=True)
    for row, name in enumerate(signals.DPAD_NAMES, start=1):
        expected = np.zeros(32, dtype=np.float32)
        expected[signals.SDL3_NAMES.index(name)] = 1.0
        np.testing.assert_array_equal(out[row], expected)
    assert not out[0].any()
    assert mask.sum() == 20 and not mask[signals.SDL3_NAMES.index("guide")]


def test_face_buttons_and_stick_sign():
    report = np.zeros(20, dtype=np.float32)
    report[signals.XINPUT_NAMES.index("a")] = 1.0
    report[signals.XINPUT_NAMES.index("ly")] = 0.5
    down, _ = signals.xinput_to_sdl3(report, sdl_y_down_positive=True)
    up, _ = signals.xinput_to_sdl3(report, sdl_y_down_positive=False)
    assert down[signals.SDL3_NAMES.index("south")] == 1.0
    assert down[signals.SDL3_NAMES.index("left_y")] == -0.5
    assert up[signals.SDL3_NAMES.index("left_y")] == 0.5


def test_sign_convention_must_be_explicit():
    with pytest.raises(TypeError):
        signals.xinput_to_sdl3(np.zeros(20), sdl_y_down_positive=None)


def test_dpad_targets():
    reports = np.zeros((2, 20), dtype=np.float32)
    reports[1, signals.XINPUT_NAMES.index("dpad_left")] = 1.0
    np.testing.assert_array_equal(
        signals.dpad_targets(reports), [[False] * 4, [False, False, True, False]])


@pytest.mark.skipif(importlib.util.find_spec("resnake_gym") is None,
                    reason="resnake_gym not installed")
def test_names_match_resnake_gym():
    from resnake_gym import gamepad
    assert gamepad.LAYOUT_VERSION == signals.XINPUT_LAYOUT_ID
    assert gamepad.LAYOUT == signals.XINPUT_NAMES


@pytest.mark.skipif(importlib.util.find_spec("xbox_gamepad_physics") is None,
                    reason="xbox_gamepad_physics not installed")
def test_names_match_xbox_gamepad_physics():
    from xbox_gamepad_physics import state
    assert state.TENSOR_LAYOUT_ID == signals.SDL3_LAYOUT_ID
    assert state.CHANNEL_NAMES == signals.SDL3_NAMES
