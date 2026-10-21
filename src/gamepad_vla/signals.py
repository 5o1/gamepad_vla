"""Name-based conversion between upstream game reports and gamepad signals.

Upstream ``resnake_gym`` emits ``xinput-normalized-v1``: float32[20], 14 held
buttons then lx, ly, rx, ry, lt, rt, with positive stick Y meaning up.
``xbox_gamepad_physics`` emits ``sdl3-gamepad-core-v1``: float32[32], the six
SDL3 axes then the 26 SDL3 buttons, plus availability and task masks.
Channel names below are copied from those two modules (resnake_gym
src/resnake_gym/gamepad.py; xbox_gamepad_physics src/xbox_gamepad_physics/state.py,
unchanged through release v0.3.1); tests check them against the installed packages
when present.

SDL reports stick Y as down-positive, and resnake_gym negates it on input
(src/resnake_gym/sdl_gamepad.py). xbox_gamepad_physics does not document its Y
sign, so callers must state it; nothing is assumed.
"""

import numpy as np

XINPUT_LAYOUT_ID = "xinput-normalized-v1"
XINPUT_NAMES = (
    "dpad_up", "dpad_down", "dpad_left", "dpad_right", "start", "back",
    "left_thumb", "right_thumb", "lb", "rb", "a", "b", "x", "y",
    "lx", "ly", "rx", "ry", "lt", "rt",
)

SDL3_LAYOUT_ID = "sdl3-gamepad-core-v1"
SDL3_NAMES = (
    "left_x", "left_y", "right_x", "right_y", "left_trigger", "right_trigger",
    "south", "east", "west", "north", "back", "guide", "start", "left_stick",
    "right_stick", "left_shoulder", "right_shoulder", "dpad_up", "dpad_down",
    "dpad_left", "dpad_right", "misc1", "right_paddle1", "left_paddle1",
    "right_paddle2", "left_paddle2", "touchpad", "misc2", "misc3", "misc4",
    "misc5", "misc6",
)

# Xbox face buttons A/B/X/Y sit at SDL's positional south/east/west/north.
XINPUT_TO_SDL3 = {
    "dpad_up": "dpad_up",
    "dpad_down": "dpad_down",
    "dpad_left": "dpad_left",
    "dpad_right": "dpad_right",
    "start": "start",
    "back": "back",
    "left_thumb": "left_stick",
    "right_thumb": "right_stick",
    "lb": "left_shoulder",
    "rb": "right_shoulder",
    "a": "south",
    "b": "east",
    "x": "west",
    "y": "north",
    "lx": "left_x",
    "ly": "left_y",
    "rx": "right_x",
    "ry": "right_y",
    "lt": "left_trigger",
    "rt": "right_trigger",
}
_STICK_Y = frozenset(("ly", "ry"))
DPAD_NAMES = ("dpad_up", "dpad_down", "dpad_left", "dpad_right")


def xinput_to_sdl3(reports, *, sdl_y_down_positive):
    """Map ``[..., 20]`` xinput reports to ``[..., 32]`` signals and a task mask.

    The task mask marks the 20 SDL3 channels that the upstream layout can
    express; it must still be intersected with the simulator's availability.
    """
    if not isinstance(sdl_y_down_positive, bool):
        raise TypeError("sdl_y_down_positive must be stated explicitly as a bool")
    reports = np.asarray(reports, dtype=np.float32)
    if reports.shape[-1:] != (len(XINPUT_NAMES),) or not np.isfinite(reports).all():
        raise ValueError("reports must be finite with a trailing dimension of 20")
    signals = np.zeros(reports.shape[:-1] + (len(SDL3_NAMES),), dtype=np.float32)
    task_mask = np.zeros(len(SDL3_NAMES), dtype=bool)
    for source, name in enumerate(XINPUT_NAMES):
        target = SDL3_NAMES.index(XINPUT_TO_SDL3[name])
        value = reports[..., source]
        if name in _STICK_Y and sdl_y_down_positive:
            value = -value
        signals[..., target] = value
        task_mask[target] = True
    return signals, task_mask


def dpad_targets(reports):
    """Return held D-pad states ``[..., 4]`` (up, down, left, right) from xinput reports."""
    reports = np.asarray(reports, dtype=np.float32)
    if reports.shape[-1:] != (len(XINPUT_NAMES),):
        raise ValueError("reports must have a trailing dimension of 20")
    return reports[..., [XINPUT_NAMES.index(name) for name in DPAD_NAMES]] > 0.5


def sdl3_to_xinput(values, *, sdl_y_down_positive, digital_threshold=None):
    """Map physical canonical32 values to complete upstream 20-value reports.

    With ``digital_threshold`` the fourteen held buttons become 0/1, matching
    the consuming game's button interface; six axes remain continuous. This is
    a state conversion, never a substitution of requested target reports.
    """
    if not isinstance(sdl_y_down_positive, bool):
        raise TypeError('sdl_y_down_positive must be stated explicitly as a bool')
    values = np.asarray(values,dtype=np.float32)
    if values.shape[-1:] != (len(SDL3_NAMES),) or not np.isfinite(values).all():
        raise ValueError('values must be finite with a trailing dimension of 32')
    reports = np.zeros(values.shape[:-1]+(len(XINPUT_NAMES),),dtype=np.float32)
    for index,name in enumerate(XINPUT_NAMES):
        value = values[...,SDL3_NAMES.index(XINPUT_TO_SDL3[name])]
        reports[...,index] = -value if name in _STICK_Y and sdl_y_down_positive else value
    if digital_threshold is not None:
        if not 0 <= digital_threshold <= 1:
            raise ValueError('digital threshold must be in [0,1]')
        reports[...,:14] = reports[...,:14] > digital_threshold
    return reports
