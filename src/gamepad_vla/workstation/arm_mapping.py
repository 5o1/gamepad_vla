"""Port of rokae_dual_impedance_teleop_node target path (not state path).

Source snapshot: node.cpp ClampToJointLimits/ClampAroundInitial, map_left/right,
handle_left/right. Physical RT filtering/deadband is not action_joint_target_final.
"""
import numpy as np
from gamepad_vla.workstation.commands import ArmCommand

VERSION = "rokae-target-map-snapshot-20261014-v1"
LOW_DEG = np.array([-178., -120., -178., -60., -178., -60., -60.])
HIGH_DEG = np.array([178., 120., 178., 145., 178., 60., 60.])
OFFSETS = {"left": np.array([90., 90., -90., 0., 0., 0., 0.]),
           "right": np.array([-90., 90., 90., 0., 0., 0., 0.])}


class ArmMapper:
    def __init__(self, startup_rad, *, margin_deg=3., max_delta_deg=60., limit_around_initial=True):
        self.initial = {side: np.rad2deg(self._vector(startup_rad[side])) for side in OFFSETS}
        self.margin = float(margin_deg)
        self.max_delta = float(max_delta_deg)
        self.limit_around_initial = bool(limit_around_initial)
        if not 0 <= self.margin < 60 or not np.isfinite(self.max_delta) or self.max_delta < 0:
            raise ValueError("invalid arm mapping limits")

    @staticmethod
    def _vector(value):
        a = np.asarray(value, float)
        if a.shape != (7,) or not np.isfinite(a).all():
            raise ValueError("arm input must contain 7 finite values")
        return a

    def command(self, side, raw_deg, stamp):
        raw = self._vector(raw_deg)
        mapped = raw + OFFSETS[side]
        limited = np.clip(mapped, LOW_DEG + self.margin, HIGH_DEG - self.margin)
        absolute = (limited != mapped).tolist()
        before_startup = limited.copy()
        if self.limit_around_initial:
            limited = np.clip(limited, self.initial[side] - self.max_delta,
                              self.initial[side] + self.max_delta)
        return ArmCommand(side, tuple(np.deg2rad(limited)), float(stamp), tuple(raw),
                          dict(absolute=absolute, startup=(limited != before_startup).tolist()))

    def metadata(self):
        return dict(version=VERSION, startup_rad={s: np.deg2rad(v).tolist() for s, v in self.initial.items()},
                    margin_deg=self.margin, max_delta_deg=self.max_delta,
                    limit_around_initial=self.limit_around_initial)
