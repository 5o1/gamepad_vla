"""Timed target reports from upstream ``resnake_gym`` realtime timelines.

``resnake_gym``'s realtime runner writes ``timeline.jsonl`` (src/resnake_gym/realtime.py
``_executed_tick``): one ``{"type": "tick", ...}`` row per game tick carrying the
executed 20-dim ``xinput-normalized-v1`` report and the tick's ``scheduled_ns`` on
one ``clock_id``, interleaved with ``command`` and ``summary`` rows. A report is
held until the next tick, so targets are a zero-order hold over tick times.
"""

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from gamepad_vla import signals

_NON_DPAD = np.array([name not in signals.DPAD_NAMES for name in signals.XINPUT_NAMES])


@dataclass(frozen=True)
class TargetSequence:
    """``reports[i]`` is held from ``times_s[i]`` (seconds since the first tick)."""

    times_s: np.ndarray
    reports: np.ndarray
    clock_id: str

    def hold_at(self, times_s):
        """Reports in force at ``times_s``; neutral before the first tick."""
        times_s = np.asarray(times_s, dtype=float)
        index = np.searchsorted(self.times_s, times_s.ravel(), side="right") - 1
        out = np.zeros((index.size, len(signals.XINPUT_NAMES)), dtype=np.float32)
        out[index >= 0] = self.reports[index[index >= 0]]
        return out.reshape(*times_s.shape, len(signals.XINPUT_NAMES))

    def resample(self, rate_hz, duration_s=None):
        """Zero-order-hold reports on a ``rate_hz`` grid from 0 to ``duration_s``.

        The default duration ends at the last tick plus one median tick period.
        """
        if duration_s is None:
            period = float(np.median(np.diff(self.times_s))) if len(self.times_s) > 1 else 0.0
            duration_s = self.times_s[-1] + period
        grid = np.arange(int(round(duration_s * rate_hz))) / rate_hz
        return grid, self.hold_at(grid)


def load_timeline(path):
    """Read the tick rows of a ``timeline.jsonl`` into a ``TargetSequence``."""
    ticks = []
    for number, line in enumerate(Path(path).read_text().splitlines(), start=1):
        row = json.loads(line)
        if row.get("type") == "tick":
            if len(row["report"]) != len(signals.XINPUT_NAMES):
                raise ValueError(f"{path}:{number}: report has {len(row['report'])} values")
            ticks.append(row)
    if not ticks:
        raise ValueError(f"{path}: no tick rows")
    clocks = {row["clock_id"] for row in ticks}
    if len(clocks) != 1:
        raise ValueError(f"{path}: ticks span several clocks {sorted(clocks)}")
    scheduled = np.array([row["scheduled_ns"] for row in ticks], dtype=np.int64)
    if np.any(np.diff(scheduled) <= 0):
        raise ValueError(f"{path}: scheduled_ns is not strictly increasing")
    return TargetSequence(
        times_s=(scheduled - scheduled[0]) / 1e9,
        reports=np.array([row["report"] for row in ticks], dtype=np.float32),
        clock_id=clocks.pop(),
    )


def dpad_only(reports):
    """True per row when no non-D-pad channel is active and at most one direction is held."""
    reports = np.asarray(reports)
    return ~np.any(reports[..., _NON_DPAD] != 0, axis=-1) & (signals.dpad_targets(reports).sum(axis=-1) <= 1)
