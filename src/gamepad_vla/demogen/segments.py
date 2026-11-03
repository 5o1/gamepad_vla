"""Signal-derived half-open intervals, with explicit ambiguity annotations."""
from dataclasses import asdict, dataclass, field

import numpy as np

from gamepad_vla.signals import SDL3_NAMES


@dataclass(frozen=True)
class SliceConfig:
    on: float = .30
    off: float = .15
    debounce_frames: int = 2
    bridge_frames: int = 1
    before_s: float = .15
    after_s: float = .20
    grip_min_s: float = .30
    grip_arm_range_rad: float = .02
    grip_hand_range_rad: float = .03

    def __post_init__(self):
        values = [self.on, self.off, self.before_s, self.after_s, self.grip_min_s,
                  self.grip_arm_range_rad, self.grip_hand_range_rad]
        if not np.isfinite(values).all() or not 0 <= self.off < self.on <= 1:
            raise ValueError("slice thresholds require 0 <= off < on <= 1")
        if min(values[2:]) < 0 or self.debounce_frames < 1 or self.bridge_frames < 0:
            raise ValueError("invalid slice durations or debounce")


@dataclass
class Segment:
    source: str
    side: str
    control: str
    start: int
    stop: int
    active_start: int
    active_stop: int
    phases: dict
    stats: dict
    truncated: bool = False
    overlaps: list = field(default_factory=list)

    def to_dict(self):
        return asdict(self)


def runs(active):
    edges = np.diff(np.r_[False, np.asarray(active, bool), False].astype(int))
    return list(zip(map(int, np.flatnonzero(edges == 1)), map(int, np.flatnonzero(edges == -1))))


def active_mask(values, config):
    held = False
    active = []
    for value in values:
        held = value >= (config.off if held else config.on)
        active.append(held)
    active = np.array(active, bool)
    for start, stop in runs(~active):
        if start > 0 and stop < len(active) and stop-start <= config.bridge_frames:
            active[start:stop] = True
    for start, stop in runs(active):
        if stop-start < config.debounce_frames:
            active[start:stop] = False
    return active


def controls(side):
    if side == "left":
        digital = ("dpad_up", "dpad_down", "dpad_left", "dpad_right", "left_stick",
                   "left_shoulder", "back", "guide")
    elif side == "right":
        digital = ("south", "east", "west", "north", "right_stick", "right_shoulder", "start")
    else:
        raise ValueError("side must be left or right")
    return [(name, (SDL3_NAMES.index(name),), "digital") for name in digital] + [
        (side+"_trigger", (SDL3_NAMES.index(side+"_trigger"),), "trigger"),
        (side+"_stick", tuple(SDL3_NAMES.index(side+"_"+axis) for axis in ("x", "y")), "stick")]


def slice_stream(stream, config=None):
    config = config or SliceConfig()
    t, signal, mask = stream.time, stream.signal, stream.mask
    if len(t) < 2:
        raise ValueError("slicing needs at least two frames")
    segments = []
    any_active = np.zeros(len(t), bool)
    for name, channels, kind in controls(stream.side):
        available = mask[:, channels].all(axis=1)
        values = signal[:, channels]
        magnitude = np.linalg.norm(values, axis=1) if kind == "stick" else values[:, 0]
        active = active_mask(np.where(available, magnitude, 0), config) & available
        any_active |= active
        for a, b in runs(active):
            start = int(np.searchsorted(t, t[a]-config.before_s))
            stop = int(np.searchsorted(t, t[b-1]+config.after_s, side="right"))
            peak = float(magnitude[a:b].max())
            high = np.flatnonzero(magnitude[a:b] >= .9*peak)+a
            h0, h1 = int(high[0]), int(high[-1]+1)
            label = name + (":press" if kind == "digital" else ":travel")
            stats = dict(kind=kind, peak=peak, mean=float(magnitude[a:b].mean()),
                         duration_s=float(t[b-1]-t[a]), channels=list(channels),
                         available=bool(available[start:stop].all()))
            if kind == "stick":
                # Directions describe recorded axis signs, not an assumed SDL Y convention.
                angles = np.unwrap(np.arctan2(values[a:b, 1], values[a:b, 0]))
                sweep = float(angles[-1]-angles[0])
                if abs(sweep) >= 1.5*np.pi:
                    suffix = "circle_positive" if sweep > 0 else "circle_negative"
                else:
                    bins = np.rint(angles/(np.pi/4)).astype(int) % 8
                    suffix = "dir"+str(int(np.bincount(bins, minlength=8).argmax()))
                label = name+":"+suffix
                stats.update(sweep_rad=sweep, direction_histogram=np.bincount(
                    np.rint(angles/(np.pi/4)).astype(int) % 8, minlength=8).tolist(),
                    amplitude_range=[float(magnitude[a:b].min()), peak])
            elif kind == "trigger":
                stats["travel_range"] = [float(magnitude[a:b].min()), peak]
            phases = dict(approach=[start, a], press=[a, h0], hold=[h0, h1],
                          release=[h1, b], return_to_grip=[b, stop])
            segments.append(Segment(stream.source, stream.side, label, start, stop, a, b,
                phases, stats, truncated=bool(a == 0 or b == len(t) or
                t[a]-t[0] < config.before_s or t[-1]-t[b-1] < config.after_s)))
    segments.sort(key=lambda s: (s.active_start, s.control))
    # Trim padding halfway between sequential operations; simultaneous active
    # ranges remain marked, rather than silently becoming independent examples.
    for i, segment in enumerate(segments):
        for other in segments[i+1:]:
            if other.active_start >= segment.active_stop:
                if segment.stop > other.start:
                    middle = (segment.active_stop+other.active_start)//2
                    segment.stop = min(segment.stop, middle)
                    other.start = max(other.start, middle)
                break
            segment.overlaps.append(other.control)
            other.overlaps.append(segment.control)
    for segment in segments:
        segment.phases["approach"][0] = segment.start
        segment.phases["return_to_grip"][1] = segment.stop
    # A neutral, stable interval is a grip candidate, not a contact certificate.
    globally_neutral = (np.abs(signal)*mask).max(axis=1) < config.off
    for a, b in runs(~any_active & globally_neutral & mask.any(axis=1)):
        # Approach/release can be neutral too. Partition by the range of the
        # whole candidate, so a moving prefix does not hide a later stable hold
        # and slow cumulative drift cannot pass a per-frame velocity shortcut.
        i = a
        while i < b:
            amin, amax = stream.arm_q[i].copy(), stream.arm_q[i].copy()
            hmin, hmax = stream.hand_q[i].copy(), stream.hand_q[i].copy()
            j = i+1
            while j < b:
                next_amin, next_amax = np.minimum(amin, stream.arm_q[j]), np.maximum(amax, stream.arm_q[j])
                next_hmin, next_hmax = np.minimum(hmin, stream.hand_q[j]), np.maximum(hmax, stream.hand_q[j])
                if (np.max(next_amax-next_amin) > config.grip_arm_range_rad or
                        np.max(next_hmax-next_hmin) > config.grip_hand_range_rad):
                    break
                amin, amax, hmin, hmax = next_amin, next_amax, next_hmin, next_hmax
                j += 1
            if t[j-1]-t[i] >= config.grip_min_s:
                segments.append(Segment(stream.source, stream.side, "grip", i, j, i, j,
                    {"hold": [i, j]}, {"kind": "grip_candidate", "duration_s": float(t[j-1]-t[i])}))
            i = j
    return segments
