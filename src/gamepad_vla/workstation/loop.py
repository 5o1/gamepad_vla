"""Deterministic fixed-rate workstation collection loop."""

import math


def run_loop(source, backend, recorder, *, rate_hz, duration_s, clock, target_sequence=None):
    """Poll, apply, step, observe, and record on a fixed tick grid.

    ``clock`` supplies ``monotonic()`` and ``sleep(seconds)``; tests inject a
    clock whose sleep advances virtual time immediately.
    """
    if rate_hz <= 0:
        raise ValueError("rate_hz must be positive")
    if duration_s < 0:
        raise ValueError("duration_s must be non-negative")
    dt = 1.0 / float(rate_hz)
    count = math.ceil(float(duration_s) * float(rate_hz))
    start = clock.monotonic()
    backend.reset()
    for tick in range(count):
        deadline = start + tick * dt
        delay = deadline - clock.monotonic()
        if delay > 0:
            clock.sleep(delay)
        now = clock.monotonic()
        commands = source.poll(now)
        backend.apply(commands)
        backend.step(dt)
        observation = backend.observe()
        target = None if target_sequence is None else target_sequence.hold_at(tick * dt)
        if recorder is not None:
            recorder.record(tick * dt, observation, commands, target)
    return {"ticks": count, "rate_hz": float(rate_hz), "duration_s": float(duration_s)}
