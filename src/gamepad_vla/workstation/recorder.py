"""Portable workstation episode writer and reader using JSON plus NumPy NPZ."""

import argparse
import json
import warnings
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from gamepad_vla.workstation.commands import ArmCommand, HandCommand
from gamepad_vla.workstation.g20_mapping import (
    G20_DEFAULT_MOTOR_POSITION,
    G20_JOINT_NAMES,
    joint_positions_to_motor_positions,
)
from gamepad_vla.workstation.spec import spec_to_dict


class EpisodeRecorder:
    """Flush each tick to a journal; export the compatible NPZ at close."""

    def __init__(self, root, episode_id, *, spec, backend_names, source_description,
                 rates, provenance):
        self.path = Path(root) / episode_id
        try:
            self.path.mkdir(parents=True, exist_ok=False)
        except FileExistsError as exc:
            raise FileExistsError(f"episode directory already exists: {self.path}") from exc
        self.meta = {
            "schema_version": 1,
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "spec": spec_to_dict(spec),
            "backend_names": list(backend_names),
            "source_description": str(source_description),
            "rates": dict(rates),
            "provenance": dict(provenance),
            "status": "recording",
        }
        (self.path / "meta.json").write_text(json.dumps(self.meta, indent=2) + "\n")
        self._commands = (self.path / "commands.jsonl").open("x", buffering=1)
        self._steps = (self.path / "steps.jsonl").open("x", buffering=1)
        self._last_motor = list(G20_DEFAULT_MOTOR_POSITION)
        self._closed = False

    def record(self, t, observation, commands, target_report=None):
        """Record one observation row and every command applied at this tick."""
        for command in commands:
            row = _command_row(float(t), command)
            self._commands.write(json.dumps(row, separators=(",", ":")) + "\n")
            if isinstance(command, HandCommand):
                self._last_motor = joint_positions_to_motor_positions(
                    command.side, command.joint_positions)
        hand_q = [observation.hand_joint_positions.get(name, np.nan) for name in G20_JOINT_NAMES]
        arm_q = observation.arm_joint_positions if observation.arm_joint_positions is not None else [np.nan] * 7
        hand_root = observation.hand_root_pose_world if observation.hand_root_pose_world is not None else [np.nan] * 7
        signal = observation.gamepad_signal if observation.gamepad_signal is not None else [np.nan] * 32
        mask = observation.gamepad_mask if observation.gamepad_mask is not None else [False] * 32
        target = target_report if target_report is not None else [0.0] * 20
        # Nulls represent unavailable measurements in portable JSON. NumPy
        # converts them back to NaN when exporting the floating-point columns.
        def numbers(values):
            return [float(v) if np.isfinite(v) else None for v in values]
        row = dict(t=float(t), hand_q=numbers(hand_q),
                   hand_cmd_motor=numbers(self._last_motor), arm_q=numbers(arm_q),
                   hand_root_pose=numbers(hand_root), gamepad_signal=numbers(signal),
                   gamepad_mask=[bool(v) for v in mask], target_report=numbers(target))
        self._steps.write(json.dumps(row, allow_nan=False, separators=(",", ":")) + "\n")

    def close(self):
        if self._closed:
            return
        self._commands.close()
        self._steps.close()
        _finalize(self.path, "complete")
        self._closed = True

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()


def _command_row(t, command):
    if isinstance(command, HandCommand):
        return {"t": t, "kind": "hand", "side": command.side,
                "joint_positions": command.joint_positions, "stamp": command.stamp}
    if isinstance(command, ArmCommand):
        return {"t": t, "kind": "arm", "side": command.side,
                "positions": list(command.positions), "stamp": command.stamp}
    raise TypeError(f"unsupported command type {type(command).__name__}")


def read_episode(path):
    """Return an episode's metadata and in-memory array dictionary."""
    path = Path(path)
    meta = json.loads((path / "meta.json").read_text())
    if (path / "steps.npz").is_file():
        with np.load(path / "steps.npz", allow_pickle=False) as archive:
            arrays = {name: archive[name] for name in archive.files}
    else:
        arrays = _journal_arrays(path)
    return meta, arrays


def read_jsonl(path):
    """Read complete records, accepting only a torn final line after interruption."""
    path = Path(path)
    with path.open() as stream:
        for number, line in enumerate(stream, 1):
            try:
                yield json.loads(line)
            except json.JSONDecodeError as exc:
                if not line.endswith("\n"):
                    warnings.warn(f"{path}:{number}: ignoring incomplete final record", RuntimeWarning)
                    break
                raise ValueError(f"{path}:{number}: invalid JSON record") from exc


def _journal_arrays(path):
    rows = list(read_jsonl(path / "steps.jsonl"))
    arrays = {"t": np.asarray([row["t"] for row in rows], dtype=np.float64)}
    for name, width, dtype in (
        ("hand_q", 21, np.float64), ("hand_cmd_motor", 20, np.float64),
        ("arm_q", 7, np.float64), ("hand_root_pose", 7, np.float64),
        ("gamepad_signal", 32, np.float32), ("gamepad_mask", 32, bool),
        ("target_report", 20, np.float32),
    ):
        arrays[name] = np.asarray([row[name] for row in rows], dtype=dtype).reshape(-1, width)
    return arrays


def _finalize(path, status):
    arrays = _journal_arrays(path)
    temporary = path / "steps.npz.tmp"
    with temporary.open("wb") as stream:
        np.savez_compressed(stream, **arrays)
    temporary.replace(path / "steps.npz")
    meta = json.loads((path / "meta.json").read_text())
    meta.update(status=status, rows=len(arrays["t"]))
    temporary = path / "meta.json.tmp"
    temporary.write_text(json.dumps(meta, indent=2) + "\n")
    temporary.replace(path / "meta.json")
    return len(arrays["t"])


def recover_episode(path):
    """Finalize the complete journal records of an interrupted, stopped capture."""
    path = Path(path)
    if (path / "steps.npz").exists():
        meta, arrays = read_episode(path)
        return len(arrays["t"])
    return _finalize(path, "recovered")


def main():
    parser = argparse.ArgumentParser(description="Recover a stopped capture from its steps.jsonl journal.")
    parser.add_argument("episode", type=Path)
    args = parser.parse_args()
    print(json.dumps({"episode": str(args.episode), "rows": recover_episode(args.episode)}))


Recorder = EpisodeRecorder


if __name__ == "__main__":
    main()
