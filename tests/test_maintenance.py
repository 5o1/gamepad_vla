import json
import os
from pathlib import Path
import runpy
import subprocess
import sys
import tarfile

import numpy as np
import pytest

from gamepad_vla.provenance import code_provenance
from gamepad_vla.workstation.appearance import load_appearance
from gamepad_vla.workstation.recorder import read_episode, recover_episode
from gamepad_vla.workstation.sources import ReplaySource
from gamepad_vla.workstation.spec import load_spec


ROOT = Path(__file__).resolve().parents[1]


def test_adopted_scene_preserves_calibration_and_uses_two_right_models(monkeypatch):
    monkeypatch.delenv("GAMEPAD_VLA_WORKSPACE", raising=False)
    spec = load_spec(ROOT / "configs/workcell/station_g20_v4_4.json")
    previous = load_spec(ROOT / "configs/workcell/station_g20_v4_3.json")
    assert spec.cameras == previous.cameras
    assert spec.table == previous.table and spec.gamepad == previous.gamepad
    for side in ("left", "right"):
        arm = getattr(spec, side + "_arm")
        assert "08R" in arm.base_link
        assert arm.home_q == getattr(previous, side + "_arm").home_q
        assert arm.base_pose == getattr(previous, side + "_arm").base_pose
    assert set(spec.left_arm.joint_names).isdisjoint(spec.right_arm.joint_names)


def test_workspace_override_resolves_scene_and_visual_parts(tmp_path, monkeypatch):
    monkeypatch.setenv("GAMEPAD_VLA_WORKSPACE", str(tmp_path))
    spec = load_spec(ROOT / "configs/workcell/station_g20_v4_4.json")
    assert spec.left_arm.urdf == str(tmp_path / "assets/derived/ar5_08r_e6bcf44f/left_arm.urdf")
    assert all(f.mesh.startswith(str(tmp_path)) for f in spec.fixtures if f.mesh)
    appearance = load_appearance(ROOT / "configs/appearance/station_v2.json")
    for split in appearance["visual_splits"]:
        assert split["source_mesh"].startswith(str(tmp_path))
        assert all(p["mesh"].startswith(str(tmp_path)) for p in split["parts"])


def test_killed_capture_recovers_flushed_observations_and_commands(tmp_path):
    # os._exit bypasses close/finally exactly as an abruptly killed worker does.
    script = """
import os,sys
from gamepad_vla.workstation.backends import NullBackend
from gamepad_vla.workstation.recorder import EpisodeRecorder
from gamepad_vla.workstation.spec import load_spec
from gamepad_vla.workstation.commands import ArmCommand
b=NullBackend()
r=EpisodeRecorder(sys.argv[1], 'interrupted', spec=load_spec(sys.argv[2]),
 backend_names=['null'],source_description='test',rates={},provenance={'git_commit':'test'})
for i in range(3):
 c=ArmCommand('left',(float(i),)*7,float(i))
 b.apply([c]);r.record(i,b.observe(),[c])
os._exit(23)
"""
    result = subprocess.run([sys.executable, "-c", script, str(tmp_path),
                             str(ROOT / "configs/workcell/station_g20_v4_4.json")])
    assert result.returncode == 23
    episode = tmp_path / "interrupted"
    assert not (episode / "steps.npz").exists()
    # Simulate a partially written trailing record too.
    for name in ("steps.jsonl", "commands.jsonl"):
        with (episode / name).open("a") as stream:
            stream.write('{"t":')
    with pytest.warns(RuntimeWarning, match="incomplete final record"):
        meta, arrays = read_episode(episode)
    assert meta["status"] == "recording"
    np.testing.assert_equal(arrays["arm_q"][:, 0], [0, 1, 2])
    with pytest.warns(RuntimeWarning, match="incomplete final record"):
        assert recover_episode(episode) == 3
    meta, recovered = read_episode(episode)
    assert meta["status"] == "recovered" and meta["provenance"]["git_commit"] == "test"
    np.testing.assert_equal(recovered["arm_q"], arrays["arm_q"])
    with pytest.warns(RuntimeWarning, match="incomplete final record"):
        replay = ReplaySource(episode)
    assert len(replay.poll(0)) == 1
    assert len(replay.poll(2)) == 2


def test_journal_does_not_hide_corrupt_complete_records(tmp_path):
    from gamepad_vla.workstation.recorder import read_jsonl
    path = tmp_path / "steps.jsonl"
    path.write_text('{"t":\n{"t":1}\n')
    with pytest.raises(ValueError, match="invalid JSON record"):
        list(read_jsonl(path))


def test_source_archive_respects_ignores_and_preserves_revision(tmp_path):
    root = tmp_path / "repo"
    root.mkdir()
    subprocess.run(["git", "init", "-q", str(root)], check=True)
    (root / ".gitignore").write_text("assets/\nbuild/\n.code-provenance.json\n")
    (root / "src").mkdir()
    (root / "src/example.py").write_text("x = 1\n")
    for folder in ("assets", "build"):
        (root / folder).mkdir()
        (root / folder / "large.bin").write_bytes(b"ignored")
    subprocess.run(["git", "-C", str(root), "add", "."], check=True)
    subprocess.run(["git", "-C", str(root), "-c", "user.name=Test",
                    "-c", "user.email=test@example.org", "commit", "-qm", "baseline"], check=True)
    build = runpy.run_path(str(ROOT / "scripts/build_source_archive.py"))["build_archive"]
    output = tmp_path / "source.tgz"
    provenance = build(root, output)
    assert provenance["git_commit"] and provenance["dirty"] is False
    deployed = tmp_path / "deployed"
    with tarfile.open(output) as archive:
        assert not any(n.startswith(("assets/", "build/", ".git/")) for n in archive.getnames())
        archive.extractall(deployed)
    remote = code_provenance(deployed)
    assert remote["git_commit"] == provenance["git_commit"]
    assert remote["deployment_modified"] is False
    metadata = deployed / "src/example.egg-info"
    metadata.mkdir()
    (metadata / "PKG-INFO").write_text("Generated by editable installation\n")
    assert code_provenance(deployed)["deployment_modified"] is False
    (deployed / "src/example.py").write_text("x = 2\n")
    assert code_provenance(deployed)["deployment_modified"] is True


def test_default_cli_records_code_and_configuration_provenance(tmp_path):
    subprocess.run([sys.executable, str(ROOT / "scripts/run_workstation.py"),
                    "--source", "scripted", "--duration", "0",
                    "--record-root", str(tmp_path)], check=True)
    meta = json.loads(next(tmp_path.glob("*/meta.json")).read_text())
    assert meta["spec"]["name"] == "station-g20-v4.4"
    assert len(meta["provenance"]["spec_sha256"]) == 64
    assert "git_commit" in meta["provenance"]
    assert meta["provenance"]["packages"]["numpy"]
