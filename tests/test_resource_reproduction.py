import hashlib
import json
from pathlib import Path
import runpy
import subprocess

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]


def test_download_requires_expected_bytes_and_checksum(tmp_path):
    fetch = runpy.run_path(str(ROOT / "scripts/fetch_resources.py"))["fetch_file"]
    source = tmp_path / "public.bin"
    source.write_bytes(b"public resource")
    entry = {"path": "asset.bin", "url": source.as_uri(), "bytes": source.stat().st_size,
             "sha256": hashlib.sha256(source.read_bytes()).hexdigest()}
    resource = {"id": "test", "destination": "assets/test"}
    workspace = tmp_path / "workspace"
    assert fetch(resource, entry, workspace)["status"] == "downloaded"
    source.unlink()
    assert fetch(resource, entry, workspace)["status"] == "verified"
    source.write_bytes(b"corrupt")
    entry["path"] = "bad.bin"
    with pytest.raises(ValueError, match="checksum/size"):
        fetch(resource, entry, workspace)
    assert not (workspace / "assets/test/bad.bin").exists()
    assert not (workspace / "assets/test/bad.bin.part").exists()


def test_git_fetch_preserves_pinned_revision_and_local_edits(tmp_path):
    source = tmp_path / "public-repo"
    source.mkdir()
    subprocess.run(["git", "init", "-q", str(source)], check=True)
    (source / "solver.py").write_text("answer = 42\n")
    subprocess.run(["git", "-C", str(source), "add", "."], check=True)
    subprocess.run(["git", "-C", str(source), "-c", "user.name=Test", "-c", "user.email=test@example.org",
                    "commit", "-qm", "solver"], check=True)
    revision = subprocess.check_output(["git", "-C", str(source), "rev-parse", "HEAD"], text=True).strip()
    fetch = runpy.run_path(str(ROOT / "scripts/fetch_resources.py"))["fetch_git"]
    resource = {"id": "solver", "repository": str(source), "revision": revision, "destination": "external/solver"}
    workspace = tmp_path / "workspace"
    assert fetch(resource, workspace)["revision"] == revision
    assert fetch(resource, workspace)["status"] == "verified"
    modified = workspace / "external/solver/solver.py"
    modified.write_text("local edits\n")
    with pytest.raises(ValueError, match="differs"):
        fetch(resource, workspace)
    assert modified.read_text() == "local edits\n"


def test_manifest_covers_complete_pinned_hand_and_arm_mesh_sets():
    manifest = json.loads((ROOT / "references/external_resources.json").read_text())
    resources = {r["id"]: r for r in manifest["resources"]}
    assert set(resources) == {"g20", "ar5_08r", "d435_shell", "gamepad", "calibration"}
    for resource in resources.values():
        assert resource["repository"].startswith("https://github.com/")
        assert not Path(resource["destination"]).is_absolute()
        for entry in resource.get("files", []):
            assert len(entry["sha256"]) == 64 and entry["bytes"] > 0
            assert entry["url"].startswith("https://")
    assert sum(f["path"].lower().endswith(".stl") for f in resources["g20"]["files"]) == 44
    assert sum(f["path"].lower().endswith(".stl") for f in resources["ar5_08r"]["files"]) == 8


def test_visual_partition_preserves_surface_area_and_plane():
    trimesh = pytest.importorskip("trimesh")
    split = runpy.run_path(str(ROOT / "scripts/build_visual_split.py"))["hand_parts"]
    mesh = trimesh.creation.box(extents=[2, 2, 2])
    black, shell = split(mesh, "hand_base_link")
    assert black.area + shell.area == pytest.approx(mesh.area)
    assert black.vertices[:, 0].min() >= -1e-10
    assert shell.vertices[:, 0].max() <= 1e-10


def test_calibration_reproduction_checks_the_actual_solver_source(tmp_path):
    check = runpy.run_path(str(ROOT / "scripts/reproduce_calibration.py"))["verify_solver_sources"]
    (tmp_path / "calib_board").mkdir()
    source = tmp_path / "calib_board/solver.py"
    source.write_bytes(b"original source")
    expected = {"solver.py": hashlib.sha256(source.read_bytes()).hexdigest()}
    check(tmp_path, expected)
    source.write_bytes(b"modified source")
    with pytest.raises(ValueError, match="differs"):
        check(tmp_path, expected)
