import json
from pathlib import Path
import runpy
import subprocess
import sys
import tarfile

from gamepad_vla.site import configure_runtime, site_value, workspace_root
from gamepad_vla.workstation.spec import load_spec


ROOT = Path(__file__).resolve().parents[1]


def test_private_settings_drive_paths_device_and_cli_defaults(tmp_path, monkeypatch):
    path = tmp_path / "site.json"
    path.write_text(json.dumps({
        "workspace": str(tmp_path / "workspace"),
        "udp": {"bind_host": "127.0.0.1", "port": 51000, "allow_sources": ["198.51.100.8"]},
        "camera": {"serials": {"camera_3": "test-device"}},
        "operator": {"port": 8888},
    }))
    monkeypatch.setenv("GAMEPAD_VLA_SITE_CONFIG", str(path))
    monkeypatch.delenv("GAMEPAD_VLA_WORKSPACE", raising=False)
    assert workspace_root() == tmp_path / "workspace"
    assert site_value("camera.width") == 1280  # unspecified public defaults merge in
    spec = load_spec(ROOT / "configs/workcell/station_g20_v4_4.json")
    assert spec.cameras[0].serial == "test-device"
    assert spec.left_arm.urdf.startswith(str(tmp_path / "workspace"))
    parser = runpy.run_path(str(ROOT / "scripts/run_workstation.py"))["parser"]()
    args = parser.parse_args(["--source", "scripted", "--duration", "0"])
    assert args.udp_port == 51000 and args.udp_allow_source == "198.51.100.8"
    assert args.operator_port == 8888
    assert parser.parse_args(["--source", "scripted", "--duration", "0", "--udp-port", "0"]).udp_port == 0


def test_runtime_resolves_config_before_workers_change_directory(tmp_path, monkeypatch):
    (tmp_path / "site.json").write_text(json.dumps({"workspace": str(tmp_path / "workspace")}))
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv("GAMEPAD_VLA_SITE_CONFIG", "site.json")
    monkeypatch.delenv("GAMEPAD_VLA_WORKSPACE", raising=False)
    monkeypatch.delenv("XBOX_GAMEPAD_PHYSICS_RESOURCES", raising=False)
    configure_runtime()
    (tmp_path / "run").mkdir()
    monkeypatch.chdir(tmp_path / "run")
    assert site_value("workspace") == str(tmp_path / "workspace")
    assert workspace_root() == tmp_path / "workspace"


def test_source_archive_excludes_private_settings_and_archive(tmp_path):
    repo = tmp_path / "repo"
    (repo / "configs").mkdir(parents=True)
    (repo / ".local").mkdir()
    (repo / ".gitignore").write_text(".local/\nconfigs/site.local.json\n")
    (repo / "configs/site.example.json").write_text('{"workspace":"workspace"}')
    (repo / "configs/site.local.json").write_text('{"private_marker":"DO_NOT_DISTRIBUTE"}')
    (repo / ".local/notes.txt").write_text("DO_NOT_DISTRIBUTE")
    subprocess.run(["git", "init", "-q", str(repo)], check=True)
    build = runpy.run_path(str(ROOT / "scripts/build_source_archive.py"))["build_archive"]
    output = tmp_path / "source.tar.gz"
    build(repo, output)
    with tarfile.open(output) as archive:
        assert "configs/site.example.json" in archive.getnames()
        assert "configs/site.local.json" not in archive.getnames()
        assert not any(n.startswith(".local/") for n in archive.getnames())


def test_public_check_finds_password_in_copied_chinese_document(tmp_path):
    subprocess.run(["git", "init", "-q", str(tmp_path)], check=True)
    (tmp_path / "copied.json").write_text(json.dumps({"excerpt": "管理员" + "密码" + "TEST_ONLY"}, ensure_ascii=False))
    find = runpy.run_path(str(ROOT / "scripts/check_public_tree.py"))["findings"]
    matches = list(find(tmp_path))
    assert any(category == "documented password" for _, _, category in matches)
