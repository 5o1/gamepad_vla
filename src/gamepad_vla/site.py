"""Private site settings, separate from shareable scene and camera geometry."""
import argparse
import json
import os
from pathlib import Path
import shlex


def repository_root():
    return Path(__file__).resolve().parents[2]


def load_site():
    directory = repository_root() / "configs"
    example = directory / "site.example.json"
    values = json.loads(example.read_text()) if example.exists() else {}
    explicit = os.environ.get("GAMEPAD_VLA_SITE_CONFIG")
    local = Path(explicit).expanduser() if explicit else directory / "site.local.json"
    if explicit or local.exists():
        override = json.loads(local.read_text())
        def merge(target, source):
            for key, value in source.items():
                if isinstance(value, dict) and isinstance(target.get(key), dict):
                    merge(target[key], value)
                else:
                    target[key] = value
        merge(values, override)
    return values


def site_value(key, default=None, *, required=False):
    value = load_site()
    for part in key.split("."):
        if not isinstance(value, dict) or part not in value:
            value = default
            break
        value = value[part]
    if required and (value is None or value == ""):
        raise ValueError(f"Set {key} in configs/site.local.json or GAMEPAD_VLA_SITE_CONFIG")
    return value


def workspace_root():
    value = os.environ.get("GAMEPAD_VLA_WORKSPACE") or site_value(
        "workspace", "~/.local/share/gamepad-vla")
    return Path(value).expanduser().resolve()


def configure_runtime():
    """Keep explicit environment overrides; otherwise use private site settings."""
    if os.environ.get("GAMEPAD_VLA_SITE_CONFIG"):
        os.environ["GAMEPAD_VLA_SITE_CONFIG"] = str(
            Path(os.environ["GAMEPAD_VLA_SITE_CONFIG"]).expanduser().resolve())
    # Renderer/gateway children change cwd to their run directory.
    os.environ["GAMEPAD_VLA_WORKSPACE"] = str(workspace_root())
    for variable, key in (("CUDA_VISIBLE_DEVICES", "compute.cuda_visible_devices"),
                          ("GAMEPAD_VLA_RENDER_DEVICE", "compute.render_device")):
        value = site_value(key)
        if value:
            os.environ.setdefault(variable, str(value))
    os.environ.setdefault("XBOX_GAMEPAD_PHYSICS_RESOURCES", str(
        workspace_root() / "assets/xbox_gamepad_physics_v0.4.1/resources"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--get")
    parser.add_argument("--shell", action="store_true")
    args = parser.parse_args()
    if args.get:
        value = site_value(args.get, required=True)
        print(json.dumps(value) if isinstance(value, (dict, list)) else value)
    elif args.shell:
        values = {
            "GAMEPAD_VLA_WORKSPACE": str(workspace_root()),
            "GAMEPAD_VLA_REMOTE_CODE": site_value("compute.remote_code") or str(workspace_root() / "code/gamepad_vla"),
            "GAMEPAD_VLA_COMPUTE_PROFILE": site_value("compute.ssh_profile", ""),
            "GAMEPAD_VLA_CONDA": site_value("conda_executable", "conda"),
        }
        for field in ("ssh_profile", "home", "workspace", "data_root", "camera_root", "estop_script",
                      "ros_setup", "python", "conda_setup", "left_arm_ip", "right_arm_ip",
                      "left_local_ip", "right_local_ip", "left_can", "right_can", "hand_joint"):
            values["STATION_" + field.upper()] = site_value("station." + field, "")
        for key, value in values.items():
            print(f"export {key}={shlex.quote(os.environ.get(key, str(value)))}")
    else:
        parser.error("choose --get KEY or --shell")


if __name__ == "__main__":
    main()
