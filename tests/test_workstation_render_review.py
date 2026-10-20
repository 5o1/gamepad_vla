from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from gamepad_vla.workstation.render_review import prepare_config
from gamepad_vla.workstation.sapien_backend import PHYSICS_DT, SapienBackend, validate_scene_injection
from gamepad_vla.workstation.spec import dump_spec, load_spec


SPEC = Path(__file__).parents[1] / "configs/workcell/station_left_g20_v3.json"


def config():
    return {"profile": "wrong", "gamepad": {"fixed_root": True, "attach_visual": True,
                                           "collision": "sdf"}, "params": {"steps": 120}}


def test_config_uses_spec_for_all_gamepad_fields(tmp_path):
    original = load_spec(SPEC)
    # Different values prove the entry fills these fields rather than keeping YAML defaults.
    spec = replace(original, gamepad=replace(original.gamepad, profile="dpad",
                   fixed_root=False, collision="coacd", root_pose=(1, 2, 3, 1, 0, 0, 0)))
    path = tmp_path / "spec.json"
    dump_spec(spec, path)
    cfg = prepare_config(config(), path)
    assert cfg["profile"] == "dpad"
    assert cfg["gamepad"] == {"pose": [1, 2, 3, 1, 0, 0, 0], "fixed_root": False,
                              "collision": "coacd", "attach_visual": True}
    assert cfg["params"] == {"steps": 120, "spec": str(path.resolve())}


@pytest.mark.parametrize("pose", [None, [0, 0, 0, 1, 0, 0, 0]])
def test_yaml_pose_is_rejected(pose):
    cfg = config()
    cfg["gamepad"]["pose"] = pose
    with pytest.raises(ValueError, match="gamepad.pose"):
        prepare_config(cfg, SPEC)


@pytest.mark.parametrize("scene,gamepad", [(object(), None), (None, object()),
    (SimpleNamespace(get_timestep=lambda: 1 / 60), object()),
    (SimpleNamespace(get_timestep=lambda: 1 / 239), object())])
def test_constructor_rejects_bad_injection_without_sapien(scene, gamepad):
    with pytest.raises(ValueError, match="together|timestep"):
        SapienBackend(None, scene=scene, gamepad=gamepad)


@pytest.mark.parametrize("timestep", [PHYSICS_DT, PHYSICS_DT + 0.5e-12,
                                      PHYSICS_DT - 0.5e-12, float(np.float32(PHYSICS_DT))])
def test_injection_accepts_matching_timestep(timestep):
    validate_scene_injection(SimpleNamespace(get_timestep=lambda: timestep), object())
    validate_scene_injection(None, None)
