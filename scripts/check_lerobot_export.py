#!/usr/bin/env python3
"""Offline native readback. Run ONLY in lerobot-check-py310."""
import argparse
import json
import os
from pathlib import Path
from gamepad_vla.site import workspace_root
os.environ["HF_HUB_OFFLINE"] = "1"
os.environ["HF_DATASETS_OFFLINE"] = "1"
os.environ.setdefault("HF_HOME", str(workspace_root() / "cache/huggingface"))
os.environ.setdefault("XDG_CACHE_HOME", str(workspace_root() / "cache"))
from lerobot.datasets.lerobot_dataset import LeRobotDataset, CODEBASE_VERSION

if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("dataset", type=Path)
    a = p.parse_args()
    dataset = LeRobotDataset("local/collection", root=a.dataset, video_backend="pyav")
    shapes = {"observation.state": [56], "action": [54], "observation.gamepad_signal": [32],
              "observation.gamepad_mask": [32], "observation.world_poses": [84]}
    for i in range(len(dataset)):
        row = dataset[i]
        for key, shape in shapes.items():
            if list(row[key].shape) != shape:
                raise ValueError(f"wrong shape {key}: {row[key].shape}")
    print(json.dumps(dict(native_reader="lerobot==0.4.0", codebase_version=CODEBASE_VERSION,
                          rows=len(dataset), shapes=shapes, all_rows_read=True)))
