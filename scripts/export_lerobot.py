#!/usr/bin/env python3
"""Run in data-tools-py310. Export collection v2/v3 directly to LeRobot v3.0."""
import argparse
import json
from pathlib import Path
from gamepad_vla.workstation.lerobot_export import export_episode

if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("episode", type=Path)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--task", default="reference teleoperation")
    p.add_argument("--video", type=Path, help="optional pre-aligned video, one frame per episode row")
    a = p.parse_args()
    print(json.dumps(export_episode(a.episode, a.output, task=a.task, video=a.video)))
