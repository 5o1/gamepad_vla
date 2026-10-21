#!/usr/bin/env python3
"""Isolated real-rclpy regression; optional renderer-free SAPIEN acceptance."""
import argparse
import json
from pathlib import Path
from gamepad_vla.workstation.ros2_check import check_ros2

if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--duration',type=float,default=2.)
    p.add_argument('--replay',type=Path)
    p.add_argument('--backend',choices=['sapien'])
    a=p.parse_args()
    result=check_ros2(a.output,duration=a.duration,replay=a.replay,backend_name=a.backend)
    print(json.dumps(result,indent=2))
    raise SystemExit(0 if result['pass_gate'] else 1)
