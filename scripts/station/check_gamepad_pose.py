#!/usr/bin/env python3
"""Estimate camera_3 gamepad pose from camera_service ZIP or image.png/camera.json."""
import argparse
import json
import math
from pathlib import Path

from gamepad_vla.workstation.gamepad_pose_check import (
    DEFAULT_SPEC, check_pose, load_hulls, load_snapshot, write_result,
)


def main():
    from gamepad_vla.site import configure_runtime
    configure_runtime()
    p = argparse.ArgumentParser(description=__doc__)
    source = p.add_mutually_exclusive_group(required=True)
    source.add_argument('--summarize', type=Path, nargs='+', help='summarize ten result JSON files')
    source.add_argument('--snapshot', type=Path, help='camera_service ZIP or unresized PNG')
    source.add_argument('--capture', action='store_true', help='station-local one-shot camera_service, domain 89, loopback')
    p.add_argument('--camera-json', type=Path)
    p.add_argument('--capture-count', type=int, default=1, help='1..30 stationary frames; mean image, default 1')
    p.add_argument('--method', choices=['marker','silhouette'], default='marker')
    from gamepad_vla.workstation.gamepad_marker import DEFAULT_MARKER
    p.add_argument('--marker-manifest', type=Path, default=DEFAULT_MARKER)
    p.add_argument('--card-geometry-mm', type=float, help='measured board scale/mount position error bound')
    p.add_argument('--card-angle-deg', type=float, help='measured mount angle error bound')
    p.add_argument('--spec', type=Path, default=DEFAULT_SPEC)
    p.add_argument('--urdf', type=Path, help='0.4.1 full_controls mobility.urdf; default installed package resource')
    p.add_argument('--output', type=Path, required=True, help='prefix for PNG and JSON')
    p.add_argument('--reference-result', type=Path, help='baseline JSON for creep/reclamping drift')
    p.add_argument('--calibration-position-mm', type=float, help='independently established calibration error bound')
    p.add_argument('--calibration-angle-deg', type=float)
    a = p.parse_args()
    if a.summarize:
        from gamepad_vla.workstation.gamepad_marker import summarize_reclamping
        summary = summarize_reclamping([json.loads(path.read_text()) for path in a.summarize])
        a.output.parent.mkdir(parents=True,exist_ok=True)
        a.output.with_suffix('.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps(summary,ensure_ascii=False,indent=2))
        return
    for value in (a.calibration_position_mm, a.calibration_angle_deg, a.card_geometry_mm, a.card_angle_deg):
        if value is not None and (not math.isfinite(value) or value < 0):
            p.error('Error bounds must be finite and nonnegative')
    if a.capture:
        from gamepad_vla.workstation.gamepad_pose_capture import capture_snapshot
        a.snapshot = a.output.with_suffix('.zip')
        captured=capture_snapshot(a.snapshot,count=a.capture_count)
        if isinstance(captured,list):
            import numpy as np
            import cv2
            frames=[load_snapshot(path) for path in captured]
            image=np.rint(np.mean([row[0].astype(float) for row in frames],axis=0)).astype(np.uint8)
            meta=frames[0][1];meta["averaged_frames"]=len(frames)
            meta["averaging_note"]="stationary-scene pixel mean; systematic calibration/print error remains"
            mean_path=a.output.with_name(a.output.name+"-mean.png")
            cv2.imwrite(str(mean_path),image)
            a.camera_json=mean_path.with_suffix(".json")
            a.camera_json.write_text(json.dumps(meta,indent=2)+"\n")
            a.snapshot=mean_path
    image, meta, _, _ = load_snapshot(a.snapshot, a.camera_json)
    scene = json.loads(a.spec.read_text())
    reference = json.loads(a.reference_result.read_text()) if a.reference_result else None
    if a.method == 'marker':
        from gamepad_vla.workstation.gamepad_marker import check_marker_pose
        marker = json.loads(a.marker_manifest.read_text())
        result, overlay = check_marker_pose(image, meta, scene, load_hulls(a.urdf), marker,
            reference=reference, calibration_position_mm=a.calibration_position_mm,
            calibration_angle_deg=a.calibration_angle_deg, card_geometry_mm=a.card_geometry_mm,
            card_angle_deg=a.card_angle_deg)
    else:
        result, overlay = check_pose(image, meta, scene, load_hulls(a.urdf),
                                 calibration_position_mm=a.calibration_position_mm,
                                 calibration_angle_deg=a.calibration_angle_deg, reference=reference)
    result['input'] = str(a.snapshot)
    result['spec'] = str(a.spec)
    write_result(a.output, result, overlay)
    print(result['message'])
    if result['detected']:
        c = result['components']
        print(f"倾角 {c['tilt_deg']:.3f}°；位置误差 {c['position_error_mm']:.2f} mm")
        for name, criterion in result['acceptance'].items():
            print(f"  {name}: {criterion['error']:+.3f} ±{criterion['uncertainty']:.3f}  {criterion['verdict']}")
    if result.get('relative_to_reference'):
        print('相对基线：' + json.dumps(result['relative_to_reference'], ensure_ascii=False))
    if result.get('sticker'):
        sticker=result['sticker']
        print('临时贴纸：' + ('已检测，测量对象为贴纸相对托架卡' if sticker.get('detected') else '未检测到'))
        if sticker.get('change_from_reference'):
            print('贴纸相对变化：' + json.dumps(sticker['change_from_reference'],ensure_ascii=False))
    print(f"输出：{a.output.with_suffix('.png')} / {a.output.with_suffix('.json')}")


if __name__ == '__main__':
    main()
