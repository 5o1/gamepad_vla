"""Fit four measured hole centers, then use the existing joint calibration API.

Run on 计算节点 with this directory as the input archive and the matching
station_calib_board source on PYTHONPATH. No source measurements are changed.
"""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from calib_board.calibrate_extrinsics import calibrate
from calib_board.common import quaternion
from calib_board.detect import detect
from calib_board.extrinsics import placement_transform


ROOT = Path(__file__).resolve().parent


def read(path):
    return json.loads(Path(path).read_text())


def write(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


def fit_holes(refs, spec, height_mm):
    """Centered, equal-weight rigid fit; fixed scale and face-up board."""
    holes = {h["id"]: h["center_mm"] for h in spec["holes"]}
    assert len(refs) == 4 and {h["id"] for h in refs} == set(holes)
    board = np.array([holes[h["id"]] for h in refs], dtype=float)
    measured = np.array([[h["from_back_edge_mm"], h["from_left_edge_mm"]]
                         for h in refs], dtype=float)
    assert np.isfinite(measured).all()
    # Reflect the board's in-plane Y before fitting a proper 2D rotation:
    # board +Z points down, while table +Z points up.
    reflection = np.diag([1., -1.])
    source = board @ reflection
    a = source - source.mean(axis=0)
    b = measured - measured.mean(axis=0)
    u, singular, vt = np.linalg.svd(a.T @ b)
    row_rotation = u @ np.diag([1., np.linalg.det(u @ vt)]) @ vt
    rotation = row_rotation.T @ reflection
    translation = measured.mean(axis=0) - board.mean(axis=0) @ rotation.T
    fitted = board @ rotation.T + translation
    residual = fitted - measured
    transform = np.eye(4)
    transform[:2, :2] = rotation
    transform[2, 2] = -1.
    transform[:2, 3] = translation / 1000
    transform[2, 3] = height_mm / 1000
    assert np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1e-12)
    assert np.isclose(np.linalg.det(transform[:3, :3]), 1.)
    pairs = []
    for i, j in ((0, 1), (1, 2), (2, 3), (3, 0), (0, 2), (1, 3)):
        nominal = float(np.linalg.norm(board[i] - board[j]))
        observed = float(np.linalg.norm(measured[i] - measured[j]))
        pairs.append({"holes": [refs[i]["id"], refs[j]["id"]],
                      "nominal_mm": nominal, "measured_mm": observed,
                      "difference_mm": observed - nominal})
    return transform, {
        "method": "Four-hole equal-weight planar rigid least squares; fixed nominal scale; no point rejection",
        "board_surface_above_table_mm": height_mm,
        "rms_hole_residual_mm": float(np.sqrt(np.mean(np.sum(residual**2, axis=1)))),
        "max_hole_residual_mm": float(np.linalg.norm(residual, axis=1).max()),
        "holes": [{"id": h["id"], "measured_table_xy_mm": measured[i].tolist(),
                   "fitted_table_xy_mm": fitted[i].tolist(), "fitted_minus_measured_mm": residual[i].tolist()}
                  for i, h in enumerate(refs)],
        "pairwise_distances": pairs, "T_world_board": transform.tolist(),
    }


def check_fit_convention(spec):
    """Compare against known rotated poses and the existing two-hole implementation."""
    error = 0.
    for theta in (0., .37, np.pi/2, -1.2):
        c, s = np.cos(theta), np.sin(theta)
        truth = np.eye(4)
        truth[:3, :3] = [[c, s, 0], [s, -c, 0], [0, 0, -1]]
        truth[:3, 3] = [.25, .4, .005]
        refs = []
        for hole in spec["holes"]:
            target = truth @ np.r_[np.array(hole["center_mm"]) / 1000, 0, 1]
            refs.append({"id": hole["id"], "from_back_edge_mm": target[0]*1000,
                         "from_left_edge_mm": target[1]*1000})
        fitted, _ = fit_holes(refs, spec, 5.)
        legacy = placement_transform({"hole_measurements": refs[:2],
                                      "board_surface_above_table_mm": 5.}, spec)
        error = max(error, float(np.max(np.abs(fitted-truth))),
                    float(np.max(np.abs(fitted-legacy))))
    assert error < 1e-10
    return {"poses": 4, "max_matrix_element_error": error,
            "scope": "Numerical frame/convention check, not physical calibration accuracy"}


def reprojection(detection, placement, camera_pose, K, D):
    objects = np.asarray(detection["object_points_m"])
    observed = np.asarray(detection["corners_px"])
    world = objects @ placement[:3, :3].T + placement[:3, 3]
    camera_world = np.linalg.inv(camera_pose)
    projected = cv2.projectPoints(world, cv2.Rodrigues(camera_world[:3, :3])[0],
                                  camera_world[:3, 3], K, D)[0].reshape(-1, 2)
    errors = np.linalg.norm(projected-observed, axis=1)
    return projected, errors


def main():
    context = read(ROOT/"capture_context.json")
    plan = read(ROOT/"board-placements.json")
    spec = read(ROOT/"board_spec.json")
    camera = read(ROOT/"intrinsics.json")
    K, D = np.asarray(camera["K"]), np.asarray(camera["D"])
    fits, transforms, manifest = [], [], {"intrinsics": "intrinsics.json", "observations": []}
    fit_check = check_fit_convention(spec)
    for item in context["observations"]:
        record = next(p for p in plan["placements"] if p["name"] == item["placement_name"])
        transform, fit = fit_holes(record["hole_measurements"], spec,
                                  context["board_surface_above_table_mm"])
        fit["label"] = item["label"]
        fits.append(fit); transforms.append(transform)
        relative = f"placements/{item['label']}.json"
        write(ROOT/relative, {"T_world_board": transform.tolist(),
                             "raw_hole_measurements": record["hole_measurements"],
                             "four_hole_fit": fit})
        manifest["observations"].append({"image": item["image"], "placement": relative})
    write(ROOT/"manifest.json", manifest)
    write(ROOT/"placement_fit.json", {"synthetic_convention_check": fit_check, "placements": fits})
    result = calibrate(ROOT/"manifest.json", spec, diagnostics_dir=ROOT/"diagnostics")
    result["world_frame"] = context["coordinate_frame"]
    result["board_surface_above_table_mm"] = context["board_surface_above_table_mm"]
    result["provenance"] = {
        "computed_at_utc": datetime.now(timezone.utc).isoformat(),
        "solver_repo_commit": context["solver_repo_commit"],
        "solver_source_hashes": context["solver_source_hashes"],
        "four_hole_adapter_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "opencv": cv2.__version__, "numpy": np.__version__,
        "intrinsics_source": camera["source"],
        "height_source": context["height_source"],
        "scale_source": context["print_scale_user_statement"],
    }
    joint = np.asarray(result["T_world_camera_optical"])
    views = []
    for i, (item, placement) in enumerate(zip(context["observations"], transforms)):
        stem = f"{i:04d}_{Path(item['image']).stem}"
        detection = read(ROOT/"diagnostics"/f"{stem}_detection.json")
        projected, errors = reprojection(detection, placement, joint, K, D)
        individual = placement @ np.linalg.inv(np.asarray(detection["T_camera_optical_board"]))
        overlay = cv2.imread(str(ROOT/item["image"]))
        for observed, predicted in zip(detection["corners_px"], projected):
            a = tuple(np.rint(observed).astype(int)); b = tuple(np.rint(predicted).astype(int))
            cv2.line(overlay, a, b, (0, 220, 255), 1)
            cv2.circle(overlay, a, 4, (0, 200, 0), 1)
            cv2.drawMarker(overlay, b, (0, 0, 255), cv2.MARKER_CROSS, 8, 1)
        cv2.putText(overlay, "green: detected / red: joint projection", (15, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 0, 0), 3)
        cv2.putText(overlay, "green: detected / red: joint projection", (15, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 255, 255), 1)
        cv2.imwrite(str(ROOT/"diagnostics"/f"{item['label']}_joint.jpg"), overlay)
        views.append({
            "label": item["label"], "image": item["image"], "corners": len(errors),
            "joint_reprojection_median_px": float(np.median(errors)),
            "joint_reprojection_p95_px": float(np.percentile(errors, 95)),
            "joint_reprojection_max_px": float(errors.max()),
            "independent_board_detection_median_px": detection["reprojection_median_px"],
            "individual_camera_pose": individual.tolist(),
            "individual_vs_joint_translation_mm": float(np.linalg.norm(individual[:3, 3]-joint[:3, 3])*1000),
            "individual_vs_joint_rotation_deg": float(np.linalg.norm(
                cv2.Rodrigues(joint[:3, :3].T @ individual[:3, :3])[0])*180/np.pi),
        })
    result["per_view_joint_diagnostics"] = views
    repeated = []
    for item in context["all_images"]:
        if item["selected_for_joint_solve"]:
            continue
        index = next(i for i, p in enumerate(context["observations"]) if p["label"] == item["label"])
        detection, _ = detect(cv2.imread(str(ROOT/item["image"])), spec, K, D)
        _, errors = reprojection(detection, transforms[index], joint, K, D)
        repeated.append({"image": item["image"], "label": item["label"], "corners": len(errors),
                         "median_px": float(np.median(errors)), "p95_px": float(np.percentile(errors, 95)),
                         "scope": "Repeated frame of an existing placement; not independent spatial/physical validation."})
    result["unused_repeated_frame_check"] = repeated
    result["max_measured_hole_fit_residual_mm"] = max(f["max_hole_residual_mm"] for f in fits)
    write(ROOT/"extrinsics_table.json", result)

    table = context["project_world_table"]
    assert table["pose"][3:] == [1., 0., 0., 0.]
    origin = np.asarray(table["pose"][:3]) + np.asarray(table["half_extents"])*[-1, -1, 1]
    project_table = np.eye(4); project_table[:3, 3] = origin
    project_camera = project_table @ joint
    write(ROOT/"extrinsics_project_world.json", {
        "T_project_world_camera_optical": project_camera.tolist(),
        "pose_xyz_qw_qx_qy_qz": np.r_[project_camera[:3, 3], quaternion(project_camera[:3, :3])].tolist(),
        "T_project_world_table": project_table.tolist(),
        "translation_unit": "m", "camera_axes": result["camera_axes"],
        "source_table_result": "extrinsics_table.json",
        "table_origin_source": context["project_world_table_source"],
        "table_origin_source_sha256": context["project_world_table_source_sha256"],
        "scope": "Derived using the existing configured table frame; robot-to-table calibration not independently verified.",
        "physical_accuracy_verified": False,
    })
    print(json.dumps({
        "images": result["images"], "corners": result["corners_per_image"],
        "median_px": result["reprojection_median_px"], "p95_px": result["reprojection_p95_px"],
        "pose_table": result["pose_xyz_qw_qx_qy_qz"],
        "hole_fits": [{"label": f["label"], "rms_mm": f["rms_hole_residual_mm"],
                       "max_mm": f["max_hole_residual_mm"]} for f in fits],
        "views": views, "uncertainty": result["uncertainty"],
    }, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
