"""Workcell review experiment registered with the gamepad asset's example runner."""

import hashlib
import math
import json
from pathlib import Path

import numpy as np

from gamepad_vla.workstation.sapien_backend import SapienBackend
from gamepad_vla.workstation.spec import load_spec, optical_to_sapien_pose, compose, pose_to_matrix
from gamepad_vla.workstation.paths import resolve_workspace_path


def prepare_config(cfg, spec_path):
    """Fill the runner configuration from the sole source of gamepad poses."""
    if "pose" in cfg["gamepad"]:
        raise ValueError("gamepad.pose must not appear in YAML; use --spec")
    spec_path = Path(spec_path).expanduser().resolve()
    spec = load_spec(spec_path)
    cfg["gamepad"].update(pose=list(spec.gamepad.root_pose),
                          fixed_root=spec.gamepad.fixed_root,
                          collision=spec.gamepad.collision)
    cfg["profile"] = spec.gamepad.profile
    cfg["params"]["spec"] = str(spec_path)
    if "review_manifest" in cfg["params"]:
        cfg["params"]["review_manifest"] = resolve_workspace_path(cfg["params"]["review_manifest"])
    return cfg


def run(cfg, ctx):
    if cfg["experiment"] == "workcell_twin_review":
        return run_twin(cfg, ctx)
    from PIL import Image
    from xbox_gamepad_physics.examples.scene import look_at, take_picture

    params = cfg["params"]
    spec_path = Path(params["spec"])
    spec = load_spec(spec_path)
    backend = SapienBackend(spec, scene=ctx.scene, gamepad=ctx.gamepad,
                            work_dir=ctx.output_dir / "work")
    backend.reset()
    arm_max = hand_max = signal_max = 0.0

    def record():
        nonlocal arm_max, hand_max, signal_max
        obs = backend.observe()
        arm_max = max(arm_max, float(np.max(np.abs(
            np.asarray(obs.arm_joint_positions) - spec.left_arm.home_q))))
        hand_max = max(hand_max, max(abs(q) for q in obs.hand_joint_positions.values()))
        signal_max = max(signal_max, float(np.max(
            np.abs(obs.gamepad_signal[obs.gamepad_mask]), initial=0.0)))
        return obs

    obs = record()
    for _ in range(params["steps"]):
        backend.step(1 / 60)
        obs = record()

    cameras = {}
    for name, camera_cfg in params["cameras"].items():
        camera = ctx.scene.add_camera(
            name, camera_cfg["width"], camera_cfg["height"],
            math.radians(camera_cfg["fovy_degrees"]), camera_cfg["near"], camera_cfg["far"],
        )
        camera.entity.set_pose(look_at(camera_cfg["position"], camera_cfg["target"]))
        filename = f"{name}.png"
        take_picture(ctx.scene, camera, ctx.output_dir / filename)
        with Image.open(ctx.output_dir / filename) as picture:
            gray_std = float(np.asarray(picture.convert("L"), dtype=float).std())
        cameras[name] = {"file": filename, "grayscale_std": gray_std}

    definitions = {camera.name: camera for camera in spec.cameras}
    for name in params.get("spec_cameras", []):
        definition = definitions[name]
        camera = create_spec_camera(backend, definition)
        filename = f"{name}.png"
        take_picture(ctx.scene, camera, ctx.output_dir / filename)
        with Image.open(ctx.output_dir / filename) as picture:
            gray_std = float(np.asarray(picture.convert("L"), dtype=float).std())
        cameras[name] = {
            "file": filename, "grayscale_std": gray_std, "source": "workcell_spec",
            "resolution": [definition.intrinsics.width, definition.intrinsics.height],
            "intrinsic_matrix": np.asarray(camera.get_intrinsic_matrix()).tolist(),
            "pose_world_optical": list(camera_world_optical(definition, backend)),
            "status": definition.status, "calibration_source": definition.source,
        }

    return {
        "spec_name": spec.name,
        "spec": str(spec_path),
        "spec_sha256": hashlib.sha256(spec_path.read_bytes()).hexdigest(),
        "steps": params["steps"],
        "arm_max_error_rad": arm_max,
        "hand_max_abs_rad": hand_max,
        "gamepad_max_abs_signal": signal_max,
        "palm_world": list(obs.hand_root_pose_world),
        "render_device": str(ctx.scene.render_system.device),
        "cameras": cameras,
        "passed": arm_max < 1e-6 and signal_max == 0
                  and all(camera["grayscale_std"] > 5 for camera in cameras.values()),
    }


def camera_world_optical(camera_spec, backend):
    if camera_spec.parent == backend.spec.world_frame:
        return camera_spec.pose_optical
    owner, link = camera_spec.parent.split(":", 1)
    side, kind = owner.split("_", 1)
    if kind == "hand" and side == "right":
        link = "right_" + link
    return compose(backend.link_pose(link), camera_spec.pose_optical)


def create_spec_camera(backend, camera_spec):
    """Use explicit K and optical centre, never a look-at/FOV approximation."""
    import sapien
    intr = camera_spec.intrinsics
    fovy = 2*math.atan(intr.height/(2*intr.fy))
    if camera_spec.parent == backend.spec.world_frame:
        camera = backend.scene.add_camera(camera_spec.name, intr.width, intr.height, fovy, .001, 20.)
        pose = optical_to_sapien_pose(camera_spec.pose_optical)
        camera.entity.set_pose(sapien.Pose(pose[:3],pose[3:]))
    else:
        pose = optical_to_sapien_pose(camera_spec.pose_optical)
        camera = backend.scene.add_mounted_camera(camera_spec.name,
                    backend.camera_parent_entity(camera_spec.parent),
                    sapien.Pose(pose[:3],pose[3:]), intr.width,intr.height,fovy,.001,20.)
    camera.set_perspective_parameters(.001,20.,intr.fx,intr.fy,intr.cx,intr.cy,0.)
    return camera


def mask_iou(real, simulated):
    union = np.count_nonzero(real | simulated)
    return float(np.count_nonzero(real & simulated) / union) if union else None


def robot_only_review_visuals(backend):
    """Hide current-epoch fixtures for old-epoch robot-only image comparison.

    Home views are captured before this. No collision/PhysX component is touched.
    Current table/camera bodies must not occlude an old recorded robot pose.
    """
    from sapien.render import RenderBodyComponent
    for actor in [backend.table,*backend.fixtures,*backend.gamepad.articulation.get_links()]:
        # ActorBuilder returns Entity; articulation links expose .entity.
        entity=getattr(actor,'entity',actor)
        body=entity.find_component_by_type(RenderBodyComponent)
        if body is not None:body.visibility=0.
    camera_meshes={f.mesh for f in backend.spec.fixtures if f.name.startswith('camera_')}
    for robot in (backend.robot,backend.right_robot):
        if robot is None:continue
        for link in robot.get_links():
            body=link.entity.find_component_by_type(RenderBodyComponent)
            if body is None:continue
            shapes=body.render_shapes
            if not any(getattr(shape,'filename',None) in camera_meshes for shape in shapes):continue
            replacement=RenderBodyComponent()
            for shape in shapes:
                if getattr(shape,'filename',None) not in camera_meshes:replacement.attach(shape.clone())
            link.entity.remove_component(body);link.entity.add_component(replacement)


def run_twin(cfg, ctx):
    import cv2
    import sapien
    from PIL import Image
    from xbox_gamepad_physics.examples.scene import take_picture
    spec_path = Path(cfg["params"]["spec"])
    spec = load_spec(spec_path)
    if len(spec.cameras) != 3:
        raise ValueError("workcell_twin_review requires three calibrated spec cameras")
    manifest = json.loads(Path(cfg["params"]["review_manifest"]).read_text())
    backend = SapienBackend(spec,scene=ctx.scene,gamepad=ctx.gamepad,work_dir=ctx.output_dir/"work",render=True)
    cameras = {c.name:create_spec_camera(backend,c) for c in spec.cameras}
    results = {}
    # Home/operator views use exactly the same calibrated camera definitions.
    for definition in spec.cameras:
        camera = cameras[definition.name]
        take_picture(ctx.scene,camera,ctx.output_dir/f"{definition.name}_home.png")
    robot_only_review_visuals(backend)
    for sample in manifest["frames"]:
        snapshot = json.loads(Path(sample["snapshot"]).read_text())
        backend.set_review_state(snapshot)
        frame_metrics = {}
        for definition in spec.cameras:
            name = definition.name
            item = sample["cameras"][name]
            camera = cameras[name]
            prefix = f"{sample['name']}_{name}"
            sim_path = ctx.output_dir/f"{prefix}_sim.png"
            take_picture(ctx.scene,camera,sim_path)
            real = cv2.imread(item["real_image"])
            intr=definition.intrinsics
            if real is None or real.shape[:2] != (intr.height,intr.width):
                raise ValueError("real image does not match calibrated stream profile")
            k=np.array([[intr.fx,0,intr.cx],[0,intr.fy,intr.cy],[0,0,1.]])
            distortion=np.asarray(intr.distortion or [0.]*5)
            real=cv2.undistort(real,k,distortion,None,k)
            real_path=ctx.output_dir/f"{prefix}_real_undistorted.png"
            cv2.imwrite(str(real_path),real)
            sim=cv2.imread(str(sim_path))
            cv2.imwrite(str(ctx.output_dir/f"{prefix}_overlay.png"),cv2.addWeighted(real,.5,sim,.5,0))
            segmentation=camera.get_picture("Segmentation")[...,1]
            classes={"arm":[],"palm":[]}
            silhouette_ids=[]
            for side,robot in (("left",backend.robot),("right",backend.right_robot)):
                if robot is None:continue
                for link in robot.get_links():
                    own_hand=(not link.name.startswith("AR5_") and
                              (definition.parent==spec.world_frame or definition.parent.startswith(side+"_")))
                    if definition.parent==spec.world_frame or own_hand:
                        silhouette_ids.append(link.entity.per_scene_id)
                    if link.name.startswith("AR5_"):
                        classes["arm"].append(link.entity.per_scene_id)
                    elif link.name.endswith("hand_base_link") and own_hand:
                        classes["palm"].append(link.entity.per_scene_id)
            masks={}
            comparison_mask=np.zeros((intr.height,intr.width),bool)
            for label,ids in classes.items():
                simulated=np.isin(segmentation,ids)
                real_mask=np.zeros((intr.height,intr.width),dtype=np.uint8)
                for polygon in item["masks"][label]:
                    cv2.fillPoly(real_mask,[np.asarray(polygon,np.int32)],1)
                real_mask=cv2.undistort(real_mask,k,distortion,None,k)>.5
                Image.fromarray((simulated*255).astype(np.uint8)).save(ctx.output_dir/f"{prefix}_{label}_sim_mask.png")
                Image.fromarray((real_mask*255).astype(np.uint8)).save(ctx.output_dir/f"{prefix}_{label}_real_mask.png")
                masks[label]=dict(iou=mask_iou(real_mask,simulated),real_pixels=int(real_mask.sum()),sim_pixels=int(simulated.sum()))
                comparison_mask |= real_mask|simulated
            automatic=cv2.imread(item["observed_mask"],cv2.IMREAD_GRAYSCALE)>0
            valid=cv2.imread(item["valid_mask"],cv2.IMREAD_GRAYSCALE)>0
            predicted=np.isin(segmentation,silhouette_ids)&valid
            automatic_iou=mask_iou(automatic,predicted)
            comparison_mask |= automatic|np.isin(segmentation,silhouette_ids)
            overlay=real.copy()
            overlay[comparison_mask]=cv2.addWeighted(real,.5,sim,.5,0)[comparison_mask]
            cv2.imwrite(str(ctx.output_dir/f"{prefix}_overlay.png"),overlay)
            calibration=json.loads(Path(item["calibration"]).read_text())
            landmark_data=json.loads(Path(sample["landmarks"]).read_text())
            projected_errors=[]
            agreement=[]
            for point in calibration["points"]:
                if point["frame"] != sample["name"]:continue
                landmark=landmark_data[point["key"]]
                world=pose_to_matrix(backend.link_pose(landmark["link"])) @ np.r_[landmark["local"],1.]
                optical=camera.get_extrinsic_matrix() @ world
                projected=k @ optical; projected=projected[:2]/projected[2]
                projected_errors.append(float(np.linalg.norm(projected-point["uv"])))
                agreement.append(float(np.linalg.norm(projected-point["projected"])))
            def error_summary(errors):
                return dict(count=len(errors),median_px=float(np.median(errors)) if errors else None,
                            p95_px=float(np.percentile(errors,95)) if errors else None,
                            max_px=max(errors) if errors else None)
            frame_metrics[name]=dict(masks=masks,reprojection=calibration["metrics"],
                                     frame_reprojection=error_summary(projected_errors),
                                     sapien_vs_numpy_projection=error_summary(agreement),
                                     automatic_roi_silhouette_iou=automatic_iou,
                                     calibration_quality=calibration["quality"],
                                     real_image=str(real_path),simulation=str(sim_path),
                                     overlay=str(ctx.output_dir/f"{prefix}_overlay.png"))
        results[sample["name"]]=frame_metrics
    return dict(spec_name=spec.name,spec_sha256=hashlib.sha256(spec_path.read_bytes()).hexdigest(),
                passed=True,scope="仅仿真；运行完整不代表相机/掩码非关键精度通过",frames=results,
                material_assignments=backend.material_assignments,
                heldout_render_scope="AR5/G20 visual geometry only; current table, floor, column, gamepad and camera shells hidden after complete home views; collisions unchanged",
                distortion_note="SAPIEN无畸变；真实图按该profile D去畸变。D未知时零先验不代表实际无畸变。")
