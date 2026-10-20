#!/usr/bin/env python3
"""Render station left-hand assets at q=0 through the gamepad example scene.

Run on the compute node, with CUDA_VISIBLE_DEVICES=3. No hardware or physics steps.
Only existing SAPIEN, trimesh, transforms3d, numpy and Pillow are used.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw, ImageFont
import trimesh
from transforms3d.euler import euler2mat


MODELS = {
    "L20": "linkerhand_g20_l20_l25_urdf_mesh/l20_left/linkerhand_l20_left.urdf",
    "L20_V10.1": "linkerhand_L20_V10.1--URDF_1785310008741/linkerhand_L20_V10.1_left.urdf/linkerhand_L20v10.1_left.urdf",
    "L25_G20": "linkerhand_g20_l20_l25_urdf_mesh/l25_left/linkerhand_l25_left.urdf",
}
VIEWS = {
    "palm": ([1, 0, 0], [0, 0, 1]),
    "back": ([-1, 0, 0], [0, 0, 1]),
    "thumb_side": ([0, -1, 0], [0, 0, 1]),
    "little_side": ([0, 1, 0], [0, 0, 1]),
    "fingertips": ([0, 0, 1], [-1, 0, 0]),
    "three_quarter": ([1, -1, .7], [0, 0, 1]),
}
TARGET = np.array([0., 0., .13])
DISTANCE = .80
FOVY = 32.
MATERIAL = dict(base_color=[.55, .55, .55, 1.], roughness=.6, metallic=0.)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def xyz(value, default="0 0 0"):
    return np.fromstring(value or default, sep=" ")


def origin(element):
    t = np.eye(4)
    if element is not None:
        t[:3, :3] = euler2mat(*xyz(element.get("rpy")), axes="sxyz")
        t[:3, 3] = xyz(element.get("xyz"))
    return t


def inspect_model(path, work):
    tree = ET.parse(path)
    robot = tree.getroot()
    links = {link.get("name"): link for link in robot.findall("link")}
    joints = robot.findall("joint")
    children = {j.find("child").get("link") for j in joints}
    root, = set(links) - children
    transforms = {root: np.eye(4)}
    pending = list(joints)
    while pending:
        progressed = False
        for joint in pending[:]:
            parent = joint.find("parent").get("link")
            if parent not in transforms:
                continue
            transforms[joint.find("child").get("link")] = transforms[parent] @ origin(joint.find("origin"))
            pending.remove(joint)
            progressed = True
        if not progressed:
            raise ValueError("URDF is not a rooted tree")
    points = []
    link_bounds = {}
    meshes = {}
    visuals = 0
    for name, link in links.items():
        local_points = []
        for visual in link.findall("visual"):
            mesh = visual.find("geometry/mesh")
            # The supplied models contain mesh visuals only.
            filename = mesh.get("filename")
            meshpath = (path.parent / filename).resolve()
            vertices = np.asarray(trimesh.load(meshpath, force="mesh").vertices)
            vertices = vertices * xyz(mesh.get("scale"), "1 1 1")
            t = transforms[name] @ origin(visual.find("origin"))
            vertices = vertices @ t[:3, :3].T + t[:3, 3]
            local_points.append(vertices)
            meshes[str(meshpath)] = sha(meshpath)
            mesh.set("filename", str(meshpath))
            visuals += 1
        if local_points:
            vertices = np.concatenate(local_points)
            points.append(vertices)
            link_bounds[name] = [vertices.min(0).tolist(), vertices.max(0).tolist()]
        # Rendering only: no collision cooking, drives, gravity integration.
        for collision in list(link.findall("collision")):
            link.remove(collision)
    vertices = np.concatenate(points)
    bounds = np.array([vertices.min(0), vertices.max(0)])
    ext = bounds[1] - bounds[0]
    derived = work / path.name
    tree.write(derived, encoding="utf-8", xml_declaration=True)
    metadata = dict(
        source_urdf=str(path), source_urdf_sha256=sha(path), robot_name=robot.get("name"),
        root_link=root, root_pose=[0, 0, 0, 1, 0, 0, 0], root_alignment=np.eye(3).tolist(),
        links=len(links), joints=len(joints),
        movable_joints=sum(j.get("type") != "fixed" for j in joints),
        fixed_joints=sum(j.get("type") == "fixed" for j in joints),
        mimic_joints=sum(j.find("mimic") is not None for j in joints),
        visual_shapes=visuals, visual_links=len(link_bounds), unique_meshes=len(meshes),
        length_width_thickness_mm=(ext[[2, 1, 0]] * 1000).tolist(),
        visual_bbox_root_m=bounds.tolist(), link_visual_bounds_root_m=link_bounds,
        zero_fk={k: v.tolist() for k, v in transforms.items()},
        joints_definition=[dict(name=j.get("name"), type=j.get("type"),
            parent=j.find("parent").get("link"), child=j.find("child").get("link"),
            origin=j.find("origin").attrib,
            axis=j.find("axis").attrib if j.find("axis") is not None else None,
            mimic=j.find("mimic").attrib if j.find("mimic") is not None else None)
            for j in joints],
        meshes_sha256=meshes, derived_urdf=str(derived),
    )
    return metadata


def font(size):
    return ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", size)


def compose(output, metadata, photos):
    # Fixed cell dimensions and a common resize preserve the render scale.
    cw, ch, header, footer = 600, 450, 42, 50
    total = Image.new("RGB", (cw * 6, 64 + 3 * (ch + header + footer)), "#fafafa")
    draw = ImageDraw.Draw(total)
    draw.text((20, 12), "LEFT HANDS | q = 0 | neutral gray | identical camera scale", fill="#222222", font=font(30))
    for row, (model, data) in enumerate(metadata.items()):
        top = 64 + row * (ch + header + footer)
        for col, view in enumerate(VIEWS):
            tile = Image.open(output / f"{model}_{view}.png").convert("RGB")
            tile = tile.resize((cw, ch), Image.Resampling.LANCZOS)
            total.paste(tile, (col * cw, top + header))
            draw.text((col * cw + 12, top + 8), f"{model.replace('_', ' / ')} | {view.replace('_', ' ')}", fill="#222222", font=font(23))
        l, w, t = data["length_width_thickness_mm"]
        text = (f"{model}: L x W x T = {l:.1f} x {w:.1f} x {t:.1f} mm  |  "
                f"joints {data['joints']} ({data['movable_joints']} movable + {data['fixed_joints']} fixed)  |  links {data['links']}")
        draw.text((12, top + header + ch + 10), text, fill="#222222", font=font(25))
    total.save(output / "comparison.jpg", quality=90, subsampling=0)
    for model in metadata:
        Image.open(output / f"{model}_three_quarter.png").convert("RGB").save(
            output / f"{model}_representative.jpg", quality=89, subsampling=0)
    # Original 2000x1500 photos: crop + uniform resize only, no rotation,
    # color correction, undistortion, retouching, or generated pixels.
    crops = {"0": (270, 640, 700, 1000), "3": (630, 600, 1460, 1050)}
    panel = Image.new("RGB", (5 * 640, 680), "#fafafa")
    d = ImageDraw.Draw(panel)
    labels = ["Photo 0: hand", "Photo 3: wrist mount only", *metadata.keys()]
    for i, label in enumerate(labels):
        d.text((i * 640 + 10, 12), label.replace("_", " / "), fill="#222222", font=font(26))
        if i < 2:
            name = str(i * 3)
            picture = Image.open(photos / f"{name}.jpg").crop(crops[name])
            picture.save(output / f"photo_{name}_crop.png")
        else:
            model = list(metadata)[i - 2]
            picture = Image.open(output / f"{model}_three_quarter.png").convert("RGB")
        picture.thumbnail((620, 580), Image.Resampling.LANCZOS)
        panel.paste(picture, (i * 640 + (640 - picture.width) // 2, 58 + (580 - picture.height) // 2))
    d.text((12, 646), "Photos: crop + resize only. Photo pose, camera and scale differ from simulation. Photo 3 does not show a complete hand.", fill="#222222", font=font(22))
    panel.save(output / "physical_reference.jpg", quality=90, subsampling=0)
    return {n: dict(file=str(photos / f"{n}.jpg"), sha256=sha(photos / f"{n}.jpg"), crop_xyxy=list(box)) for n, box in crops.items()}


def render(output, metadata):
    if os.environ.get("CUDA_VISIBLE_DEVICES") != "3":
        raise RuntimeError("Run with CUDA_VISIBLE_DEVICES=3 (physical GPU 3 only)")
    import sapien
    from sapien.render import RenderBodyComponent, RenderMaterial, RenderShapeTriangleMesh
    from xbox_gamepad_physics.examples.scene import create_scene, look_at, take_picture

    scene_cfg = dict(render=True, ground=False, timestep=1 / 240, robotwin_root=None)
    # Keep the example RT scene/lighting; use 32 spp for readable closeups.
    scene = create_scene(scene_cfg)
    sapien.render.set_ray_tracing_samples_per_pixel(32)
    background = RenderMaterial(base_color=[.88, .88, .88, 1], roughness=1, metallic=0)
    scene.add_ground(-.008, render_material=background)
    cameras = {}
    camera_info = {}
    for view, (direction, up) in VIEWS.items():
        direction = np.asarray(direction, dtype=float)
        position = TARGET + DISTANCE * direction / np.linalg.norm(direction)
        camera = scene.add_camera(view, 1280, 960, math.radians(FOVY), .01, 3.)
        camera.entity.set_pose(look_at(position, TARGET, up_hint=up))
        cameras[view] = camera
        camera_info[view] = dict(position=position.tolist(), target=TARGET.tolist(), up_hint=up,
                                 width=1280, height=960, fovy_degrees=FOVY, near=.01, far=3., distance=DISTANCE)
    material = RenderMaterial(**MATERIAL)
    for model, data in metadata.items():
        loader = scene.create_urdf_loader()
        loader.fix_root_link = True
        articulation = loader.load(data["derived_urdf"])
        articulation.set_root_pose(sapien.Pose())
        articulation.set_qpos(np.zeros(articulation.dof))
        articulation.set_qvel(np.zeros(articulation.dof))
        applied = 0
        max_error = 0.
        for link in articulation.get_links():
            max_error = max(max_error, float(np.max(np.abs(
                link.pose.to_transformation_matrix() - np.array(data["zero_fk"][link.name])))))
            body = link.entity.find_component_by_type(RenderBodyComponent)
            if body is None:
                continue
            replacement = RenderBodyComponent()
            for shape in body.render_shapes:
                if not isinstance(shape, RenderShapeTriangleMesh):
                    raise ValueError(f"unexpected primitive: {model}/{link.name}")
                gray = RenderShapeTriangleMesh(shape.filename, shape.scale, material)
                gray.local_pose = shape.local_pose
                replacement.attach(gray)
                applied += 1
            link.entity.remove_component(body)
            link.entity.add_component(replacement)
        assert applied == data["visual_shapes"]
        assert max_error < 1e-6
        data.update(engine_dof=articulation.dof, qpos=articulation.get_qpos().tolist(),
                    gray_shapes=applied, numpy_vs_sapien_zero_fk_max_error=max_error)
        for view, camera in cameras.items():
            take_picture(scene, camera, output / f"{model}_{view}.png")
            segmentation = camera.get_picture("Segmentation")[..., 1]
            ids = [link.entity.per_scene_id for link in articulation.get_links()]
            mask = np.isin(segmentation, ids)
            ys, xs = np.where(mask)
            assert len(xs) > 1000, (model, view, "empty image")
            bbox = [int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())]
            assert bbox[0] > 0 and bbox[1] > 0 and bbox[2] < 1279 and bbox[3] < 959, (model, view, bbox)
            data.setdefault("images", {})[view] = dict(file=f"{model}_{view}.png", silhouette_bbox_px=bbox, silhouette_pixels=int(mask.sum()))
            print(model, view, bbox, flush=True)
        scene.remove_articulation(articulation)
    return dict(scene_entry="xbox_gamepad_physics.examples.scene.create_scene", scene_config=scene_cfg,
                shader="rt", samples_per_pixel=32, path_depth=8, denoiser="oidn",
                material_linear=MATERIAL, cameras=camera_info,
                device_summary=sapien.render.get_device_summary(),
                render_device=str(scene.render_system.device),
                sapien_version=sapien.__version__)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--assets", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--photos", type=Path, required=True)
    p.add_argument("--measure-only", action="store_true")
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=True)
    work = a.output / "derived_urdf"
    work.mkdir(exist_ok=True)
    metadata = {model: inspect_model(a.assets / path, work) for model, path in MODELS.items()}
    result = dict(models=metadata, script_sha256=sha(__file__))
    if not a.measure_only:
        result["render"] = render(a.output, metadata)
        result["photos"] = compose(a.output, metadata, a.photos)
    (a.output / "metadata.json").write_text(json.dumps(result, indent=2) + "\n")
    for model, data in metadata.items():
        print(model, data["length_width_thickness_mm"], data["joints"], data["links"], flush=True)


if __name__ == "__main__":
    main()
