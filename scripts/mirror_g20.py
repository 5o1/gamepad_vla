"""Generate and audit an UNVERIFIED right-hand candidate; never drive hardware.

URDF origin/axis semantics: https://docs.ros.org/en/rolling/p/urdfdom/
Reflection is a project implementation, not a manufacturer-provided right hand.
All local frames are reflected with S=diag(1,-1,1). Rotations become S R S;
revolute axes (axial vectors) become det(S) S a, keeping q and mimic unchanged.
"""

import argparse
import copy
import hashlib
import json
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

COMMIT = "075cc7d42cc1e756bdcbece0fc069a0779fc5237"
ROOT = f"https://raw.githubusercontent.com/linker-bot/linkerhand-urdf/{COMMIT}"
SOURCE = "archive/lhg20/left"
S = np.diag([1.0, -1.0, 1.0])
H = np.diag([1.0, -1.0, 1.0, 1.0])
DTYPE = np.dtype([("normal", "<f4", (3,)), ("v", "<f4", (3, 3)), ("attr", "<u2")])


def vec(value):
    return np.fromstring(value, sep=" ")


def fmt(value):
    return " ".join(f"{x:.15g}" for x in value)


def download(path, dest):
    dest.parent.mkdir(parents=True, exist_ok=True)
    # Explicit physical interface and real IP bypass fake DNS and inherited proxies.
    subprocess.run(
        [
            "curl",
            "-q",
            "--proxy",
            "",
            "--noproxy",
            "*",
            "--interface",
            "eno2",
            "--resolve",
            "raw.githubusercontent.com:443:185.199.108.133",
            "--connect-timeout",
            "10",
            "--max-time",
            "60",
            "--retry",
            "2",
            "-fsS",
            f"{ROOT}/{path}",
            "-o",
            str(dest),
        ],
        check=True,
    )


def stl(path):
    raw = path.read_bytes()
    count = int.from_bytes(raw[80:84], "little")
    assert len(raw) == 84 + 50 * count, f"Not binary STL: {path}"
    return raw[:84], np.frombuffer(raw, dtype=DTYPE, offset=84).copy()


def transform(origin):
    t = np.eye(4)
    if origin is not None:
        t[:3, 3] = vec(origin.get("xyz", "0 0 0"))
        t[:3, :3] = Rotation.from_euler(
            "xyz", vec(origin.get("rpy", "0 0 0"))
        ).as_matrix()
    return t


def fk(root, q):
    joints = root.findall("joint")
    children = {j.find("child").get("link") for j in joints}
    roots = {l.get("name") for l in root.findall("link")} - children
    assert len(roots) == 1
    poses = {roots.pop(): np.eye(4)}
    q = dict(q)
    for _ in range(len(joints) + 1):
        for j in joints:
            mimic = j.find("mimic")
            if mimic is not None and mimic.get("joint") in q:
                q[j.get("name")] = q[mimic.get("joint")] * float(
                    mimic.get("multiplier", 1)
                ) + float(mimic.get("offset", 0))
        for j in joints:
            parent, child = j.find("parent").get("link"), j.find("child").get("link")
            if parent not in poses or child in poses:
                continue
            motion = np.eye(4)
            if j.get("type") != "fixed":
                if j.get("name") not in q:
                    continue
                axis = vec(j.find("axis").get("xyz"))
                motion[:3, :3] = Rotation.from_rotvec(
                    axis * q[j.get("name")]
                ).as_matrix()
            poses[child] = poses[parent] @ transform(j.find("origin")) @ motion
    assert len(poses) == len(root.findall("link"))
    return poses


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    left, right = args.output / "left", args.output / "right_candidate"
    right.mkdir()
    download(
        f"{SOURCE}/linkerhand_lhg20_left.urdf", left / "linkerhand_lhg20_left.urdf"
    )
    download("LICENSE", args.output / "LICENSE")
    root = ET.parse(left / "linkerhand_lhg20_left.urdf").getroot()
    hashes = {}
    mesh_names = sorted({e.get("filename") for e in root.iter("mesh")})
    for name in mesh_names:
        download(f"{SOURCE}/{name}", left / name)
        hashes[name] = hashlib.sha256((left / name).read_bytes()).hexdigest()
        header, data = stl(left / name)
        data["normal"] = data["normal"] @ S
        data["v"] = (data["v"] @ S)[:, [0, 2, 1], :]
        (right / name).parent.mkdir(parents=True, exist_ok=True)
        (right / name).write_bytes(header + data.tobytes())
    mirrored = copy.deepcopy(root)
    mirrored.set("name", "linkerhand_lhg20_right_MIRRORED_UNVERIFIED")
    for origin in mirrored.iter("origin"):
        t = H @ transform(origin) @ H
        origin.set("xyz", fmt(t[:3, 3]))
        origin.set("rpy", fmt(Rotation.from_matrix(t[:3, :3]).as_euler("xyz")))
    for joint in mirrored.findall("joint"):
        assert joint.get("type") in ("revolute", "continuous", "fixed")
        axis = joint.find("axis")
        if axis is not None:
            axis.set("xyz", fmt(-S @ vec(axis.get("xyz"))))
    eigenvalues = []
    for inertia in mirrored.iter("inertia"):
        i = np.array(
            [
                [
                    float(inertia.get("i" + a + b if a <= b else "i" + b + a))
                    for b in "xyz"
                ]
                for a in "xyz"
            ]
        )
        i = S @ i @ S
        eigenvalues.extend(np.linalg.eigvalsh(i).tolist())
        for a, b in ("xx", "xy", "xz", "yy", "yz", "zz"):
            inertia.set("i" + a + b, f"{i['xyz'.index(a), 'xyz'.index(b)]:.15g}")
    ET.indent(mirrored)
    output_urdf = right / "linkerhand_lhg20_right_mirrored.urdf"
    ET.ElementTree(mirrored).write(output_urdf, encoding="utf-8", xml_declaration=True)
    # Validate the serialized artifact, not only the in-memory transformation.
    mirrored = ET.parse(output_urdf).getroot()
    independent = [
        j
        for j in root.findall("joint")
        if j.get("type") != "fixed" and j.find("mimic") is None
    ]
    rng = np.random.default_rng(20261006)
    max_error = 0.0
    for _ in range(100):
        q = {
            j.get("name"): rng.uniform(
                float(j.find("limit").get("lower")), float(j.find("limit").get("upper"))
            )
            for j in independent
        }
        lp, rp = fk(root, q), fk(mirrored, q)
        for name in lp:
            max_error = max(
                max_error, float(np.max(np.abs(rp[name] - H @ lp[name] @ H)))
            )
    mesh_error = 0.0
    volume_error = 0.0
    for name in mesh_names:
        _, a = stl(left / name)
        _, b = stl(right / name)
        mesh_error = max(
            mesh_error, float(np.max(np.abs(b["v"][:, [0, 2, 1]] - a["v"] @ S)))
        )

        def volume(v):
            return (
                np.einsum(
                    "ij,ij->i",
                    v[:, 0].astype(float),
                    np.cross(v[:, 1].astype(float), v[:, 2].astype(float)),
                ).sum()
                / 6
            )

        volume_error = max(volume_error, abs(volume(a["v"]) - volume(b["v"])))
    assert max_error < 1e-10 and mesh_error < 1e-7 and min(eigenvalues) > 0
    report = dict(
        upstream_commit=COMMIT,
        status="MIRRORED_UNVERIFIED_NOT_HARDWARE_CALIBRATED",
        plane="XZ (negate Y)",
        links=len(root.findall("link")),
        joints=len(root.findall("joint")),
        independent_urdf_joints=len(independent),
        mimic_joints=len(root.findall(".//mimic")),
        mesh_count=len(mesh_names),
        random_poses=100,
        max_fk_matrix_error=max_error,
        max_mesh_error_m=mesh_error,
        max_signed_volume_error_m3=volume_error,
        minimum_inertia_eigenvalue=min(eigenvalues),
        source_mesh_sha256=hashes,
        limits_and_mimic="unchanged because q is preserved; not validated against real right-hand SDK",
        limitations=[
            "No measured dynamics",
            "No right-hand hardware verification",
            "Visual mesh reused as collision, as upstream",
            "No contact-control training",
        ],
    )
    (args.output / "audit.json").write_text(json.dumps(report, indent=2))
    render(root, mirrored, left, right, independent, args.output)
    print(
        json.dumps(
            {k: v for k, v in report.items() if k != "source_mesh_sha256"}, indent=2
        )
    )


def render(left_root, right_root, left_dir, right_dir, independent, output):
    """Orthographic triangle preview, not a contact simulation or camera result."""
    from PIL import Image, ImageDraw

    image = Image.new("RGB", (1400, 900), "#101820")
    draw = ImageDraw.Draw(image)
    for row, fraction in enumerate((0.0, 0.55)):
        q = {
            j.get("name"): fraction * float(j.find("limit").get("upper"))
            for j in independent
        }
        for col, (root, directory) in enumerate(
            ((left_root, left_dir), (right_root, right_dir))
        ):
            poses = fk(root, q)
            triangles = []
            for link in root.findall("link"):
                visual = link.find("visual")
                _, mesh = stl(directory / visual.find("geometry/mesh").get("filename"))
                t = poses[link.get("name")] @ transform(visual.find("origin"))
                vertices = mesh["v"].astype(float) @ t[:3, :3].T + t[:3, 3]
                triangles.append(vertices)
            v = np.concatenate(triangles)
            # Look along X: horizontal Y, vertical Z, depth X.
            center = (v.min(axis=(0, 1)) + v.max(axis=(0, 1))) / 2
            xy = (
                np.stack((v[:, :, 1] - center[1], -(v[:, :, 2] - center[2])), -1) * 1150
            )
            xy += np.array([350 + 700 * col, 250 + 430 * row])
            normals = np.cross(v[:, 1] - v[:, 0], v[:, 2] - v[:, 0])
            normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
            shade = 0.35 + 0.65 * np.abs(
                normals @ np.array([0.8, 0.3, 0.5]) / np.linalg.norm([0.8, 0.3, 0.5])
            )
            for idx in np.argsort(v[:, :, 0].mean(1)):
                color = tuple(int(c * shade[idx]) for c in (165, 207, 225))
                draw.polygon([tuple(p) for p in xy[idx]], fill=color)
            draw.text(
                (col * 700 + 100, row * 430 + 25),
                f"{'OFFICIAL LEFT' if col == 0 else 'MIRRORED RIGHT (UNVERIFIED)'} / pose {row}",
                fill="white",
            )
    draw.text(
        (35, 865),
        "Geometric / kinematic preview only. Not a physical manipulation result.",
        fill="white",
    )
    image.save(output / "comparison.png")


if __name__ == "__main__":
    main()
