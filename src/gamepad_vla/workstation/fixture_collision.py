"""Static fixture geometry and explicit collection collision policy.

OBJ coordinates are metres; each object is one closed convex hull. Runtime
loading uses numpy only (CAD/decomposition dependencies stay in the CAD env).
"""
from pathlib import Path

import numpy as np

CONTACT_OFFSET = 0.0002
FIXTURE_GAMEPAD_IGNORE_BIT = 0x20000000
GAMEPAD_FILTER_ID = 0x5842


def convex_objects(path):
    """Read triangular multi-object OBJ and reject PhysX-incompatible hulls."""
    vertices, faces, objects = [], [], []
    for raw in Path(path).read_text().splitlines():
        fields = raw.split()
        if not fields or fields[0].startswith('#'):
            continue
        if fields[0] in ('o', 'g'):
            if faces:
                objects.append(faces)
                faces = []
        elif fields[0] == 'v':
            vertices.append([float(v) for v in fields[1:4]])
        elif fields[0] == 'f':
            if len(fields) != 4:
                raise ValueError('convex OBJ faces must be triangles')
            ids = [int(v.split('/')[0]) for v in fields[1:]]
            faces.append([i - 1 if i > 0 else len(vertices) + i for i in ids])
    if faces:
        objects.append(faces)
    result = []
    all_vertices = np.asarray(vertices, float)
    for faces in objects:
        used = np.unique(faces)
        if len(used) < 4 or len(used) > 255 or used.min() < 0 or used.max() >= len(vertices):
            raise ValueError('convex OBJ object must have 4..255 referenced vertices')
        v = all_vertices[used]
        f = np.searchsorted(used, faces)
        if not np.isfinite(v).all():
            raise ValueError('convex OBJ vertices must be finite')
        if np.linalg.matrix_rank(v - v.mean(axis=0), tol=1e-12) < 3:
            raise ValueError('convex OBJ object must enclose volume')
        triangles = v[f]
        normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
        norms = np.linalg.norm(normals, axis=1)
        if np.any(norms <= 1e-15):
            raise ValueError('convex OBJ has degenerate faces')
        normals /= norms[:, None]
        distance = np.einsum('fvi,fi->fv', v[None, :, :] - triangles[:, :1, :], normals)
        edges = np.sort(np.concatenate([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]]), axis=1)
        if np.any(np.unique(edges, axis=0, return_counts=True)[1] != 2) or distance.max() > 1e-7:
            raise ValueError('OBJ object must be a closed outward-wound convex hull')
        result.append((v, f))
    if not result:
        raise ValueError('convex OBJ contains no hulls')
    return result


def add_sapien_fixture_collision(builder, fixture):
    if fixture.type == 'box':
        builder.add_box_collision(half_size=list(fixture.half_extents))
    elif fixture.type == 'convex':
        convex_objects(fixture.mesh)
        builder.add_multiple_convex_collisions_from_file(fixture.mesh, decomposition='none')
    else:
        builder.add_nonconvex_collision_from_file(fixture.mesh)


def sapien_shapes(actor):
    # Static builders return an Entity in 3.0.3; articulation links expose the
    # convenience method directly.
    if hasattr(actor, 'get_collision_shapes'):
        return actor.get_collision_shapes()
    import sapien
    # Entity.find_component_by_type matches concrete component types in 3.0.3;
    # querying the RigidBody base silently returns None for static actors.
    for kind in (sapien.physx.PhysxRigidStaticComponent,
                 sapien.physx.PhysxRigidDynamicComponent):
        body = actor.find_component_by_type(kind)
        if body is not None:
            return body.get_collision_shapes()
    return []


def configure_sapien_shapes(actor, *, exclude_fixture_gamepad=False):
    for shape in sapien_shapes(actor):
        shape.set_contact_offset(CONTACT_OFFSET)
        if exclude_fixture_gamepad:
            groups = list(shape.get_collision_groups())
            groups[2] |= FIXTURE_GAMEPAD_IGNORE_BIT
            groups[3] = GAMEPAD_FILTER_ID
            shape.set_collision_groups(groups)


def add_mujoco_fixture(parent, mj, fixture):
    masks = dict(contype=8 if fixture.collision else 0,
                 conaffinity=6 if fixture.collision else 0, margin=CONTACT_OFFSET)
    pose = dict(pos=fixture.pose[:3], quat=fixture.pose[3:])
    if fixture.type == 'box':
        parent.worldbody.add_geom(name=fixture.name, type=mj.mjtGeom.mjGEOM_BOX,
                                  size=fixture.half_extents, **pose, **masks)
    elif fixture.collision:
        source = fixture.mesh if fixture.type == 'convex' else fixture.convex_mesh
        if source is None:
            raise ValueError('MuJoCo mesh fixture collision requires convex_mesh')
        for index, (vertices, faces) in enumerate(convex_objects(source)):
            name = f'{fixture.name}_convex_{index:03d}'
            parent.add_mesh(name=name, uservert=vertices.ravel().tolist(), userface=faces.ravel().tolist())
            parent.worldbody.add_geom(name=name, type=mj.mjtGeom.mjGEOM_MESH,
                                      meshname=name, **pose, **masks)
