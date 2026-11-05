"""Independent full-resolution SAPIEN pictures of one frozen physical state."""
from dataclasses import replace
import hashlib
import io
import json
import math
from pathlib import Path
import subprocess
import time


def code_identity():
    from gamepad_vla.site import repository_root
    root = repository_root()
    try:
        result = subprocess.run(['git','-C',str(root),'rev-parse','HEAD'], capture_output=True, text=True)
        revision = result.stdout.strip() if result.returncode == 0 else None
        status = subprocess.run(['git','-C',str(root),'status','--porcelain','--untracked-files=no'],capture_output=True,text=True) if revision else None
        dirty = bool(status.stdout) if status and status.returncode == 0 else None
    except OSError:
        revision = dirty = None
    files = ('operator_view.py','operator_stream.py','auxiliary_views.py','scene_snapshot.py',
             'collection_server.py','collection_sapien.py','sapien_backend.py',
             'collection_camera.py','collection_snapshots.py','collection_sources.py','collection_controller.py',
             'arm_mapping.py','g20_mapping.py')
    here = Path(__file__).parent
    return dict(git_revision=revision, git_worktree_dirty=dirty,
                source_sha256={name:hashlib.sha256((here/name).read_bytes()).hexdigest() for name in files})


def snapshot_cameras(backend, width, height, names=None):
    """Dedicated native-resolution cameras; caller owns their lifetimes."""
    from xbox_gamepad_physics.examples.scene import look_at
    from .auxiliary_views import AUXILIARY_VIEWS, camera_vectors
    from .render_review import create_spec_camera
    from .spec import pose_to_matrix
    original = next(c for c in backend.spec.cameras if c.name == 'camera_3')
    width, height = max(width, original.intrinsics.width), max(height, original.intrinsics.height)
    selected = set(names) if names is not None else {'camera_3', *('aux_'+n for n in AUXILIARY_VIEWS)}
    intr = original.intrinsics
    sx, sy = width/intr.width, height/intr.height
    definition = replace(original, name='snapshot_camera_3', intrinsics=replace(intr,
        width=width,height=height,fx=intr.fx*sx,fy=intr.fy*sy,cx=intr.cx*sx,cy=intr.cy*sy))
    cameras = {'camera_3':create_spec_camera(backend, definition)} if 'camera_3' in selected else {}
    transform = pose_to_matrix(backend.spec.gamepad.root_pose)
    for name in AUXILIARY_VIEWS:
        if 'aux_'+name not in selected:
            continue
        camera = backend.scene.add_camera('snapshot_aux_'+name, width, height, math.radians(50), .001, 20.)
        camera.entity.set_pose(look_at(*camera_vectors(transform, name)))
        cameras['aux_'+name] = camera
    return cameras


def render_pending_snapshot(backend, job, cfg):
    """No scene.step: each camera observes exactly the same frozen joint values."""
    import numpy as np
    from PIL import Image
    from .operator_view import apply_render_state
    from .collection_camera import color_to_rgb8
    folder = Path(job['folder'])
    result = {}
    try:
        state = json.loads((folder/'state.json').read_text())
        apply_render_state(backend, state)
        from .auxiliary_views import AUXILIARY_VIEWS
        width, height = max(job['width'],cfg.get('width',1280)), max(job['height'],cfg.get('height',720))
        used = sum(p.stat().st_size for p in folder.iterdir())
        images = {}
        for name in ('camera_3', *('aux_'+n for n in AUXILIARY_VIEWS)):
            if not folder.exists():
                return  # The API owner abandoned a timed-out request.
            camera = snapshot_cameras(backend, width, height, [name])[name]
            try:
                backend.scene.update_render()
                start = time.monotonic_ns()
                camera.take_picture()
                rgb = color_to_rgb8(camera.get_picture('Color'))
                end = time.monotonic_ns()
                stream = io.BytesIO()
                Image.fromarray(rgb).save(stream, format='PNG')
                data = stream.getvalue()
                used += len(data)
                if used > job['max_bytes']:
                    raise RuntimeError('Snapshot images exceed the configured size limit.')
                (folder/(name+'.png')).write_bytes(data)
                world_from_gl = np.asarray(camera.get_model_matrix())
                world_from_optical = world_from_gl @ np.diag([1.,-1.,-1.,1.])
                images[name] = dict(file=name+'.png', width=rgb.shape[1],height=rgb.shape[0],
                    intrinsic_matrix=np.asarray(camera.get_intrinsic_matrix()).tolist(),
                    world_from_camera_opengl=world_from_gl.tolist(),
                    world_from_camera_optical=world_from_optical.tolist(),
                    camera_optical_from_world=np.linalg.inv(world_from_optical).tolist(),
                    optical_axes='x right, y down, z forward', near=.001,far=20.,
                    render_started_monotonic_ns=start,render_finished_monotonic_ns=end,
                    source='independent_sapien_camera_take_picture', bytes=len(data))
            finally:
                # Keep only one full-resolution snapshot target resident. A mounted
                # camera shares a robot entity: detach the camera, not its owner.
                entity = camera.entity
                entity.remove_component(camera)
                if not entity.components:
                    backend.scene.remove_entity(entity)
                del camera
        from .g20_mapping import G20_JOINT_NAMES
        result = dict(cameras=images, camera_count=len(images), frozen_step=state.get('step'),
            arm_joint_names={side:list(getattr(backend.spec,side+'_arm').joint_names) for side in ('left','right')},
            hand_joint_names=list(G20_JOINT_NAMES), gamepad_joint_names=[j.name for j in backend.gamepad.articulation.get_active_joints()],
            units=dict(arm_q='rad',hand_q='rad',hand_motors='0..255',raw_master='deg',position='m',time='ns'),
            gamepad_root_world_matrix=backend.gamepad.articulation.get_root_pose().to_transformation_matrix().tolist(),
            fixture_world_matrices={f.name:f.get_pose().to_transformation_matrix().tolist() for f in backend.fixtures},
            scene_spec_name=backend.spec.name, scene_spec_file=cfg.get('spec_name'), code_version=cfg.get('code_identity') or code_identity(),
            snapshot_note='All cameras independently rendered at native resolution from the same frozen state; physics was not stepped.',
            render_device=str(backend.scene.render_system.device))
        if cfg.get('spec_text'):
            text = cfg['spec_text']
            (folder/'scene-spec.json').write_text(text)
            result['scene_spec_sha256'] = hashlib.sha256(text.encode()).hexdigest()
        else:
            raise RuntimeError('Scene spec provenance is missing from operator_view config.')
    except Exception as exc:
        result = dict(error=str(exc))
    temporary = folder/'render-result.tmp'
    try:
        temporary.write_text(json.dumps(result, indent=2))
        temporary.replace(folder/'render-result.json')
    except FileNotFoundError:
        pass  # The API owner removed an abandoned capture; keep live views running.
