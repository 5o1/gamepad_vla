"""Operator-only cameras and a nonoverlapping atlas, shared by live/offline views.

Controller axes: +X right, +Y shoulder/USB edge, +Z button face. These cameras
never enter the collection camera list or episode schema.
"""
import math


TILE_WIDTH, TILE_HEIGHT = 640, 540
AUXILIARY_FPS = 20
AUXILIARY_VIEWS = ('face', 'front', 'back', 'left', 'right', 'overview')
ATLAS_WIDTH, ATLAS_HEIGHT = TILE_WIDTH * 3, TILE_HEIGHT * 2

# Position, target and up are controller-local, in metres.
CAMERA_OFFSETS = {
    'face': ((0., .005, .34), (0., .005, 0.), (0., 1., 0.)),
    'front': ((0., -.15, .28), (0., 0., .015), (0., 0., 1.)),
    'back': ((0., .205, -.23), (0., .015, -.005), (0., 1., 0.)),
    'left': ((-.24, .19, .28), (0., .01, 0.), (0., 0., 1.)),
    'right': ((.24, .19, .28), (0., .01, 0.), (0., 0., 1.)),
}


def selected_views(values):
    """Validate browser demand and keep the fixed atlas order."""
    if not isinstance(values, list) or any(v not in AUXILIARY_VIEWS for v in values):
        raise ValueError('expected a list of auxiliary camera names')
    return [name for name in AUXILIARY_VIEWS if name in values]


def camera_vectors(transform, name):
    import numpy as np
    if name == 'overview':
        # World-vertical overview, shifted toward the arms to keep their approach
        # area in frame without spending half the image on the far table.
        target = transform[:3, 3] + (-.10, 0., 0.)
        return target + (0., 0., .48), target, np.array((1., 0., 0.))
    position, target, up = CAMERA_OFFSETS[name]
    def world(point):
        return (transform @ np.r_[point, 1.])[:3]
    return world(position), world(target), transform[:3, :3] @ up


def auxiliary_options(cfg=None):
    values = dict(width=ATLAS_WIDTH, height=ATLAS_HEIGHT, fps=AUXILIARY_FPS)
    values.update((cfg or {}).get('auxiliary', {}))
    for key in ('width', 'height', 'fps'):
        if type(values[key]) is not int:
            raise ValueError('auxiliary dimensions and fps must be integers')
    if values['width'] < 1920 or values['height'] < 1080 or values['width'] % 2 or values['height'] % 2 or not 1 <= values['fps'] <= 60:
        raise ValueError('auxiliary stream requires even dimensions >=1920x1080 and 1..60 fps')
    return values


def frame_capacity(width, height):
    return width * height * 3 + 65536


def scaled_camera_definition(definition, width, height):
    from dataclasses import replace
    intr = definition.intrinsics
    sx, sy = width / intr.width, height / intr.height
    return replace(definition, intrinsics=replace(intr, width=width, height=height,
        fx=intr.fx*sx, fy=intr.fy*sy, cx=intr.cx*sx, cy=intr.cy*sy))


def atlas_tiles(cfg=None):
    options = auxiliary_options(cfg)
    width, height = options['width'], options['height']
    # Integer partition fills even a 2560-pixel atlas exactly (853/853/854).
    return {name: dict(x=(i % 3)*width//3, y=(i // 3)*height//2,
                       width=((i % 3)+1)*width//3-(i % 3)*width//3,
                       height=height//2)
            for i, name in enumerate(AUXILIARY_VIEWS)}


class AuxiliaryCameras:
    """Lazy camera creation; disabled cameras never take a picture."""
    def __init__(self, backend, cfg=None):
        from .spec import pose_to_matrix
        self.backend = backend
        self.cfg = cfg or {}
        self.options = auxiliary_options(self.cfg)
        self.transform = pose_to_matrix(backend.spec.gamepad.root_pose)
        self.cameras = {}

    def capture(self, names):
        import numpy as np
        from xbox_gamepad_physics.examples.scene import look_at
        from .collection_camera import color_to_rgb8
        atlas = np.zeros((self.options['height'], self.options['width'], 3), dtype=np.uint8)
        tiles = atlas_tiles(self.cfg)
        for name in selected_views(names):
            tile = tiles[name]
            if name not in self.cameras:
                camera = self.backend.scene.add_camera('aux_' + name, tile['width'], tile['height'],
                                                       math.radians(50), .001, 20.)
                camera.entity.set_pose(look_at(*camera_vectors(self.transform, name)))
                self.cameras[name] = camera
                # SAPIEN requires update_render after adding a camera.
                self.backend.scene.update_render()
            camera = self.cameras[name]
            camera.take_picture()
            tile = tiles[name]
            atlas[tile['y']:tile['y'] + tile['height'], tile['x']:tile['x'] + tile['width']] = color_to_rgb8(camera.get_picture('Color'))
        return atlas
