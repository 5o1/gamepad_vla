"""The calibrated v4.4 ego view, applied only to independent render replicas."""
import time
import os
import numpy as np
from .spec import pose_to_matrix


def color_to_rgb8(picture):
    """Preserve clip/scale/round pixels using contiguous RGB ufunc inputs."""
    import cv2
    rgb = cv2.cvtColor(picture, cv2.COLOR_RGBA2RGB)
    np.clip(rgb, 0., 1., out=rgb)
    np.multiply(rgb, 255, out=rgb)
    np.rint(rgb, out=rgb)
    return rgb.astype(np.uint8)


class EgoRenderer:
    def __init__(self, backend):
        self.backend = backend
        self.timings = {}
        self.pixel_checks = 0
        self.definition = next(c for c in backend.spec.cameras if c.name=='camera_3')
        if self.definition.parent != backend.spec.world_frame:
            raise ValueError('collection ego camera must be fixed in world')
        self.intr = self.definition.intrinsics
        if backend.name=='sapien':
            from .render_review import create_spec_camera
            self.camera = create_spec_camera(backend,self.definition)
        else:
            backend.model.vis.global_.offwidth = self.intr.width
            backend.model.vis.global_.offheight = self.intr.height
            self.renderer = backend.mj.Renderer(backend.model,height=self.intr.height,width=self.intr.width)
            # Reuse backend.close ownership, without touching its stepping path.
            backend._renderer = self.renderer

    def render(self):
        b=self.backend
        if b.name=='sapien':
            t0=time.monotonic_ns();b.scene.update_render();t1=time.monotonic_ns()
            self.camera.take_picture();t2=time.monotonic_ns()
            picture=self.camera.get_picture('Color');t3=time.monotonic_ns()
            rgb=color_to_rgb8(picture);t4=time.monotonic_ns()
            self.timings=dict(scene_update_ms=(t1-t0)/1e6,take_picture_ms=(t2-t1)/1e6,
                              readback_ms=(t3-t2)/1e6,rgb_convert_ms=(t4-t3)/1e6)
            if os.environ.get('GAMEPAD_VLA_VERIFY_PIXELS')=='1' and self.pixel_checks<3:
                reference=np.rint(np.clip(picture[...,:3],0.,1.)*255).astype(np.uint8)
                equal=np.array_equal(rgb,reference)
                self.timings['rgb_reference_equal']=bool(equal)
                self.pixel_checks+=1
                if not equal:raise RuntimeError('RGB conversion changed rendered pixels')
            return rgb
        import cv2
        self.renderer.update_scene(b.data)
        transform=pose_to_matrix(list(self.definition.pose_optical))
        intr=self.intr
        for camera in self.renderer.scene.camera:
            camera.pos[:]=transform[:3,3]
            camera.forward[:]=transform[:3,2]
            camera.up[:]=-transform[:3,1]
            camera.frustum_near=.001;camera.frustum_far=20.
            camera.frustum_top=intr.cy*.001/intr.fy
            camera.frustum_bottom=-(intr.height-intr.cy)*.001/intr.fy
            camera.frustum_center=(intr.width/2-intr.cx)*.001/intr.fy
            camera.orthographic=0
        rgb=self.renderer.render()
        # mjvGLCamera derives horizontal FOV from viewport aspect. Correct fx/fy
        # in the rendered image; preserve the measured off-centre principal point.
        scale=intr.fx/intr.fy
        return cv2.warpAffine(rgb,np.array([[scale,0,intr.cx*(1-scale)],[0,1,0]]),
                              (intr.width,intr.height))
