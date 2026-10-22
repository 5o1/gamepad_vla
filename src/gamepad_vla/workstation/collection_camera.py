"""The calibrated v4.4 ego view, applied only to independent render replicas."""
import numpy as np


def color_to_rgb8(picture):
    """Preserve clip/scale/round pixels using contiguous RGB ufunc inputs."""
    import cv2
    rgb = cv2.cvtColor(picture, cv2.COLOR_RGBA2RGB)
    np.clip(rgb, 0., 1., out=rgb)
    np.multiply(rgb, 255, out=rgb)
    np.rint(rgb, out=rgb)
    return rgb.astype(np.uint8)
