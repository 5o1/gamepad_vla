"""Object-relative augmentation of collection v4 demonstrations.

Generation proposes commands. Only native simulator replay can accept data.
"""

from .episode import load_demonstration
from .segments import SliceConfig, slice_stream

__all__ = ["load_demonstration", "SliceConfig", "slice_stream"]
