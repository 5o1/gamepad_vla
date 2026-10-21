"""Backend-independent robot contact classification, in metres.

Adapters provide geometry distance and whether either body belongs to a robot.
Positive-margin candidate pairs are not geometric contact. Every native substep
must contribute to the control interval's OR, even when the final pose separates.
"""
import math


def has_robot_contact(samples):
    for distance, is_robot_pair in samples:
        distance = float(distance)
        if not math.isfinite(distance):
            raise ValueError("contact distance must be finite")
        if is_robot_pair and distance <= 0.:
            return True
    return False
