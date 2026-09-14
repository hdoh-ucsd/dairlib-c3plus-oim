"""Shared visualization camera pose; imported only by rendering backends."""
import numpy as np
from pydrake.math import RigidTransform, RotationMatrix

EYE = np.array([1.3, -0.9, 0.9])
TARGET = np.array([0.4, 0.0, 0.1])


def look_at(eye, target):
    """Camera pose: +Z forward (view direction), +X right, +Y down."""
    fwd = target - eye
    fwd = fwd / np.linalg.norm(fwd)
    up = np.array([0.0, 0.0, 1.0])
    right = np.cross(fwd, up)
    right = right / np.linalg.norm(right)
    down = np.cross(fwd, right)
    R = np.column_stack([right, down, fwd])
    return RigidTransform(RotationMatrix(R), eye)
