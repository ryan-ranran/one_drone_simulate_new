import math

import cv2
import numpy as np


def solve_tag_pose(corners, camera_matrix, distortion, tag_size):
    """这是一个负责计算的文件，和ros无关，只是由标签算位置"""
    half = float(tag_size) * 0.5
    if not math.isfinite(half) or half <= 0:
        raise ValueError('tag_size must be positive and finite')
    object_points = np.array([
        [-half, -half, 0.0],
        [half, -half, 0.0],
        [half, half, 0.0],
        [-half, half, 0.0],
    ], dtype=np.float64)
    image_points = np.asarray(corners, dtype=np.float64).reshape(4, 2)
    ok, rvec, tvec = cv2.solvePnP(
        object_points, image_points, np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3),
        np.asarray(distortion, dtype=np.float64), flags=cv2.SOLVEPNP_SQPNP)
    if not ok or not np.isfinite(tvec).all() or not np.isfinite(rvec).all():
        raise ValueError('tag pose solution is not finite')
    projected, _ = cv2.projectPoints(
        object_points, rvec, tvec, np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3),
        np.asarray(distortion, dtype=np.float64))
    error = float(np.sqrt(np.mean(np.sum((projected.reshape(4, 2) - image_points) ** 2, axis=1))))
    return tvec.reshape(3), error


def camera_point_to_base(point_optical, translation_base, rotation_optical_to_base):
    rotation = np.asarray(rotation_optical_to_base, dtype=np.float64).reshape(3, 3)
    translation = np.asarray(translation_base, dtype=np.float64).reshape(3)
    point = np.asarray(point_optical, dtype=np.float64).reshape(3)
    if not all(np.isfinite(value) for value in rotation.ravel()):
        raise ValueError('camera rotation is not finite')
    if not all(np.isfinite(value) for value in translation) or not all(np.isfinite(value) for value in point):
        raise ValueError('camera translation and point must be finite')
    return rotation @ point + translation
