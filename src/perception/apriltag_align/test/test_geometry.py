import math

import numpy as np

from apriltag_align.geometry import camera_point_to_base, solve_tag_pose


def test_down_camera_rotation_projects_center_to_body_z():
    rotation = [0., -1., 0., -1., 0., 0., 0., 0., -1.]
    value = camera_point_to_base([0., 0., 1.], [0., 0., -.07], rotation)
    assert np.allclose(value, [0., 0., -1.07])


def test_synthetic_tag_pose_recovers_camera_position():
    size = 0.4
    half = size / 2.
    object_points = np.array([
        [-half, -half, 0.], [half, -half, 0.],
        [half, half, 0.], [-half, half, 0.],
    ])
    camera = np.array([[400., 0., 320.], [0., 400., 240.], [0., 0., 1.]])
    expected = np.array([.05, -.03, 1.5])
    projected, _ = __import__('cv2').projectPoints(object_points, np.zeros(3), expected,
                                                  camera, np.zeros(5))
    actual, error = solve_tag_pose(projected, camera, np.zeros(5), size)
    assert np.allclose(actual, expected, atol=1e-6)
    assert math.isfinite(error) and error < 1e-6
