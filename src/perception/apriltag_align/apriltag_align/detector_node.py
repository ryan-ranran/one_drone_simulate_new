import math
from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from apriltag_align.geometry import camera_point_to_base, solve_tag_pose


class AprilTagAlignNode(Node):
    def __init__(self):
        super().__init__('apriltag_align')
        defaults = dict(
            image_topic='/uav1/d435i/down/image_raw',
            camera_info_topic='/uav1/d435i/down/camera_info',
            output_topic='/uav1/apriltag/landing_target',
            status_topic='/uav1/apriltag/alignment_status',
            tag_id=0, tag_size=.4, min_detections=5,
            max_reprojection_error=1.5, max_camera_info_age=2.,
            min_tag_area_px=250., max_tag_jump=.35,
            min_tag_height=.3, max_tag_height=5.,
            camera_translation=[0., 0., -.07],
            camera_rotation_optical_to_base=[0., -1., 0., -1., 0., 0., 0., 0., -1.])
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value
        self.tag_id = int(p('tag_id'))
        self.tag_size = float(p('tag_size'))
        self.min_detections = int(p('min_detections'))
        self.max_error = float(p('max_reprojection_error'))
        self.max_info_age = float(p('max_camera_info_age'))
        self.min_tag_area = float(p('min_tag_area_px'))
        self.max_jump = float(p('max_tag_jump'))
        self.min_height = float(p('min_tag_height'))
        self.max_height = float(p('max_tag_height'))
        self.translation = np.asarray(p('camera_translation'), dtype=float).reshape(3)
        self.rotation = np.asarray(p('camera_rotation_optical_to_base'), dtype=float).reshape(3, 3)
        if self.tag_id < 0 or self.tag_size <= 0 or self.min_detections <= 0:
            raise ValueError('tag_id, tag_size and min_detections are invalid')
        if (self.max_error <= 0 or self.max_info_age <= 0 or self.min_tag_area <= 0 or
                self.max_jump <= 0 or not 0 < self.min_height < self.max_height):
            raise ValueError('tag quality and range limits are invalid')
        orthogonality = self.rotation.T @ self.rotation
        if not np.allclose(orthogonality, np.eye(3), atol=1e-6):
            raise ValueError('camera_rotation_optical_to_base must be orthonormal')
        if not math.isclose(float(np.linalg.det(self.rotation)), 1., abs_tol=1e-6):
            raise ValueError('camera_rotation_optical_to_base must be a proper rotation')
        self.bridge = CvBridge()
        self.camera_matrix = self.distortion = None
        self.camera_size = None
        self.camera_info_at = -math.inf
        self.consecutive = 0
        self.samples = deque(maxlen=self.min_detections)
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        self.detector_parameters = cv2.aruco.DetectorParameters_create()
        self.target_pub = self.create_publisher(PoseStamped, str(p('output_topic')), 1)
        self.status_pub = self.create_publisher(String, str(p('status_topic')), 10)
        self.create_subscription(CameraInfo, str(p('camera_info_topic')),
                                 self.on_camera_info, qos_profile_sensor_data)
        self.create_subscription(Image, str(p('image_topic')),
                                 self.on_image, qos_profile_sensor_data)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def report(self, value):
        self.status_pub.publish(String(data=value))

    def on_camera_info(self, msg):
        matrix = np.asarray(msg.k, dtype=float).reshape(3, 3)
        distortion = np.asarray(msg.d, dtype=float).reshape(-1)
        if (msg.width <= 0 or msg.height <= 0 or not np.isfinite(matrix).all() or
                not np.isfinite(distortion).all()):
            return
        self.camera_matrix = matrix
        self.distortion = distortion
        self.camera_size = (msg.width, msg.height)
        self.camera_info_at = self.now()

    def on_image(self, msg):
        if self.camera_matrix is None or self.now() - self.camera_info_at > self.max_info_age:
            self.consecutive = 0
            self.samples.clear()
            self.report('WAIT_CAMERA_INFO')
            return
        if self.camera_size != (msg.width, msg.height):
            self.consecutive = 0
            self.samples.clear()
            self.report('IMAGE_INFO_MISMATCH')
            return
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except CvBridgeError as exc:
            self.consecutive = 0
            self.samples.clear()
            self.get_logger().warn('Image conversion failed: ' + str(exc), throttle_duration_sec=2.)
            self.report('BAD_IMAGE')
            return
        corners, ids, _ = cv2.aruco.detectMarkers(image, self.dictionary,
                                                   parameters=self.detector_parameters)
        if ids is None:
            self.consecutive = 0
            self.samples.clear()
            self.report('TAG_NOT_VISIBLE')
            return
        matches = [corner for corner, detected_id in zip(corners, ids.ravel())
                   if int(detected_id) == self.tag_id]
        if len(matches) != 1:
            self.consecutive = 0
            self.samples.clear()
            self.report('TAG_NOT_UNIQUE' if len(matches) > 1 else 'WRONG_TAG_ID')
            return
        refined = cv2.cornerSubPix(
            image, matches[0].reshape(4, 2).astype(np.float32), (3, 3), (-1, -1),
            (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 20, .01))
        tag_area = abs(float(cv2.contourArea(refined.reshape(4, 1, 2))))
        if tag_area < self.min_tag_area:
            self.consecutive = 0
            self.samples.clear()
            self.report('TAG_TOO_SMALL')
            return
        try:
            point_camera, error = solve_tag_pose(
                refined, self.camera_matrix, self.distortion, self.tag_size)
            point_base = camera_point_to_base(point_camera, self.translation, self.rotation)
        except (ValueError, cv2.error) as exc:
            self.consecutive = 0
            self.samples.clear()
            self.report('POSE_INVALID')
            self.get_logger().warn('Tag pose rejected: ' + str(exc), throttle_duration_sec=2.)
            return
        height = -float(point_base[2])
        if (not math.isfinite(error) or error > self.max_error or
                not self.min_height <= height <= self.max_height):
            self.consecutive = 0
            self.samples.clear()
            self.report('POSE_REJECTED')
            return
        if self.samples and np.linalg.norm(point_base[:2] - self.samples[-1][:2]) > self.max_jump:
            self.consecutive = 0
            self.samples.clear()
            self.report('POSE_JUMP')
            return
        self.samples.append(point_base)
        self.consecutive += 1
        if self.consecutive < self.min_detections:
            self.report('ACCUMULATING')
            return
        filtered = np.median(np.asarray(self.samples), axis=0)
        target = PoseStamped()
        target.header = msg.header
        target.header.frame_id = 'base_link'
        target.pose.position.x = float(filtered[0])
        target.pose.position.y = float(filtered[1])
        target.pose.position.z = float(filtered[2])
        target.pose.orientation.w = 1.0
        self.target_pub.publish(target)
        self.report(f'TRACKING error={error:.3f}px')


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagAlignNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
