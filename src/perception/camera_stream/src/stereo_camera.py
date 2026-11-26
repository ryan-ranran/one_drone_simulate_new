"""Publish the two images carried by a side-by-side UVC stereo camera."""
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class StereoCamera(Node):
    def __init__(self):
        super().__init__('stereo_camera')
        self.declare_parameter(
            'device',
            '/dev/v4l/by-id/usb-USB_Camera_USB_Camera_01.00.00-video-index0')
        self.declare_parameter('width', 2560)
        self.declare_parameter('height', 800)
        self.declare_parameter('fps', 60.0)
        self.declare_parameter('left_topic', '/camera/ov9782/left/image_raw')
        self.declare_parameter('right_topic', '/camera/ov9782/right/image_raw')
        self.declare_parameter('combined_topic', '/camera/ov9782/combined/image_raw')
        self.declare_parameter('publish_combined', False)
        self.declare_parameter('frame_id', 'ov9782_camera')

        device = self.get_parameter('device').value
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = float(self.get_parameter('fps').value)
        self.frame_id = self.get_parameter('frame_id').value

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publishers_ = {
            'left': self.create_publisher(
                Image, self.get_parameter('left_topic').value, qos),
            'right': self.create_publisher(
                Image, self.get_parameter('right_topic').value, qos),
        }
        self.publish_combined = self.get_parameter('publish_combined').value
        if self.publish_combined:
            self.publishers_['combined'] = self.create_publisher(
                Image, self.get_parameter('combined_topic').value, qos)
        self.capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self.capture.isOpened():
            raise RuntimeError(f'cannot open stereo camera {device}; '
                               'check the device path and stop any other user')
        self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.capture.set(cv2.CAP_PROP_FPS, self.fps)
        actual_width = int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if actual_width != self.width or actual_height != self.height:
            self.capture.release()
            raise RuntimeError(
                f'camera did not accept {self.width}x{self.height}; '
                f'got {actual_width}x{actual_height}')

        self.get_logger().info(
            f'publishing {device} at {self.width}x{self.height}, '
            f'{self.fps:g} fps as OV9782 left/right topics')
        self.timer = self.create_timer(1.0 / self.fps, self.publish_frame)

    @staticmethod
    def image_message(frame, stamp, frame_id):
        message = Image()
        message.header.stamp = stamp
        message.header.frame_id = frame_id
        message.height, message.width = frame.shape[:2]
        message.encoding = 'bgr8'
        message.is_bigendian = False
        message.step = message.width * 3
        message.data = frame.tobytes()
        return message

    def publish_frame(self):
        ok, combined = self.capture.read()
        if not ok or combined is None:
            self.get_logger().warning('failed to read a camera frame', throttle_duration_sec=2.0)
            return
        if combined.ndim != 3 or combined.shape[2] != 3:
            self.get_logger().error(
                f'unexpected frame shape {combined.shape}; expected a BGR image',
                throttle_duration_sec=2.0)
            return
        if combined.shape[1] % 2:
            self.get_logger().error(
                f'combined frame width {combined.shape[1]} is not divisible by two',
                throttle_duration_sec=2.0)
            return

        midpoint = combined.shape[1] // 2
        left = combined[:, :midpoint]
        right = combined[:, midpoint:]
        stamp = self.get_clock().now().to_msg()
        self.publishers_['left'].publish(
            self.image_message(left, stamp, self.frame_id + '_left'))
        self.publishers_['right'].publish(
            self.image_message(right, stamp, self.frame_id + '_right'))
        if self.publish_combined:
            self.publishers_['combined'].publish(
                self.image_message(combined, stamp, self.frame_id))

    def destroy_node(self):
        self.capture.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StereoCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
