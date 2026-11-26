from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'device',
            default_value=(
                '/dev/v4l/by-id/'
                'usb-USB_Camera_USB_Camera_01.00.00-video-index0')),
        DeclareLaunchArgument('width', default_value='2560'),
        DeclareLaunchArgument('height', default_value='800'),
        DeclareLaunchArgument('fps', default_value='15.0'),
        DeclareLaunchArgument('publish_combined', default_value='false'),
        Node(
            package='camera_stream',
            executable='stereo_camera',
            name='ov9782_camera',
            output='screen',
            parameters=[{
                'device': LaunchConfiguration('device'),
                'width': LaunchConfiguration('width'),
                'height': LaunchConfiguration('height'),
                'fps': LaunchConfiguration('fps'),
                'left_topic': '/camera/ov9782/left/image_raw',
                'right_topic': '/camera/ov9782/right/image_raw',
                'combined_topic': '/camera/ov9782/combined/image_raw',
                'publish_combined': LaunchConfiguration('publish_combined'),
                'frame_id': 'ov9782_camera',
            }],
        ),
    ])
