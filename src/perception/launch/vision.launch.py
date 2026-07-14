"""Launch detector buoy dari topik ROS atau kamera USB V4L2.

Contoh:
    ros2 launch perception vision.launch.py
    ros2 launch perception vision.launch.py use_camera:=true camera_device:=/dev/video0
    ros2 launch perception vision.launch.py image_topic:=/camera/image_raw
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('perception'), 'config', 'vision_params.yaml')

    image_topic_arg = DeclareLaunchArgument(
        'image_topic', default_value='/camera/image_raw',
        description='Topic sensor_msgs/Image dari kamera.')
    obstacle_topic_arg = DeclareLaunchArgument(
        'obstacle_topic', default_value='/asv/obstacles',
        description='Topic hasil deteksi ObstacleArray.')
    use_camera_arg = DeclareLaunchArgument(
        'use_camera', default_value='false',
        description='Jalankan driver ros-jazzy-v4l2-camera untuk kamera USB.')
    camera_device_arg = DeclareLaunchArgument(
        'camera_device', default_value='/dev/video0',
        description='Device V4L2 yang diteruskan WSL, misalnya /dev/video0.')

    camera = Node(
        package='v4l2_camera', executable='v4l2_camera_node', name='usb_camera',
        namespace='camera', output='screen',
        condition=IfCondition(LaunchConfiguration('use_camera')),
        parameters=[{'video_device': LaunchConfiguration('camera_device')}])
    detector = Node(
        package='perception', executable='vision_detector_node',
        name='vision_detector_node', output='screen', parameters=[config],
        remappings=[
            ('camera/image_raw', LaunchConfiguration('image_topic')),
            ('obstacles', LaunchConfiguration('obstacle_topic')),
        ])

    return LaunchDescription([
        image_topic_arg, obstacle_topic_arg, use_camera_arg, camera_device_arg,
        camera, detector,
    ])
