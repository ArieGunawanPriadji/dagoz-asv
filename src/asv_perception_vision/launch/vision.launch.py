"""
Jalankan node vision detector di bawah namespace 'dagozilla'.
Topic yang dihasilkan otomatis jadi:
    /dagozilla/camera/image_raw   (input, subscribe)
    /dagozilla/obstacles          (output, publish)

Jalankan dengan:
    ros2 launch asv_perception_vision vision.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('asv_perception_vision'),
        'config', 'vision_params.yaml')

    return LaunchDescription([
        Node(
            package='asv_perception_vision',
            executable='vision_detector_node',
            name='vision_detector_node',
            namespace='dagozilla',
            output='screen',
            parameters=[config],
            remappings=[
                ('camera/image_raw', 'camera/image_raw'),
                ('obstacles', 'obstacles'),
            ],
        )
    ])
