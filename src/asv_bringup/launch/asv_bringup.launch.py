"""
Launch file utama: menjalankan MAVROS (koneksi ke Pixhawk) + seluruh node ASV.
Jalankan dengan:
    ros2 launch asv_bringup asv_bringup.launch.py fcu_url:=/dev/ttyACM0:115200
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url', default_value='/dev/ttyACM0:115200',
        description='Port serial/UDP koneksi ke Pixhawk')

    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mavros'),
                         'launch', 'apm.launch')
        ),
        launch_arguments={'fcu_url': LaunchConfiguration('fcu_url')}.items()
    )

    nodes = [
        Node(package='asv_mavros_bridge', executable='mavros_bridge_node', output='screen'),
        Node(package='asv_guidance_control', executable='waypoint_follower_node', output='screen'),
        Node(package='asv_mission', executable='mission_manager_node', output='screen'),
        Node(package='asv_perception', executable='obstacle_detector_node', output='screen'),
    ]

    return LaunchDescription([fcu_url_arg, mavros_launch] + nodes)
