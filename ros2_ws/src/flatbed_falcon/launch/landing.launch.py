"""Starts the Gazebo bridge and every Flatbed Falcon node, all on sim time.

    ros2 launch flatbed_falcon landing.launch.py [params:=/path/to/other.yaml]
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('flatbed_falcon')
    params = LaunchConfiguration('params')
    sim_time = {'use_sim_time': True}
    nodes = [
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='truck_bridge', output='screen',
             parameters=[{'config_file': os.path.join(share, 'config', 'bridge.yaml')}, sim_time]),
    ] + [
        Node(package='flatbed_falcon', executable=name, output='screen', parameters=[params, sim_time])
        for name in ('truck_driver', 'target_tracker', 'landing_controller', 'logger')
    ]
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value=os.path.join(share, 'config', 'falcon.yaml')),
        *nodes,
    ])
