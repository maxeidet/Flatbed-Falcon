from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    nodes = ['truck_driver', 'target_tracker', 'landing_controller', 'logger']
    return LaunchDescription([
        Node(package='flatbed_falcon', executable=name, output='screen')
        for name in nodes
    ])
