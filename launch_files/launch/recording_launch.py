
from launch import LaunchDescription, actions
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        actions.ExecuteProcess(
            # TODO: change this to be parametrized
            cmd=['ros2', 'bag', 'record', '-a', '-o', '/bags/scan_assembler_node.bag'],
            output='screen'
        ),
    ])
