from launch import LaunchDescription, actions
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server_node',
            output='both',
            respawn=False
        ),
        actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'play', '/bags/scan_assembler_node.bag', '&&',
                 'ros2', 'run', 'octomap_server', 'octomap_saver_node', '--ros-args', '-p' 'octomap_path:=/$(pwd)/results/pcd.ot'],
            output='screen'
        ),
    ])
