from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    path_planning2_node = Node(
        package='om_custom',
        executable='path_planning2.py',
        name='path_planning2',
        output='screen',
    )

    return LaunchDescription([
        path_planning2_node,
    ])
