from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    # Start action servers first
    move_server = Node(
        package='om_custom',
        executable='move_turtlebot_action_server.py',
        name='move_turtlebot_action_server',
        output='screen',
    )

    rotate_server = Node(
        package='om_custom',
        executable='rotate_turtlebot_action_server.py',
        name='rotate_turtlebot_action_server',
        output='screen',
    )

    # Start the path planning node after a short delay so servers are ready
    path_planning_node = Node(
        package='om_custom',
        executable='path_planning.py',
        name='path_planning',
        output='screen',
    )

    return LaunchDescription([
        move_server,
        rotate_server,
        path_planning_node,
    ])
