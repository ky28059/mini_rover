from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launches both the `joy` and twist publisher nodes to send processed controller
    input to `cmd_vel`.
    :return: The `LaunchDescription`.
    """

    joy_node = Node(
        package='joy',
        executable='joy_node',
        emulate_tty=True,
        output='screen'
    )

    twist_publisher_node = Node(
        package='mini_rover',
        executable='twist_publisher_middleman',
        emulate_tty=True,
        output='screen'
    )

    return LaunchDescription([
        joy_node,
        twist_publisher_node
    ])
