from launch import LaunchDescription
from launch_ros.actions import Node


# Simultaneously launches the `joy` and twist publisher nodes to receive, process, and send
# controller input to the rover.
def generate_launch_description():

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
