import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

from mini_rover.minirover_twist_publisher import tank_drive


class TwistPublisherMiddleman(Node):

    def __init__(self):
        super().__init__('twist_publisher_middleman')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.controller_callback,
            qos_profile_sensor_data
        )
        self.publisher_ = self.create_publisher(
            Twist,
            'cmd_vel',
            qos_profile_system_default
        )

    def controller_callback(self, message: Joy):
        # Publish tank drive data. Left velocity is controlled via `linear.x` and
        # right velocity via `angular.z`.
        left_pow, right_pow = tank_drive(message.axes[1], message.axes[2])

        msg = Twist()
        msg.linear.x = left_pow
        msg.angular.z = right_pow

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published l: {left_pow}, r: {right_pow}')


def main(args=None):
    rclpy.init(args=args)

    twist_publisher = TwistPublisherMiddleman()
    rclpy.spin(twist_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    twist_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
