import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class NameSubscriber(Node):

    def __init__(self):
        super().__init__('name_subscriber')
        self.subscription = self.create_subscription(
            String,
            'name',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    name_subscriber = NameSubscriber()
    rclpy.spin(name_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    name_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
