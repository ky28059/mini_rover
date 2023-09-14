import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from geometry_msgs.msg import Twist

import pygame
from pygame.joystick import Joystick


class TwistPublisher(Node):
    DEADBAND = 0.05

    def __init__(self):
        super().__init__('twist_publisher')
        self.publisher_ = self.create_publisher(
            Twist,
            'cmd_vel',
            qos_profile_system_default
        )

        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.periodic)

        self.controller = Joystick(0)  # TODO: joystick #?

    def periodic(self):
        # Publish tank drive data. Left velocity is controlled via `linear.x` and
        # right velocity via `angular.z`.
        left_pow, right_pow = tank_drive(
            deadband(self.controller.get_axis(0), self.DEADBAND),  # TODO axis #
            deadband(self.controller.get_axis(1), self.DEADBAND)  # TODO axis #
        )

        msg = Twist()
        msg.linear.x = left_pow
        msg.angular.z = right_pow

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published l: {left_pow}, r: {right_pow}')


def tank_drive(translational: float, angular: float) -> tuple[float, float]:
    """
    Simple tank drive control scheme, where joystick 1 x-axis controls translation and joystick 2
    y-axis controls rotation. Input is returned as a tuple of [left, right] wheel velocities.

    :param translational: The forward / back power, in [-1.0, 1.0].
    :param angular: The left / right power, in [-1.0, 1.0].
    :return: The parsed left and right wheel speeds, in [-1.0, 1.0].
    """
    scale = min(1.0, abs(translational) + abs(angular))  # Scale net inputs > 1.0 down to 1.0

    return (translational - angular) / scale, \
           (translational + angular) / scale


def deadband(x: float, tolerance: float) -> float:
    """
    Squish an input to 0 within a given tolerance around 0.0, scaling the rest of the input accordingly.
    :param x: The input, in [-1.0, 1.0].
    :param tolerance: The radius around 0.0 to squash input in.
    :return: The input with deadband applied.
    """
    # y = 0 { -tolerance < x < tolerance }
    if -tolerance < x < tolerance:
        return 0.0

    # https://www.desmos.com/calculator/3b9pqz1ozn
    # y = (x - tolerance) / (1.0 - tolerance) { x > tolerance }
    # y = (x + tolerance) / (1.0 - tolerance) { x < -tolerance }
    return (x - math.copysign(tolerance, x)) * (1.0 - tolerance)


def main(args=None):
    pygame.init()
    rclpy.init(args=args)

    twist_publisher = TwistPublisher()
    rclpy.spin(twist_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    twist_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
