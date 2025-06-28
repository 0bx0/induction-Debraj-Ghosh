#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Publisher created")

        self.laser_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.obstacle_checker_callback,
            10
        )
        self.get_logger().info("Subscriber created")

    def obstacle_checker_callback(self, msg):
        front_ranges = msg.ranges[0:15] + msg.ranges[-15:]

        valid_ranges = [r for r in front_ranges if math.isfinite(r)]

        if valid_ranges:
            min_front_distance = min(valid_ranges)
        else:
            min_front_distance = msg.range_max

        velocity = Twist()

        if min_front_distance > 0.5:
            velocity.linear.x = 0.2
            velocity.angular.z = 0.0
        else:
            velocity.linear.x = 0.0
            velocity.angular.z = -0.5

        self.cmd_vel_publisher.publish(velocity)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

