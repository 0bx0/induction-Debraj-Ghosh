#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_velocity)
        self.get_logger().info("TurtleBot controller started")

    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = 0.2       # Forward motion (m/s)
        msg.angular.z = 0.5      # Rotational speed (rad/s)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: linear={msg.linear.x}, angular={msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    rclpy.shutdown()

