
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleController(Node):
    def __init__(self):
        super().__init__("turtlebot_controller")
        self.cmd_vel_publisher_ = self.create_publisher(
                Twist, "/cmd_vel", 10)
        self.get_logger().info("Publisher created")
        self.timer = self.create_timer(0.5,self.publish_cmd)

    def publish_cmd(self):
        cmd = Twist()
        cmd.linear.x = 0.2
        cmd.angular.z = 0.2
        self.cmd_vel_publisher_.publish(cmd)
        self.get_logger().info(f"Velocity Thing: {cmd.linear.x} , {cmd.angular.z}")

    
def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    rclpy.shutdown()

