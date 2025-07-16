#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import math
import time
import threading

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from coordinate_follower.action import NavigateToCoord


class CoordinateActionServer(Node):
    def __init__(self):
        super().__init__('coordinate_action_server')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        self.cb_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            NavigateToCoord,
            'navigate_to_coord',
            self.execute_callback,
            callback_group=self.cb_group
        )

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.get_logger().info("Coordinate Action Server started and ready.")

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        orientation = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

        self.get_logger().debug(
            f"Odometry updated: x={self.current_x:.3f}, y={self.current_y:.3f}, yaw={self.current_yaw:.3f}")

    def execute_callback(self, goal_handle):
        target_x = goal_handle.request.x_coordinates
        target_y = goal_handle.request.y_coordinates
        self.get_logger().info(f"Received new goal: x={target_x}, y={target_y}")

        result = NavigateToCoord.Result()
        goal_reached = threading.Event()

        def movement_loop():
            self.get_logger().info("Movement thread started.")
            try:
                while rclpy.ok() and not goal_reached.is_set():
                    if goal_handle.is_cancel_requested:
                        self.get_logger().info("Goal cancellation requested.")
                        goal_handle.canceled()
                        result.success = False
                        goal_reached.set()
                        self.get_logger().info("Goal canceled, stopping movement.")
                        return

                    dx = target_x - self.current_x
                    dy = target_y - self.current_y
                    distance = math.sqrt(dx ** 2 + dy ** 2)
                    self.get_logger().info(f"Distance to target: {distance:.2f} meters.")

                    if distance < 0.3:
                        self.get_logger().info("Target reached successfully.")
                        self.cmd_vel_pub.publish(Twist())  # Stop robot
                        goal_handle.succeed()
                        result.success = True
                        goal_reached.set()
                        return

                    target_angle = math.atan2(dy, dx)
                    angle_diff = target_angle - self.current_yaw

                    # Normalize angle_diff to [-pi, pi]
                    while angle_diff > math.pi:
                        angle_diff -= 2.0 * math.pi
                    while angle_diff < -math.pi:
                        angle_diff += 2.0 * math.pi

                    cmd_vel = Twist()

                    if abs(angle_diff) > 0.3:
                        self.get_logger().info(f"Turning in place. Angle difference: {angle_diff:.2f} rad.")
                        cmd_vel.angular.z = 0.2 if angle_diff > 0 else -0.2
                        cmd_vel.linear.x = 0.0
                    else:
                        self.get_logger().info("Moving forward with slight heading correction.")
                        cmd_vel.linear.x = 0.05
                        cmd_vel.angular.z = 0.1 * angle_diff

                    self.cmd_vel_pub.publish(cmd_vel)
                    time.sleep(0.3)
            except Exception as e:
                self.get_logger().error(f"Exception in movement thread: {e}")
                goal_handle.abort()
                result.success = False
                goal_reached.set()

            self.get_logger().info("Movement thread finished.")

        movement_thread = threading.Thread(target=movement_loop)
        movement_thread.start()

        # Just wait for movement thread to complete — do NOT spin here!
        movement_thread.join()

        self.get_logger().info("Goal execution completed.")
        return result


def main(args=None):
    rclpy.init(args=args)
    node = CoordinateActionServer()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
