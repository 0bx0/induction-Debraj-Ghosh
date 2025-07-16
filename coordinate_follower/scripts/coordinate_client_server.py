#!/usr/bin/env python3

import rclpy
import os
from rclpy.node import Node
from rclpy.action import ActionClient

from coordinate_follower.action import NavigateToCoord

class CoordinateClient(Node):
    def __init__(self):
        super().__init__('coordinate_client')
        self._action_client = ActionClient(self, NavigateToCoord, 'navigate_to_coord')
        self.goal_index = 0

        script_dir = os.path.dirname(os.path.realpath(__file__))
        file_path = os.path.join(script_dir, 'coords.txt')
        self.coords = self.load_goals_from_file(file_path)

        self._action_client.wait_for_server()
        self.send_next_goal()

    def load_goals_from_file(self, filename):
        coords = []
        with open(filename, 'r') as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) == 2:
                    x, y = float(parts[0]), float(parts[1])
                    coords.append((x, y))
        return coords

    def send_next_goal(self):
        if self.goal_index >= len(self.coords):
            self.get_logger().info("Completed all coordinate goals.")
            return

        x, y = self.coords[self.goal_index]
        goal_msg = NavigateToCoord.Goal()
        goal_msg.x_coordinates = x          
        goal_msg.y_coordinates = y        

        self.get_logger().info(f"Sending goal: ({x}, {y})")

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('Goal succeeded.')
        else:
            self.get_logger().warn('Goal failed.')


        self.goal_index += 1
        self.send_next_goal()


def main(args=None):
    rclpy.init(args=args)
    node = CoordinateClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
