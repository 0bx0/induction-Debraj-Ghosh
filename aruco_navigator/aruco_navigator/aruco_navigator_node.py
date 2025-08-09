import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

from cv_bridge import CvBridge
import cv2
import numpy as np


class ArucoNavigator(Node):
    def __init__(self):
        super().__init__('aruco_navigator')

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        # Use correct DetectorParameters creation method for newer OpenCV
        self.parameters = cv2.aruco.DetectorParameters()

        self.tag_count = 0
        self.turning = False
        self.turn_direction = None
        self.turn_angle_turned = 0.0
        self.turn_speed = 0.5
        self.last_time = None  # Will be set on first turn update

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.parameters
        )

        if ids is not None:
            self.get_logger().info("Fetched id")
            ids = ids.flatten()
            for marker_id in ids:
                if marker_id in [0, 1] and not self.turning:
                    self.get_logger().info(f"Tag detected: {marker_id}")
                    self.turn_direction = 'right' if marker_id == 0 else 'left'
                    self.turning = True
                    self.turn_angle_turned = 0.0
                    self.tag_count += 1
                    self.last_time = self.get_clock().now()  # reset timer
                    break

        if self.turning:
            twist_msg = Twist()
            angular_speed = self.turn_speed
            angular_distance = np.pi / 2

            twist_msg.angular.z = -angular_speed if self.turn_direction == 'right' else angular_speed
            self.cmd_pub.publish(twist_msg)

            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1e9
            self.last_time = current_time
            self.turn_angle_turned += angular_speed * dt

            if self.turn_angle_turned >= angular_distance - 0.01:
                self.turning = False
                self.get_logger().info("Turn completed")

        else:
            twist_msg = Twist()
            if self.tag_count < 5:
                twist_msg.linear.x = 0.1
            self.cmd_pub.publish(twist_msg)
            if self.tag_count >= 5:
                self.get_logger().info("Completed")


def main(args=None):
    rclpy.init(args=args)
    node = ArucoNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

