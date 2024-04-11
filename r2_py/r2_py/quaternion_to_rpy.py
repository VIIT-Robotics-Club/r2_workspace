#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import os
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from math import atan2, asin, pi


class QuatToRPY(Node):
    def __init__(self):
        super().__init__("quat_to_rpy")

        self.subscription = self.create_subscription(
            Imu,
            "/imu/data",
            self.listener_callback,
            10
        )

        self.publisher = self.create_publisher(Vector3, "/rpy", 10)

    def listener_callback(self, msg):

        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w

        # Convert the quaternion to roll, pitch, and yaw
        roll = atan2(2*(w*x + y*z), 1 - 2*(x**2 + y**2))
        pitch = asin(2*(w*y - z*x))
        yaw = atan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2))

        # Convert to degrees
        roll = roll * 180.0 / pi
        pitch = pitch * 180.0 / pi
        yaw = yaw * 180.0 / pi

        # Create a Vector3 message
        rpy = Vector3()
        rpy.x = roll
        rpy.y = pitch
        rpy.z = yaw

        self.publisher.publish(rpy)
    

        
def main(args=None):
    rclpy.init(args=args)
    node = QuatToRPY()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
