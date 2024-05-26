#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import os
from math import sqrt
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from math import atan2, asin, pi


class QuatToRPY(Node):
    def __init__(self):
        super().__init__("quat_to_rpy")
        self.get_logger().info("Quat to RPY node has been started")

        self.subscription = self.create_subscription(
            Imu,
            "/imu/data",
            self.listener_callback,
            10
        )

        self.declare_parameter("logging", False)

        self.publisher = self.create_publisher(Vector3, "/rpy", 10)
    
        self.create_timer(0.1, self.time_callback)
        

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0


    def listener_callback(self, msg):

        if self.get_parameter("logging").value:
            self.get_logger().info("Received a message")
            
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w
        
        if self.get_parameter("logging").value:
            self.get_logger().info(f"x: {x}, y: {y}, z: {z}, w: {w}")

        # roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = atan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = sqrt(1 + 2 * (w * y - z * x))
        cosp = sqrt(1 - 2 * (w * y - x * z))
        pitch = 2 * atan2(sinp, cosp) - pi/2

        # yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = atan2(siny_cosp, cosy_cosp)


        # Convert to degrees
        self.roll = roll * 180.0 / pi
        self.pitch = pitch * 180.0 / pi
        self.yaw = yaw * 180.0 / pi
        
        
        if self.get_parameter("logging").value:
            self.get_logger().info(f"Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")

        

    def time_callback(self):
        
        if self.get_parameter("logging").value:
            self.get_logger().info("Time callback called")
        
        # Create a Vector3 message
        rpy = Vector3()
        rpy.x = self.roll
        rpy.y = self.pitch
        rpy.z = self.yaw
        
        self.publisher.publish(rpy)
    

        
def main(args=None):
    rclpy.init(args=args)
    node = QuatToRPY()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
