#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3, Twist
import math
import numpy as np
import time
import sys

class RotateAndMoveNode(Node):
    def __init__(self):
        super().__init__('rotate_and_move_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('logging', True),              # Enable/disable logging
                ('linear_speed', 0.7),          # Linear speed for forward movement
                ('move_duration', 3.0),         # Duration of forward movement in seconds
                ('angular_kp', 4.1),            # Proportional gain for angular PID controller
                ('angular_ki', 0.05),           # Integral gain for angular PID controller
                ('angular_kd', 0.00),          # Derivative gain for angular PID controller
                ('max_angular_speed', 4.0),     # Maximum angular speed for rotation
                ('max_integral', 10.0),         # Maximum integral term value for angular PID controller
                ('rotation_angle', 180.0),       # Angle of rotation in degrees
                ('yaw_error_threshold', 0.1)   # Threshold for considering rotation completed (in radians
            ]
        )

        #Retrieve parameters
        self.logging = self.get_parameter('logging').get_parameter_value().bool_value
        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        self.move_duration = self.get_parameter('move_duration').get_parameter_value().double_value
        self.angular_kp = self.get_parameter('angular_kp').get_parameter_value().double_value
        self.angular_ki = self.get_parameter('angular_ki').get_parameter_value().double_value
        self.angular_kd = self.get_parameter('angular_kd').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        self.max_integral = self.get_parameter('max_integral').get_parameter_value().double_value
        self.rotation_angle = self.get_parameter('rotation_angle').get_parameter_value().double_value
        self.yaw_error_threshold = self.get_parameter('yaw_error_threshold').get_parameter_value().double_value

        self.current_yaw = 0.0
        self.target_yaw = None
        self.rotation_completed = False
        self.moving_forward = False

        self.angular_error_sum = 0.0
        self.angular_last_error = 0.0

        self.subscription = self.create_subscription(Vector3, 'rpy', self.rpy_callback, 10)
        self.publisher = self.create_publisher(Twist, 'nav_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Convert rotation angle from degrees to radians
        self.rotation_angle_rad = self.rotation_angle * (math.pi / 180)  

        self.start_time = None

    def rpy_callback(self, msg):

        # Update current yaw in radians 
        self.current_yaw = msg.z * (math.pi / 180)  # Convert degrees to radians
        self.get_logger().info(f'Current Yaw (degrees): {msg.z}')


        # If target yaw is not set, set it
        if self.target_yaw is None:

            # Set target yaw to current yaw plus rotation angle
            self.target_yaw = self.normalize_angle(self.current_yaw + self.rotation_angle_rad)  # Add 180 degrees in radians
            self.get_logger().info(f'Target Yaw (radians) Normalized: {self.target_yaw}')

        if self.logging:
            self.get_logger().info(f'Current Yaw (radians): {self.current_yaw}')
            self.get_logger().info(f'Target Yaw (radians): {self.target_yaw}')

    def PID_controller(self, error, error_sum, last_error, kp, ki, kd):
        '''PID controller for angular velocity control.'''
        P = kp * error
        error_sum += error
        error_sum = np.clip(error_sum, -self.max_integral, self.max_integral)   # Clip the integral term
        I = ki * error_sum
        D = kd * (error - last_error)
        control = P + I + D
        return control, error_sum

    def timer_callback(self):
        '''Timer callback for controlling robot rotation and movement.'''

        if self.target_yaw is None: # If target yaw is not set
            return


        # If rotation is not completed
        if not self.rotation_completed:

            # Calculate the yaw error
            yaw_error = self.normalize_angle(self.target_yaw - self.current_yaw)

            # Calculate angular velocity using PID controller
            angular_z, self.angular_error_sum = self.PID_controller(yaw_error, self.angular_error_sum, self.angular_last_error,
                                                                    self.angular_kp, self.angular_ki, self.angular_kd)      
            

            self.angular_last_error = yaw_error     # Update last error

            # Clip angular velocity to maximum angular speed
            angular_z = np.clip(angular_z, -self.max_angular_speed, self.max_angular_speed)


            # Publish angular velocity
            twist_msg = Twist()
            twist_msg.angular.z = angular_z
            self.publisher.publish(twist_msg)   


            # Check if rotation is completed
            if abs(yaw_error) < self.yaw_error_threshold:  

                self.rotation_completed = True  # Set rotation completed flag to True

                self.publisher.publish(Twist())  # Stop rotation

                if self.logging:
                    self.get_logger().info('Rotation completed. Moving forward.')


        # If rotation is completed and forward movement is not started
        elif not self.moving_forward:

            twist_msg = Twist()
            twist_msg.linear.x = self.linear_speed  # Set linear speed for forward movement

            self.publisher.publish(twist_msg)       # Publish forward movement command
            self.get_logger().info('Moving forward.')


            # Move forward for move_duration seconds 
            time.sleep(self.move_duration)
            self.moving_forward = True  # Set moving forward flag to True
            self.get_logger().info('Sleeping for 5 seconds.')

            # Linear speed for stopping executed after {move_duration} seconds
            twist_msg2 = Twist()
            twist_msg2.linear.x = 0.0

            self.publisher.publish(twist_msg2)  # Stop moving forward
            self.get_logger().info('Movement completed. Stopping.')

            # Exit the node
            sys.exit()
            

    def normalize_angle(self, angle):
        """Normalize the angle to the range [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    rotate_and_move_node = RotateAndMoveNode()
    rclpy.spin(rotate_and_move_node)
    rotate_and_move_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
