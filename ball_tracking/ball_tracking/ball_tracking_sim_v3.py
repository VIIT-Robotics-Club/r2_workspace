#!/usr/bin/env python3

from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
import rclpy

import cv_bridge
import cv2
from time import time
import os
import numpy as np

from ultralytics import YOLO
from ultralytics import utils

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

from r2_interfaces.msg import YoloResults
from r2_interfaces.msg import Xywh
from r2_interfaces.msg import XyXy

import sys


'''

For error in angular z tune the PID such that angular alignment is slow (that is the robot does not rotate too fast), it should not align perfectily to -90 degrees, it should be slow and smooth.
Because if the ball is on edges the robot facing -90 degrees will not be able to see the ball, so it should rotate slowly to see the ball.


'''

class BallTrackingNode(Node):
    def __init__(self):
        super().__init__('ball_tracking_node')
        self.get_logger().info("Ball Tracking Node has been started")
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('desired_contour_area', 50000),
                ('linearX_kp', 0.00001),
                ('linearX_ki', 0.00),
                ('linearX_kd', 0.00),
                ('linearY_kp', 0.005),
                ('linearY_ki', 0.00),
                ('linearY_kd', 0.00),
                ('angular_kp', 0.2),
                ('angular_ki', 0.00),
                ('angular_kd', 0.00),
                ('max_linear_speed', 2.0),
                ('max_angular_speed', 4.0),
                ('max_integral', 10.0),
                ('contour_area_threshold', 3000),
                ('difference_threshold', 10)
            ]
        )
        
        
        self.linearX_kp = self.get_parameter('linearX_kp').value
        self.linearX_ki = self.get_parameter('linearX_ki').value
        self.linearX_kd = self.get_parameter('linearX_kd').value
        
        self.linearY_kp = self.get_parameter('linearY_kp').value
        self.linearY_ki = self.get_parameter('linearY_ki').value
        self.linearY_kd = self.get_parameter('linearY_kd').value      
        
        self.angular_kp = self.get_parameter('angular_kp').value
        self.angular_ki = self.get_parameter('angular_ki').value
        self.angular_kd = self.get_parameter('angular_kd').value
        
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        
        self.max_integral = self.get_parameter('max_integral').value
        
        self.desired_contour_area = self.get_parameter('desired_contour_area').value
        self.contour_area_threshold = self.get_parameter('contour_area_threshold').value
        self.difference_threshold = self.get_parameter('difference_threshold').value
        
        self.add_on_set_parameters_callback(self.param_callback)
        
        self.linearX_error_sum = 0.0
        self.linearX_last_error = 0.0
        
        self.linearY_error_sum = 0.0
        self.linearY_last_error = 0.0
        
        self.angular_error_sum = 0.0
        self.angular_last_error = 0.0
        
        self.tracking_blue_ball = False
        
        self.difference_error = 0.0
        self.contour_area_error = 0.0

        self.create_subscription(YoloResults, 'yolo_results', self.yolo_results_callback, 10)
        
        self.create_subscription(
            Vector3,
            '/rpy',
            self.rpy_callback,
            10
        )
               
        
        self.cmd_vel_pub = self.create_publisher(Twist, 'nav_vel', 10)
        
        
        self.class_ids_list = []
        self.contour_areas_list = []
        self.differences_list = []
        self.confidences_list = []  
        self.tracking_ids_list = []
        self.xyxys_list = []
        self.xywhs_list = []
        
        self.closest_blue_ball = {
            'class_id': None,
            'contour_area': None,
            'difference': None,
            'confidence': None,
            'tracking_id': None,
            'xyxy': None,
            'xywh': None
        }
        
        self.last_seen_direction = None
        
        self.yaw = 0.0      # Degrees
        self.target_yaw = -90.0
        self.target_yaw_in_rad = self.target_yaw * (np.pi / 180)
        self.yaw_in_rad = 0.0
        self.yaw_error = 0.0
        
        
        # Debug: detect and print parameter changes
    #     self.create_timer(1, self.time_callback)
    
    
    
    # def time_callback(self):
    #     # Print all parameter values
    #     print("Parameter Values:")
    #     print(f"desired_contour_area: {self.desired_contour_area}")
    #     print(f"linear_kp: {self.linearX_kp}")
    #     print(f"linear_ki: {self.linearX_ki}")
    #     print(f"linear_kd: {self.linearX_kd}")
    #     print(f"angular_kp: {self.angular_kp}")
    #     print(f"angular_ki: {self.angular_ki}")
    #     print(f"angular_kd: {self.angular_kd}")
    #     print(f"max_linear_speed: {self.max_linear_speed}")
    #     print(f"max_angular_speed: {self.max_angular_speed}")
    #     print(f"max_integral: {self.max_integral}")
    #     print(f"contour_area_threshold: {self.contour_area_threshold}")
    #     print(f"difference_threshold: {self.difference_threshold}")
        
        
        
    def param_callback(self, params: list[Parameter]):
        
        for param in params:
            param_name = param.name
            param_value = param.value
            setattr(self, param_name, param_value)
        
        return SetParametersResult(successful=True)     
    
    def rpy_callback(self, msg):
        self.yaw = msg.z
        self.yaw_in_rad = self.yaw * (np.pi / 180)
        self.yaw_error = self.target_yaw_in_rad - self.yaw_in_rad
        
        self.get_logger().warn(f"Yaw: {self.yaw}, Yaw Error: {self.yaw_error}")
        
    
    def yolo_results_callback(self, msg):
        self.get_logger().info("Yolo Results Received")
        
        self.class_ids_list.clear()
        self.contour_areas_list.clear()
        self.differences_list.clear()
        self.confidences_list.clear()
        self.tracking_ids_list.clear()
        self.xyxys_list.clear()
        self.xywhs_list.clear()
        
        self.class_ids_list.extend(msg.class_ids)
        self.contour_areas_list.extend(msg.contour_area)
        self.differences_list.extend(msg.differences)
        self.confidences_list.extend(msg.confidence)
        self.tracking_ids_list.extend(msg.tracking_id)
        
        for xyxy, xywh in zip(msg.xyxy, msg.xywh):
            self.xyxys_list.append([xyxy.tl_x, xyxy.tl_y, xyxy.br_x, xyxy.br_y])
            self.xywhs_list.append([xywh.center_x, xywh.center_y, xywh.width, xywh.height])
        
        def is_behind_other_ball(blue_ball_coords, other_ball_coords):
            blue_tl_x, blue_tl_y, blue_br_x, blue_br_y = blue_ball_coords
            other_tl_x, other_tl_y, other_br_x, other_br_y = other_ball_coords
            return (blue_tl_x > other_tl_x and blue_tl_y > other_tl_y and blue_br_x < other_br_x and blue_br_y < other_br_y)
        
        blue_ball_indices = [i for i, class_id in enumerate(self.class_ids_list) if class_id == 0]
        filtered_blue_ball_indices = []
        for i in blue_ball_indices:
            blue_ball_coords = self.xyxys_list[i]
            behind_other_ball = False
            for j, class_id in enumerate(self.class_ids_list):
                if class_id in [1, 2]:  # Purple or Red ball
                    other_ball_coords = self.xyxys_list[j]
                    if is_behind_other_ball(blue_ball_coords, other_ball_coords):
                        behind_other_ball = True
                        break
            if not behind_other_ball:
                filtered_blue_ball_indices.append(i)
        
        if filtered_blue_ball_indices:
            largest_contour_index = max(filtered_blue_ball_indices, key=lambda i: self.contour_areas_list[i])
            
            self.closest_blue_ball = {
                'class_id': self.class_ids_list[largest_contour_index],
                'contour_area': self.contour_areas_list[largest_contour_index],
                'difference': self.differences_list[largest_contour_index],
                'confidence': self.confidences_list[largest_contour_index],
                'tracking_id': self.tracking_ids_list[largest_contour_index] if self.tracking_ids_list else None,
                'xyxy': self.xyxys_list[largest_contour_index],
                'xywh': self.xywhs_list[largest_contour_index]
            }
            
            self.get_logger().info(f"Closest Blue Ball: {self.closest_blue_ball}")
            self.contour_area_error = self.desired_contour_area - self.closest_blue_ball['contour_area']
            self.difference_error = self.closest_blue_ball['difference'] # Adjusted error calculation
            self.tracking_blue_ball = True
            
            self.move_robot()
            
    def PID_controller(self, error, error_sum, last_error, kp, ki, kd):
        P = kp * error
        error_sum += error
        error_sum = np.clip(error_sum, -self.max_integral, self.max_integral)
        I = ki * error_sum
        D = kd * (error - last_error)
        control = P + I + D
        return control, error_sum
        
    def move_robot(self):
        if self.closest_blue_ball['class_id'] is not None:
            self.get_logger().info(f"Contour Area Error: {self.contour_area_error}")
            self.get_logger().info(f"Difference Error: {self.difference_error}")
            
            if self.contour_area_error < self.contour_area_threshold and abs(self.difference_error) < self.difference_threshold:
                twist_msg = Twist()
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
                self.cmd_vel_pub.publish(twist_msg)
                self.get_logger().info("Reached the ball. Stopping the robot.")
                sys.exit()
  
                self.tracking_blue_ball = False
                self.closest_blue_ball = {
                    'class_id': None,
                    'contour_area': None,
                    'difference': None,
                    'confidence': None,
                    'tracking_id': None,
                    'xyxy': None,
                    'xywh': None
                }
                return

            self.linearX_error_sum += self.contour_area_error
            self.linearY_error_sum += self.difference_error
            self.angular_error_sum += self.yaw_error
            
            linear_x, self.linearX_error_sum = self.PID_controller(self.contour_area_error, self.linearX_error_sum, self.linearX_last_error,
                                                                  self.linearX_kp, self.linearX_ki, self.linearX_kd)
            
            linear_y, self.linearY_error_sum = self.PID_controller(self.difference_error, self.linearY_error_sum, self.linearY_last_error,
                                                                    self.linearY_kp, self.linearY_ki, self.linearY_kd)
            
            angular_z, self.angular_error_sum = self.PID_controller(self.yaw_error, self.angular_error_sum, self.angular_last_error,
                                                                    self.angular_kp, self.angular_ki, self.angular_kd)
            
            self.linearX_last_error = self.contour_area_error
            self.linearY_last_error = self.difference_error
            self.angular_last_error = self.yaw_error
            
            linear_x = np.clip(linear_x, -self.max_linear_speed, self.max_linear_speed)
            linear_y = np.clip(linear_y, -self.max_angular_speed, self.max_angular_speed)
            angular_z = np.clip(angular_z, -self.max_angular_speed, self.max_angular_speed) 
            
            twist_msg = Twist()
            twist_msg.linear.x = linear_x
            twist_msg.linear.y = -linear_y
            twist_msg.angular.z = angular_z  # Ensure correct direction
            
            self.cmd_vel_pub.publish(twist_msg)
            self.get_logger().info(f"Publishing cmd_vel: linear_x = {linear_x}, angular_z = {-angular_z}")
        
    def sweep_for_ball(self):
        twist_msg = Twist()
        if self.last_seen_direction == 'right':
            twist_msg.angular.z = -2.5
        else:
            twist_msg.angular.z = 2.5
        twist_msg.linear.x = 0.0
        
        self.cmd_vel_pub.publish(twist_msg)
        self.get_logger().info(f"Sweeping for ball: angular_z = {twist_msg.angular.z}")
    
    def get_camera_width(self):
        # Assuming a fixed camera resolution, e.g., 640x480
        return 640
    
def main(args=None):
    rclpy.init(args=args)
    ball_tracking_node = BallTrackingNode()
    rclpy.spin(ball_tracking_node)
    ball_tracking_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
