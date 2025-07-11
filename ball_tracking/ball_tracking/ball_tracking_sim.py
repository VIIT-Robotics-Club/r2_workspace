#!/usr/bin/env python3

'''
 Class names:
0- Blue-ball
1- Purple-ball
2- Red-Ball
3- silo
'''

from rclpy.node import Node
import rclpy
import cv_bridge
import cv2
from time import time, sleep
import os
import sys
import numpy as np

from ultralytics import YOLO
from ultralytics import utils

from geometry_msgs.msg import Twist

from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

from r2_interfaces.msg import YoloResults
from r2_interfaces.msg import Xywh
from r2_interfaces.msg import XyXy


class BallTrackingNode(Node):
    def __init__(self):
        super().__init__('ball_tracking_node')
        self.get_logger().info("Ball Tracking Node has been started")
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('desired_contour_area', 70000),
                ('linear_kp', 0.002),
                ('linear_ki', 0.001),
                ('linear_kd', 0.001),
                ('angular_kp', 0.5),
                ('angular_ki', 0.05),
                ('angular_kd', 0.01),
                ('max_linear_speed', 1.0),
                ('max_angular_speed', 5.0),
                ('max_integral', 10.0),
                ('contour_area_threshold', 20000),
                ('difference_threshold', 30),
                ('sweep_duration', 1.0),
                ('logging', True)
                ]
        )
        
        self.desired_contour_area = self.get_parameter('desired_contour_area').value
                
        self.linear_kp = self.get_parameter('linear_kp').value
        self.linear_ki = self.get_parameter('linear_ki').value
        self.linear_kd = self.get_parameter('linear_kd').value
        
        self.angular_kp = self.get_parameter('angular_kp').value
        self.angular_ki = self.get_parameter('angular_ki').value
        self.angular_kd = self.get_parameter('angular_kd').value
        
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.max_integral = self.get_parameter('max_integral').value
        
        self.contour_area_threshold = self.get_parameter('contour_area_threshold').value
        self.difference_threshold = self.get_parameter('difference_threshold').value
        self.sweep_duration = self.get_parameter('sweep_duration').value
        
        self.logging = self.get_parameter('logging').value
        
        self.linear_error_sum = 0.0
        self.linear_last_error = 0.0
        
        self.angular_error_sum = 0.0
        self.angular_last_error = 0.0
        
        self.tracking_blue_ball = False
        self.sweeping = True
        self.tracking_id = None
        
        self.difference_error = 0.0
        self.contour_area_error = 0.0

        
        self.create_subscription(
            YoloResults,
            'yolo_results',
            self.yolo_results_callback,
            10
        )
        
        self.cmd_vel_pub = self.create_publisher(
            Twist, 
            'cmd_vel', 
            10
        )
        
        # Arrays to store values from the message
        self.class_ids_list = []
        self.contour_areas_list = []
        self.differences_list = []
        self.confidences_list = []  
        self.tracking_ids_list = []
        self.xyxys_list = []
        self.xywhs_list = []
        
        # Variables to store the closest blue ball
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
        
    def yolo_results_callback(self, msg):
        # self.get_logger().info("Yolo Results Received")
        
        # Clear Previous Values
        self.class_ids_list.clear()
        self.contour_areas_list.clear()
        self.differences_list.clear()
        self.confidences_list.clear()
        self.tracking_ids_list.clear()
        self.xyxys_list.clear()
        self.xywhs_list.clear()
        
        # Store the values from the message
        self.class_ids_list.extend(msg.class_ids)
        self.contour_areas_list.extend(msg.contour_area)
        self.differences_list.extend(msg.differences)
        self.confidences_list.extend(msg.confidence)
        self.tracking_ids_list.extend(msg.tracking_id)
        
        # Loop through the XyXy and Xywh data in the message
        for xyxy, xywh in zip(msg.xyxy, msg.xywh):
            # Append the data to the lists
            self.xyxys_list.append([xyxy.tl_x, xyxy.tl_y, xyxy.br_x, xyxy.br_y])
            self.xywhs_list.append([xywh.center_x, xywh.center_y, xywh.width, xywh.height])
        
        # if self.logging:
        #     # Log the received values       
        #     self.get_logger().info(f"Class IDs: {self.class_ids_list}")
        #     self.get_logger().info(f"Contour Areas: {self.contour_areas_list}")
        #     self.get_logger().info(f"Differences: {self.differences_list}")
        #     self.get_logger().info(f"Confidences: {self.confidences_list}")
        #     self.get_logger().info(f"Tracking IDs: {self.tracking_ids_list}")
        #     self.get_logger().info(f"XyXys: {self.xyxys_list}")
        #     self.get_logger().info(f"Xywhs: {self.xywhs_list}")
            
        if self.sweeping:
            #Sweep until the first blue ball is found
            blue_ball_indices = [i for i, class_id in enumerate(self.class_ids_list) if class_id == 0]
            
            if blue_ball_indices:
                self.sweeping = False
                self.get_logger().info("Blue ball found, sweeping for additional 1 second")
                sleep(self.sweep_duration)
                self.select_closest_blue_ball()
        else:
            # Continue tracking the same blue ball
            self.update_tracking_blue_ball()       
            
        # If no balls detected, start sweeping
        if not self.class_ids_list:
            self.tracking_blue_ball = False
            self.sweeping = True
            self.sweep_for_ball()
            
    def select_closest_blue_ball(self):
        blue_ball_indices = [i for i, class_id in enumerate(self.class_ids_list) if class_id == 0]

        # Filter out blue balls that are behind red or purple balls
        filtered_blue_ball_indices = []
        for i in blue_ball_indices:
            blue_ball_coords = self.xyxys_list[i]
            behind_other_ball = False
            for j, class_id in enumerate(self.class_ids_list):
                if class_id in [1, 2]:  # Purple or Red ball
                    other_ball_coords = self.xyxys_list[j]
                    if self.is_behind_other_ball(blue_ball_coords, other_ball_coords):
                        behind_other_ball = True
                        break
            if not behind_other_ball:
                filtered_blue_ball_indices.append(i)
        
        if filtered_blue_ball_indices:
            largest_contour_index = max(filtered_blue_ball_indices, key=lambda i: self.contour_areas_list[i])
            # Ensure the index is valid
            if largest_contour_index < len(self.tracking_ids_list):
                self.tracking_id = self.tracking_ids_list[largest_contour_index]
                self.closest_blue_ball = {
                    'class_id': self.class_ids_list[largest_contour_index],
                    'contour_area': self.contour_areas_list[largest_contour_index],
                    'difference': self.differences_list[largest_contour_index],
                    'confidence': self.confidences_list[largest_contour_index],
                    'tracking_id': self.tracking_ids_list[largest_contour_index],
                    'xyxy': self.xyxys_list[largest_contour_index],
                    'xywh': self.xywhs_list[largest_contour_index]
                }
                
                if self.logging:
                    self.get_logger().info(f"Closest Blue Ball: {self.closest_blue_ball}")
                    
                self.contour_area_error = self.desired_contour_area - self.closest_blue_ball['contour_area']
                self.difference_error = self.closest_blue_ball['difference']
                self.tracking_blue_ball = True
                self.move_robot()
            else:
                self.get_logger().error(f"Invalid index {largest_contour_index} for tracking_ids_list with length {len(self.tracking_ids_list)}")

    def update_tracking_blue_ball(self):
        for i, tracking_id in enumerate(self.tracking_ids_list):
            if tracking_id == self.tracking_id:
                self.closest_blue_ball = {
                    'class_id': self.class_ids_list[i],
                    'contour_area': self.contour_areas_list[i],
                    'difference': self.differences_list[i],
                    'confidence': self.confidences_list[i],
                    'tracking_id': self.tracking_ids_list[i],
                    'xyxy': self.xyxys_list[i],
                    'xywh': self.xywhs_list[i]
                }
                self.contour_area_error = self.desired_contour_area - self.closest_blue_ball['contour_area']
                self.difference_error = self.closest_blue_ball['difference']
                
                # if self.logging:
                #     self.get_logger().info(f"Closest Blue Ball: {self.closest_blue_ball}")
                
                self.move_robot()
                return
        
        # If the tracked ball is not found, stop tracking and sweep again
        
        if self.logging:
            self.get_logger().info(f"Tracked Ball with tracking_id {self.tracking_id} not found. Sweeping again.")
        
        self.tracking_blue_ball = False
        self.sweeping = True
        self.sweep_for_ball()

        
    def is_behind_other_ball(self, blue_ball_coords, other_ball_coords):
        blue_tl_x, blue_tl_y, blue_br_x, blue_br_y = blue_ball_coords
        other_tl_x, other_tl_y, other_br_x, other_br_y = other_ball_coords
        return (blue_tl_x > other_tl_x and blue_tl_y > other_tl_y and blue_br_x < other_br_x and blue_br_y < other_br_y)
            
                 
    def PID_controller(self, error, error_sum, last_error, kp, ki, kd):
        
        # Calculate the proportional term
        P = kp * error
        
        # Prevent integral wind-up by clamping the error sum
        error_sum = np.clip(error_sum, -self.max_integral, self.max_integral)
        
        # Calculate the integral term
        I = ki * error_sum
        
        # Calculate the derivative term
        D = kd * (error - last_error)
        
        # Calculate the control variable
        control = P + I + D
        
        return control, error_sum
        
    def move_robot(self):
        if self.closest_blue_ball['class_id'] is not None:
            if self.logging:
                self.get_logger().info(f"Contour Area Error: {self.contour_area_error}")
                self.get_logger().info(f"Difference Error: {self.difference_error}")
            
            if abs(self.contour_area_error) < self.contour_area_threshold and abs(self.difference_error) < self.difference_threshold:
                twist_msg = Twist()
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
                self.cmd_vel_pub.publish(twist_msg)
                self.get_logger().info("Reached the ball. Stopping the robot.")
                
                sys.exit()
                
            self.linear_error_sum += self.contour_area_error
            self.angular_error_sum += self.difference_error
            
            linear_x, self.linear_error_sum = self.PID_controller(self.contour_area_error, self.linear_error_sum, self.linear_last_error,
                                                                self.linear_kp, self.linear_ki, self.linear_kd)
            
            angular_z, self.angular_error_sum = self.PID_controller(self.difference_error, self.angular_error_sum, self.angular_last_error,
                                                                    self.angular_kp, self.angular_ki, self.angular_kd)
            
            self.linear_last_error = self.contour_area_error
            self.angular_last_error = self.difference_error
            
            linear_x = np.clip(linear_x, -self.max_linear_speed, self.max_linear_speed)
            angular_z = np.clip(angular_z, -self.max_angular_speed, self.max_angular_speed)
            
            twist_msg = Twist()
            twist_msg.linear.x = linear_x
            twist_msg.angular.z = -angular_z  # Negative to correct the direction of rotation
            
            self.cmd_vel_pub.publish(twist_msg)
            
            if self.logging:    
                self.get_logger().info(f"Publishing cmd_vel: linear_x = {linear_x}, angular_z = {angular_z}")
        
        
    def sweep_for_ball(self):
        twist_msg = Twist()
        if self.last_seen_direction == 'right':
            twist_msg.angular.z = -2.0  # Sweep right
        else:
            twist_msg.angular.z = 2.0  # Sweep left
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        
        self.cmd_vel_pub.publish(twist_msg)
        
        if self.logging:
            self.get_logger().info(f"Sweeping for ball: angular_z = {twist_msg.angular.z}")
    
def main(args=None):
    rclpy.init(args=args)
    ball_tracking_node = BallTrackingNode()
    rclpy.spin(ball_tracking_node)
    ball_tracking_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
