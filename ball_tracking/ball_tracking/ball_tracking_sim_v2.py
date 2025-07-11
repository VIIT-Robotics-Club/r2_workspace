#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tkinter as tk
from tkinter import ttk
import numpy as np
import sys

from geometry_msgs.msg import Twist
from r2_interfaces.msg import YoloResults

class BallTrackingNode(Node):
    def __init__(self):
        super().__init__('ball_tracking_node')
        self.get_logger().info("Ball Tracking Node has been started")
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('desired_contour_area', 220000),
                ('linear_kp', 0.1),
                ('linear_ki', 0.0002),
                ('linear_kd', 0.00),
                ('angular_kp', 0.01),
                ('angular_ki', 0.01),
                ('angular_kd', 0.01),
                ('max_linear_speed', 2.0),
                ('max_angular_speed', 0.5),
                ('max_integral', 10.0),
                ('contour_area_threshold', 3000),
                ('difference_threshold', 30)
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
        
        self.linear_error_sum = 0.0
        self.linear_last_error = 0.0
        self.angular_error_sum = 0.0
        self.angular_last_error = 0.0
        self.tracking_blue_ball = False
        self.difference_error = 0.0
        self.contour_area_error = 0.0

        self.create_subscription(YoloResults, 'yolo_results', self.yolo_results_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'nav_vel', 10)
        self.cmd_vel_pub_count = 0
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
        
            
    
    def parameters_callback(self, params):

        # update local state of all parameters
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
        
        def is_inside_silo(blue_ball_coords, silo_coords):
            blue_tl_x, blue_tl_y, blue_br_x, blue_br_y = blue_ball_coords
            silo_tl_x, silo_tl_y, silo_br_x, silo_br_y = silo_coords
            return (blue_tl_x >= silo_tl_x and blue_tl_y >= silo_tl_y and blue_br_x <= silo_br_x and blue_br_y <= silo_br_y)
        
        blue_ball_indices = [i for i, class_id in enumerate(self.class_ids_list) if class_id == 0]
        silo_indices = [i for i, class_id in enumerate(self.class_ids_list) if class_id == 3] 
        
        filtered_blue_ball_indices = []
        for i in blue_ball_indices:
            blue_ball_coords = self.xyxys_list[i]
            behind_other_ball = False
            inside_silo = False
            
            for j, class_id in enumerate(self.class_ids_list):
                if class_id in [1, 2]:  # Purple or Red ball
                    other_ball_coords = self.xyxys_list[j]
                    if is_behind_other_ball(blue_ball_coords, other_ball_coords):
                        behind_other_ball = True
                        break
            
            for k in silo_indices:
                silo_coords = self.xyxys_list[k]
                if is_inside_silo(blue_ball_coords, silo_coords):
                    inside_silo = True
                    break

            if not behind_other_ball and not inside_silo:
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
            self.difference_error = self.closest_blue_ball['difference']
            self.tracking_blue_ball = True

            if self.difference_error < 0:
                self.last_seen_direction = 'left'
            else:
                self.last_seen_direction = 'right'

            self.move_robot()
        else:
            self.sweep_for_ball()
           
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

            self.linear_error_sum += self.contour_area_error / 10000
            self.angular_error_sum += self.difference_error/70
            
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
            twist_msg.angular.z = -angular_z  # Ensure correct direction
            self.cmd_vel_pub_count=self.cmd_vel_pub_count+1
            if(self.cmd_vel_pub_count >9):
                self.cmd_vel_pub.publish(twist_msg)
                self.cmd_vel_pub_count=0
            self.get_logger().info(f"Publishing cmd_vel: linear_x = {linear_x}, angular_z = {-angular_z}")
        
    def sweep_for_ball(self):
        twist_msg = Twist()
        if self.last_seen_direction == 'right':
            twist_msg.angular.z = -0.4
        else:
            twist_msg.angular.z = 0.4
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
