#!/usr/bin/env python3

'''
 Class names:
0- Blue-ball
1- Purple-ball
2- Red-Ball
3- silo
'''
from std_msgs.msg import Bool

from rclpy.node import Node
import rclpy
import cv_bridge
import cv2
from time import time
import os
import numpy as np
import sys
from ultralytics import YOLO
from ultralytics import utils
from r2_interfaces.srv import BestSilo
from r2_interfaces.srv import SiloToGo
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
from rcl_interfaces.msg import SetParametersResult


from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

from r2_interfaces.msg import YoloResults
from r2_interfaces.msg import Xywh
from r2_interfaces.msg import XyXy


class SiloDetectionNode(Node):
    def __init__(self):
        super().__init__('silo_detection_node')
        self.get_logger().info("Silo Detection Node has been started")
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('desired_contour_area', 47000),
                ('linear_kp', 1.0),
                ('linear_ki', 0.0),
                ('linear_kd', 0.0),
                ('angular_kp', 0.1),
                ('angular_ki', 0.0),
                ('angular_kd', 0.0),
                ('max_linear_speed', 2.0),
                ('max_angular_speed', 1.0),
                ('max_integral', 10.0),
                ('contour_area_threshold', 3000),
                ('difference_threshold', 400),
                ('logging', False)
                ]
        )
        
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        
        self.desired_contour_area = self.get_parameter('desired_contour_area').value
                
        self.linear_kp = self.get_parameter('linear_kp').value
        self.linear_ki = self.get_parameter('linear_ki').value
        self.linear_kd = self.get_parameter('linear_kd').value
        
        self.angular_kp = self.get_parameter('angular_kp').value
        self.angular_ki = self.get_parameter('angular_ki').value
        self.angular_kd = self.get_parameter('angular_kd').value
        self.active=False
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.max_integral = self.get_parameter('max_integral').value
        
        self.contour_area_threshold = self.get_parameter('contour_area_threshold').value
        self.difference_threshold = self.get_parameter('difference_threshold').value
        self.logging = self.get_parameter("logging").value

        self.linear_error_sum = 0.0
        self.linear_last_error = 0.0
        
        self.angular_error_sum = 0.0
        self.angular_last_error = 0.0
        
        self.silos = []
        self.balls = []

        self.create_subscription(
            YoloResults,
            'yolo_results',
            self.yolo_results_callback,
            10
        )
        
        self.cmd_vel_pub = self.create_publisher(
            Twist, 
            'nav_vel', 
            10
        )
        self.create_service(SetBool, "/silo_tracking_srv", self.silo_tracking_callback)
        self.publisher = self.create_publisher(Bool, 'status', 10)
        
        self.reached=0
        self.class_ids_list = []
        self.xyxys_list = []
        self.contour_areas_list = []
        self.differences_list = []
       
              
        #Publisher
    
    def parameters_callback(self, params):
        for param in params:
            param_name = param.name
            param_value = param.value
            setattr(self, param_name, param_value)
        return SetParametersResult(successful=True)
      
       
        
    def silo_tracking_callback(self, request, response):
        self.active=True
        self.get_logger().info("silo tracking is active")
        response.success = True
        response.message = "silo tracking started"
        return response
      
    def yolo_results_callback(self, msg):
        if self.active:
            if self.logging:
                self.get_logger().info("Yolo Results Received")
            self.class_ids_list.clear()
            self.xyxys_list.clear()
            self.contour_areas_list.clear()
            self.differences_list.clear()
            
            self.class_ids_list.extend(msg.class_ids)
            for xyxy in msg.xyxy:
                self.xyxys_list.append([xyxy.tl_x, xyxy.tl_y, xyxy.br_x, xyxy.br_y])
            self.contour_areas_list.extend(msg.contour_area)
            self.differences_list.extend(msg.differences)
            
            # self.get_logger().info(f"Class IDs: {self.class_ids_list}")
            # self.get_logger().info(f"XyXys: {self.xyxys_list}")
            # self.get_logger().info(f"Contour Areas: {self.contour_areas_list}")
            # self.get_logger().info(f"Differences: {self.differences_list}")
            
            self.silos = [self.xyxys_list[i] for i in range(len(self.class_ids_list)) if self.class_ids_list[i] == 3]
            self.balls = [(self.class_ids_list[i], self.xyxys_list[i]) for i in range(len(self.class_ids_list)) if self.class_ids_list[i] in [0, 1, 2]]

            # self.get_logger().info(f"Silos: {self.silos}")
            # self.get_logger().info(f"Balls: {self.balls}")
            self.silo_count = self.class_ids_list.count(3)
            
            
                
                # Sleep for 2 seconds 
                # time.sleep(2)
            if len(self.silos) >= 3:
                # self.stop_robot()
                self.move_to_middle_silo()
            else:
                self.sweep_for_silos()

    def move_to_middle_silo(self):
        # Assuming the middle silo is the 3rd one in the list of 5 silos
        middle_silo_index = len(self.silos)//2
        middle_silo = self.silos[middle_silo_index]
        target_contour_area = self.contour_areas_list[middle_silo_index]
        deviation = self.differences_list[middle_silo_index]
        if self.logging:
            self.get_logger().info(f"Target Contour Area: {target_contour_area}")
            self.get_logger().info(f"Deviation: {deviation}")

        contour_area_error = self.desired_contour_area - target_contour_area
        deviation_error = deviation
        if self.logging:    
            self.get_logger().info(f"Contour Area Error: {contour_area_error}")
            self.get_logger().info(f"Deviation Error: {deviation_error}")

        if (contour_area_error) < self.contour_area_threshold and abs(deviation_error) < self.difference_threshold:
        # if (contour_area_error) < self.contour_area_threshold:
        # if (contour_area_error) < 30000:
            # self.stop_robot()
            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            
            self.cmd_vel_pub.publish(twist_msg)
            self.active =False

            bool_msg = Bool()
            bool_msg.data = True
            self.publisher.publish(bool_msg)
            # self.send_silo_to_go_request()
            self.get_logger().info("Reached the middle silo. Stopping the robot.")
            # sys.exit()
            return

        contour_area_error=contour_area_error/15000
        deviation_error=deviation_error/170
        self.linear_error_sum += contour_area_error
        self.angular_error_sum += contour_area_error

        linear_x, self.linear_error_sum = self.PID_controller(contour_area_error, self.linear_error_sum, self.linear_last_error,
                                                            self.linear_kp, self.linear_ki, self.linear_kd)
        
        angular_z, self.angular_error_sum = self.PID_controller(deviation_error, self.angular_error_sum, self.angular_last_error,
                                                                self.angular_kp, self.angular_ki, self.angular_kd)
        
        self.linear_last_error = contour_area_error
        self.angular_last_error = deviation_error

        linear_x = np.clip(linear_x, -self.max_linear_speed, self.max_linear_speed)
        angular_z = np.clip(angular_z, -self.max_angular_speed, self.max_angular_speed)

        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.angular.z = -angular_z

        self.cmd_vel_pub.publish(twist_msg)
        if self.logging:
            self.get_logger().info(f"Moving towards middle silo: linear_x = {twist_msg.linear.x}, angular_z = {twist_msg.angular.z}")

    def PID_controller(self, error, error_sum, last_error, kp, ki, kd):
        P = kp * error
        error_sum = np.clip(error_sum, -self.max_integral, self.max_integral)
        I = ki * error_sum
        D = kd * (error - last_error)
        control = P + I + D
        return control, error_sum

    def sweep_for_silos(self):
        twist_msg = Twist()
        twist_msg.angular.z = 1.20
        twist_msg.linear.x = 0.0
        
        self.cmd_vel_pub.publish(twist_msg)
        if self.logging:
            self.get_logger().info(f"Sweeping for silos: angular_z = {twist_msg.angular.z}")

    def stop_robot(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        
        self.cmd_vel_pub.publish(twist_msg)
        self.get_logger().info("Stopping the robot.")

def main(args=None):
    rclpy.init(args=args)
    silo_detection_node = SiloDetectionNode()
    rclpy.spin(silo_detection_node)
    silo_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
