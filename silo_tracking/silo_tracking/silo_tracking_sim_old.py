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
from time import time
import os
import numpy as np
import sys
from ultralytics import YOLO
from ultralytics import utils
from r2_interfaces.srv import BestSilo
from r2_interfaces.srv import SiloToGo
from geometry_msgs.msg import Twist

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
                ('difference_threshold', 400)
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
        
        self.class_ids_list = []
        self.xyxys_list = []
        self.contour_areas_list = []
        self.differences_list = []
        self.best_silo_client = self.create_client(BestSilo, '/best_silo')
        
        # Wait for the service to be available
        while not self.best_silo_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().info('Service not available, waiting again...')
            
        # SiloToGo service client - to go to the best silo
        self.go_to_silo_client = self.create_client(SiloToGo, '/silo_to_go')
        
        # Wait for the service to be available
        while not self.go_to_silo_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().info('Service not available, waiting again...')
            
        # Request object for BestSilo service    
        self.request = BestSilo.Request()
              
        #Publisher
        
      
       
        
    
        self.silo_count = 0                                                                # Yaw alignment state
        self.silo_count_published = False                                             # Silo count published state
        
        self.best_silo = ''                                                           # Best silo - s1, s2, s3, s4, s5
        
        self.class_ids_list = []           
        self.silo_number_togo = 0                                                     # Silo number to go



    def send_silo_deciding_request(self):           # Send request to best_silo service
        self.get_logger().info('Sending request to best_silo service')
        
        #Future object for best_silo service- Asynchronous call
        self.future = self.best_silo_client.call_async(self.request)
        
        #Callback function to handle the response from the service 
        self.future.add_done_callback(self.handle_service_response)

    def handle_service_response(self, future):
        if future.done():                # Check if the future object is done
            try:
                response = future.result()
                
            except Exception as e:      # If the future object is done but with exception
                self.get_logger().info('Service call failed %r' % (e,))
                
            else:                       # If the future object is done and no exception
                self.best_silo = response.best_silo
                self.get_silo_number()
                self.get_logger().info('Best silo: %s' % self.best_silo)
                self.get_logger().info('Silo number to go: %d' % self.silo_number_togo)
                
    def send_silo_to_go_request(self):      # Send request to silo_to_go service
        
        # Request object for SiloToGo service
        request = SiloToGo.Request()
        request.silo_number = self.silo_number_togo        
        self.get_logger().info('Sending request to silo_to_go service')
        
        # Future object for silo_to_go service - Asynchronous call
        self.future = self.go_to_silo_client.call_async(request)
        
        # Callback function to handle the response from the service
        self.future.add_done_callback(self.handle_service_silo_to_go_response)
        
    def handle_service_silo_to_go_response(self, future):
        
        if future.done():
            try:
                response = future.result()
            except Exception as e:
                self.get_logger().info('Service call failed %r' % (e,))
            else:
                if response.success:
                    self.get_logger().info('Robot successfully aligned with silo %d' % self.silo_number_togo)
                else:
                    self.get_logger().info('Failed to align robot with silo %d' % self.silo_number_togo)
        
        

    def get_silo_number(self):      # Get silo number to go based on best silo
        if self.best_silo == 's1':
            self.silo_number_togo = 1
        
        elif self.best_silo == 's2':
            self.silo_number_togo = 2
            
        elif self.best_silo == 's3':
            self.silo_number_togo = 3
        
        elif self.best_silo == 's4':
            self.silo_number_togo = 4
            
        elif self.best_silo == 's5':
            self.silo_number_togo = 5
            
        # if self.logging:
        self.get_logger().info('Silo number to go: %d' % self.silo_number_togo)
    def yolo_results_callback(self, msg):
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
        
        self.get_logger().info(f"Class IDs: {self.class_ids_list}")
        self.get_logger().info(f"XyXys: {self.xyxys_list}")
        self.get_logger().info(f"Contour Areas: {self.contour_areas_list}")
        self.get_logger().info(f"Differences: {self.differences_list}")
        
        self.silos = [self.xyxys_list[i] for i in range(len(self.class_ids_list)) if self.class_ids_list[i] == 3]
        self.balls = [(self.class_ids_list[i], self.xyxys_list[i]) for i in range(len(self.class_ids_list)) if self.class_ids_list[i] in [0, 1, 2]]

        self.get_logger().info(f"Silos: {self.silos}")
        self.get_logger().info(f"Balls: {self.balls}")
        self.silo_count = self.class_ids_list.count(3)
        
        # if self.logging:
        self.get_logger().info('Silo count: %d' % self.silo_count)
        
        # Publish the silo count if it is greater than or equal to 5 - Only once AND Send request to best_silo service
        if not self.silo_count_published and self.silo_count >= 5:
            
            # if self.logging:
            self.get_logger().info('Silo count: %d' % self.silo_count)
                
            # Send request to best_silo service
            self.send_silo_deciding_request()
            
            # Update the state
            self.silo_count_published = True 
            
            # Sleep for 2 seconds 
            # time.sleep(2)
        if len(self.silos) >= 2:
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

        self.get_logger().info(f"Target Contour Area: {target_contour_area}")
        self.get_logger().info(f"Deviation: {deviation}")

        contour_area_error = self.desired_contour_area - target_contour_area
        deviation_error = deviation

        self.get_logger().info(f"Contour Area Error: {contour_area_error}")
        self.get_logger().info(f"Deviation Error: {deviation_error}")

        if (contour_area_error) < self.contour_area_threshold and abs(deviation_error) < self.difference_threshold:
            # self.stop_robot()
            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            
            self.cmd_vel_pub.publish(twist_msg)
            self.send_silo_to_go_request()
            self.get_logger().info("Reached the middle silo. Stopping the robot.")
            sys.exit()
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
