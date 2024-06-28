#!/usr/bin/env python3

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

from geometry_msgs.msg import Twist
import math
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from r2_interfaces.srv import BestSilo
from r2_interfaces.srv import SiloToGo
from r2_interfaces.msg import YoloResults
from r2_interfaces.msg import Xywh
from r2_interfaces.msg import XyXy


class NumberSubscriberNode(Node):
    def __init__(self):
        super().__init__("Number_counter")
        self.declare_parameters(
            namespace='',
            parameters=[
              
                ('track', 'ball')
            ]
        )
        # self.track = "silo"  # This can be changed to "ball" as needed
        self.track = self.get_parameter('track').value
        self.number_count_publisher_ = self.create_publisher(Twist, "/nav_vel", 10)
        
        self.silo_subscriber_ = self.create_subscription(Twist, "/silo_data", self.callback_silo_data, 10)
        self.ball_subscriber_ = self.create_subscription(Twist, "/ball_data", self.callback_ball_data, 10)
        
        self.get_logger().info("Number counter has been started")
        # BestSilo service client - to get the best silo
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
            
        if self.logging:
            self.get_logger().info('Silo number to go: %d' % self.silo_number_togo)

        
        
            

    

    def yolo_callback(self, msg):
        
        # Clear the list before updating
        self.class_ids_list.clear()
        self.class_ids_list.extend(msg.class_ids)
        
        # Count the number of silos detected
        self.silo_count = self.class_ids_list.count(3)
        
        if self.logging:
            self.get_logger().info('Silo count: %d' % self.silo_count)
        
        # Publish the silo count if it is greater than or equal to 5 - Only once AND Send request to best_silo service
        if not self.silo_count_published and self.silo_count >= 1:
            
            if self.logging:
                self.get_logger().info('Silo count: %d' % self.silo_count)
                
            # Send request to best_silo service
            self.send_silo_deciding_request()
            
            # Update the state
            self.silo_count_published = True 
            
            # Sleep for 2 seconds 
            time.sleep(2)
            
    def callback_silo_data(self, msg: Twist):
        if self.track == "silo":
            if msg.linear.x == 0.0 and msg.linear.y == 0.0 and msg.angular.z == 0.0:

                self.get_logger().info("Received zero velocities, exiting.")
                self.number_count_publisher_.publish(msg)
                self.send_silo_to_go_request()
                self.track="dont"
                
                # rclpy.shutdown()
                # sys.exit()
            self.number_count_publisher_.publish(msg)

    def callback_ball_data(self, msg: Twist):
        if self.track == "ball":
            if msg.linear.x == 0.0 and msg.linear.y == 0.0 and msg.angular.z == 0.0:
                self.get_logger().info("Received zero velocities, exiting.")
                self.number_count_publisher_.publish(msg)
                self.track="dont"
                # rclpy.shutdown()
                # sys.exit()
            self.number_count_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = NumberSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
