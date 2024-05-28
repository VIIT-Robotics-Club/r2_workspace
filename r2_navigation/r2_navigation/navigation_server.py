#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3, Twist
from example_interfaces.msg import Bool

from r2_interfaces.msg import YoloResults
from r2_interfaces.srv import BestSilo
from r2_interfaces.srv import SiloToGo


import math
import sys
import time

class OrientAndMoveNode(Node):
    def __init__(self):
        super().__init__('orient_and_move')
        
        # Subscriptions
        self.create_subscription(Vector3, '/rpy', self.rpy_callback, 10)
        self.create_subscription(YoloResults, '/yolo_results', self.yolo_callback, 10)
        self.create_subscription(Bool, '/robot_altitude_state', self.altitude_callback, 10)
        
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
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('kp', 1.0),                                # Kp 
                ('ki', 0.1),
                ('kd', 0.3),
                ('yaw_error_threshold', 5.0),
                ('goal_yaw', 90.0),
                ('logging', False)
            ]
        )
        
        self.kp = self.get_parameter('kp').value                            
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.yaw_error_threshold = self.get_parameter('yaw_error_threshold').value
        self.goal_yaw = self.get_parameter('goal_yaw').value
        self.logging = self.get_parameter('logging').value
        
        
        self.current_yaw = 0.0                  # Current yaw in degrees
        self.current_yaw_rad = 0.0              # Current yaw in radians
        
        self.silo_count = 0                                                           # Number of silos detected                                       
        self.goal_yaw_rad = math.radians(self.goal_yaw)                               # Goal yaw in radians
        self.yaw_error_threshold_rad = math.radians(self.yaw_error_threshold)         # Yaw error threshold in 
        
        self.robot_altitude_state = False                                             # Robot altitude state  
        self.initial_altitude_state = None                                            # Initial altitude state                    
        self.is_yaw_aligned = False                                                   # Yaw alignment state
        self.silo_count_published = False                                             # Silo count published state
        
        self.best_silo = ''                                                           # Best silo - s1, s2, s3, s4, s5
        
        self.class_ids_list = []                                                      # List of class ids from YOLO results
        self.prev_yaw_error = 0.0                                                     # Previous yaw error                     
        self.integral = 0.0                                                           # Integral term for PID
        self.prev_time = self.get_clock().now()                                       # Previous time for PID

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
            

    def altitude_callback(self, msg):
        
        self.robot_altitude_state = msg.data
        
        if self.logging:
            self.get_logger().info('Robot altitude state: %s' % self.robot_altitude_state)
        
        
        # Check if the robot altitude state is True
        if self.is_yaw_aligned:             # Check if the yaw is aligned
            if self.initial_altitude_state is None:     # Check if the initial altitude state is None
                
                if self.logging:
                    self.get_logger().info('Setting initial altitude state')
                
                self.initial_altitude_state = self.robot_altitude_state     # Set the initial altitude state
                
                self.get_logger().info('Initial altitude state: %s' % self.initial_altitude_state)
                
            elif self.robot_altitude_state != self.initial_altitude_state:      # Check if the robot altitude state is different from the initial altitude state
               
                twist_msg = Twist()
                twist_msg.linear.x = 0.0
                self.cmd_vel_publisher.publish(twist_msg)
                
                self.get_logger().info('Altitude changed, stopping the robot')
                
                self.is_yaw_aligned = False  # Reset state for future tasks
                self.initial_altitude_state = None  # Reset initial altitude state
                
                self.get_logger().debug('Sleeping for 3 seconds before sending silo_to_go request')
                time.sleep(3)
                
                self.get_logger().info('Sending silo_to_go request')
                
                self.send_silo_to_go_request()
                
                self.get_logger().debug('System exiting...')
                sys.exit()

    def rpy_callback(self, msg):
        
        self.current_yaw = msg.z
        self.current_yaw_rad = math.radians(self.current_yaw)
        
        twist_msg = Twist()
        
        if self.logging:
            self.get_logger().info('Updated current yaw: %f' % self.current_yaw)
            self.get_logger().info('Updated current yaw in radians: %f' % self.current_yaw_rad)
        
        if not self.is_yaw_aligned:
            
            # Calculate yaw error
            yaw_error = self.goal_yaw_rad - self.current_yaw_rad
            
            if self.logging:
                self.get_logger().info('Yaw error before normalize: %f' % yaw_error)
            
            # Normalize yaw_error to be within -pi to pi - Required for PID
            yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
            
            if self.logging:
                self.get_logger().info('Yaw error after normalize: %f' % yaw_error)
            
            # Time calculations for PID            
            current_time = self.get_clock().now()
            delta_time = (current_time - self.prev_time).nanoseconds / 1e9
            
            if self.logging:
                self.get_logger().info('Delta time: %f' % delta_time)
                
            # Check if yaw is aligned
            if abs(yaw_error) < self.yaw_error_threshold_rad:
                # Stop the robot
                twist_msg.angular.z = 0.0
                twist_msg.linear.x = 0.0
                self.cmd_vel_publisher.publish(twist_msg)
                self.get_logger().info('Goal reached, yaw aligned')
                self.is_yaw_aligned = True
                return
            
            # PID calculations
            self.integral += yaw_error * delta_time                                                       # Integral term
            derivative = (yaw_error - self.prev_yaw_error) / delta_time                                   # Derivative term
            angular_z = (self.kp * yaw_error) + (self.ki * self.integral) + (self.kd * derivative)        # PID output
            
            # Publish cmd_vel
            twist_msg.angular.z = angular_z
            self.cmd_vel_publisher.publish(twist_msg)
            
            if self.logging:
                self.get_logger().info('Publishing cmd_vel: angular.z = %f' % twist_msg.angular.z)
            
            # Update previous values
            self.prev_yaw_error = yaw_error
            self.prev_time = current_time
            
        else:
            # Move forward if yaw is aligned
            twist_msg.linear.x = 1.0  # Adjust the speed as needed
            self.cmd_vel_publisher.publish(twist_msg)
            
            if self.logging:
                self.get_logger().info('Moving forward with linear.x = %f' % twist_msg.linear.x)

    def yolo_callback(self, msg):
        
        # Clear the list before updating
        self.class_ids_list.clear()
        self.class_ids_list.extend(msg.class_ids)
        
        # Count the number of silos detected
        self.silo_count = self.class_ids_list.count(3)
        
        if self.logging:
            self.get_logger().info('Silo count: %d' % self.silo_count)
        
        # Publish the silo count if it is greater than or equal to 5 - Only once AND Send request to best_silo service
        if not self.silo_count_published and self.silo_count >= 5:
            
            if self.logging:
                self.get_logger().info('Silo count: %d' % self.silo_count)
                
            # Send request to best_silo service
            self.send_silo_deciding_request()
            
            # Update the state
            self.silo_count_published = True 
            
            # Sleep for 2 seconds 
            time.sleep(2)

def main(args=None):
    rclpy.init(args=args)
    orient_and_move_node = OrientAndMoveNode()
    try:
        rclpy.spin(orient_and_move_node)
    except KeyboardInterrupt:
        pass
    finally:
        orient_and_move_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
