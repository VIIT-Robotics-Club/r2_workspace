#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Twist
from r2_interfaces.msg import YoloResults
from r2_interfaces.srv import BestSilo
from example_interfaces.msg import Bool
import math
import sys
import time

class OrientAndMoveActionServer(Node):
    def __init__(self):
        super().__init__('orient_and_count_action_server')
        
        self.create_subscription(Vector3, '/rpy', self.rpy_callback, 10)
        self.create_subscription(YoloResults, '/yolo_results', self.yolo_callback, 10)
        self.create_subscription(Bool, '/robot_altitude_state', self.altitude_callback, 10)
        
        self.best_silo_client = self.create_client(BestSilo, '/best_silo')
        
        while not self.best_silo_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().info('Service not available, waiting again...')
            
        self.request = BestSilo.Request()
              
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('kp', 1.0),
                ('ki', 0.1),
                ('kd', 0.3),
                ('yaw_error_threshold', 5.0),
                ('goal_yaw', 90.0)
            ]
        )
        
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.yaw_error_threshold = self.get_parameter('yaw_error_threshold').value
        self.goal_yaw = self.get_parameter('goal_yaw').value
        
        self.current_yaw = 0.0
        self.current_yaw_rad = 0.0 
        self.silo_count = 0
        self.goal_yaw_rad = math.radians(self.goal_yaw)
        self.yaw_error_threshold_rad = math.radians(self.yaw_error_threshold)
        self.robot_altitude_state = False
        self.initial_altitude_state = None
        self.is_yaw_aligned = False
        self.silo_count_published = False
        
        self.best_silo = ''
        
        self.class_ids_list = []
        self.prev_yaw_error = 0.0
        self.integral = 0.0
        self.prev_time = self.get_clock().now()
        
        self.silo_number_togo = 0

    def send_silo_deciding_request(self):
        self.get_logger().info('Sending request to best_silo service')
        self.future = self.best_silo_client.call_async(self.request)
        self.future.add_done_callback(self.handle_service_response)

    def handle_service_response(self, future):
        if future.done():
            try:
                response = future.result()
            except Exception as e:
                self.get_logger().info('Service call failed %r' % (e,))
            else:
                self.best_silo = response.best_silo
                self.get_silo_number()
                self.get_logger().info('Best silo: %s' % self.best_silo)
                self.get_logger().info('Silo number to go: %d' % self.silo_number_togo)
        

    def get_silo_number(self):
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
            

    def altitude_callback(self, msg):
        self.robot_altitude_state = msg.data
        if self.is_yaw_aligned:
            if self.initial_altitude_state is None:
                self.initial_altitude_state = self.robot_altitude_state
                self.get_logger().info('Initial altitude state: %s' % self.initial_altitude_state)
            elif self.robot_altitude_state != self.initial_altitude_state:
                twist_msg = Twist()
                twist_msg.linear.x = 0.0
                self.cmd_vel_publisher.publish(twist_msg)
                self.get_logger().info('Altitude changed, stopping the robot')
                self.is_yaw_aligned = False  # Reset state for future tasks
                self.initial_altitude_state = None  # Reset initial altitude state
                sys.exit()

    def rpy_callback(self, msg):
        self.current_yaw = msg.z
        self.current_yaw_rad = math.radians(self.current_yaw)
        
        twist_msg = Twist()
        
        # self.get_logger().info('Updated current yaw: %f' % self.current_yaw)
        
        if not self.is_yaw_aligned:
            # Calculate yaw error
            yaw_error = self.goal_yaw_rad - self.current_yaw_rad
            # Normalize yaw_error to be within -pi to pi
            yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
            self.get_logger().info('Yaw error: %f' % yaw_error)
            
            current_time = self.get_clock().now()
            delta_time = (current_time - self.prev_time).nanoseconds / 1e9
            
            if abs(yaw_error) < self.yaw_error_threshold_rad:
                # Stop the robot
                twist_msg.angular.z = 0.0
                twist_msg.linear.x = 0.0
                self.cmd_vel_publisher.publish(twist_msg)
                self.get_logger().info('Goal reached, yaw aligned')
                self.is_yaw_aligned = True
                return
            
            # PID calculations
            self.integral += yaw_error * delta_time
            derivative = (yaw_error - self.prev_yaw_error) / delta_time
            angular_z = (self.kp * yaw_error) + (self.ki * self.integral) + (self.kd * derivative)
            
            twist_msg.angular.z = angular_z
            self.cmd_vel_publisher.publish(twist_msg)
            
            # self.get_logger().info('Publishing cmd_vel: angular.z = %f' % twist_msg.angular.z)
            
            # Update previous values
            self.prev_yaw_error = yaw_error
            self.prev_time = current_time
        else:
            # Move forward if yaw is aligned
            twist_msg.linear.x = 1.0  # Adjust the speed as needed
            self.cmd_vel_publisher.publish(twist_msg)
            # self.get_logger().info('Moving forward with linear.x = %f' % twist_msg.linear.x)

    def yolo_callback(self, msg):
        self.class_ids_list.clear()
        self.class_ids_list.extend(msg.class_ids)
        self.silo_count = self.class_ids_list.count(3)
        # self.get_logger().info('Silo count: %d' % self.silo_count)
        
        if not self.silo_count_published and self.silo_count >= 5:
            # self.get_logger().info('Silo count: %d' % self.silo_count)
            self.send_silo_deciding_request()
            self.silo_count_published = True 
            time.sleep(2)

def main(args=None):
    rclpy.init(args=args)
    action_server = OrientAndMoveActionServer()
    try:
        rclpy.spin(action_server)
    except KeyboardInterrupt:
        pass
    finally:
        action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
