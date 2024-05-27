# /usr/bin/env python3

# How to run this code:
# ros2 run luna_control luna_wall_align --ros-args -p x_goal:=40.0 -p y_goal:=50.0 -p kp_linear:=0.05 -p ki_linear:=0.00 -p kd_linear:=0.00 -p kp_angular:=0.08 -p ki_angular:=0.00 -p kd_angular:=0.0

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int64MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from r2_interfaces.srv import SiloToGo

import sys
import os

class LunaWallAlignNode(Node):
    def __init__(self):
        super().__init__('luna_wall_align')
        self.get_logger().info('Initializing my_node')

        self.prev_ang_error = 0

        self.declare_parameters(
            namespace='',
            parameters=[
                ('linear_x_max', 2.0),
                ('linear_y_max', 2.0),
                ('linear_x_min', -2.0),
                ('linear_y_min', -2.0),
                ('angular_z_max', 3.0),
                ('angular_z_min', -3.0),
                ('kp_linear_x', 0.5),
                ('ki_linear_x', 0.01),
                ('kd_linear_x', 0.00),
                ('kp_linear_y', 0.9),
                ('ki_linear_y', 0.01),
                ('kd_linear_y', 0.00),
                ('kp_angular', 1.4),
                ('ki_angular', 0.0),
                ('kd_angular', 0.0),          
                # ('x_goal', 13.0),
                # ('y_goal', 250.0),
                # ('silo_number', 1),    
                ('silo_1_x', 0.1),
                ('silo_1_y', 3.28),
                ('silo_2_x', 0.1),
                ('silo_2_y', 2.55),
                ('silo_3_x', 0.1),
                ('silo_3_y', 1.76),
                ('silo_4_x', 0.1),
                ('silo_4_y', 1.04),
                ('silo_5_x', 0.1),
                ('silo_5_y', 0.31),     
                ]
        )

        self. linear_x_max = self.get_parameter('linear_x_max').value
        self.linear_y_max = self.get_parameter('linear_y_max').value
        self.linear_x_min = self.get_parameter('linear_x_min').value
        self.linear_y_min = self.get_parameter('linear_y_min').value
        self.angular_z_max = self.get_parameter('angular_z_max').value
        self.angular_z_min = self.get_parameter('angular_z_min').value

        self.kp_linear_x = self.get_parameter('kp_linear_x').value
        self.ki_linear_x = self.get_parameter('ki_linear_x').value
        self.kd_linear_x = self.get_parameter('kd_linear_x').value
        self.kp_linear_y = self.get_parameter('kp_linear_y').value
        self.ki_linear_y = self.get_parameter('ki_linear_y').value
        self.kd_linear_y = self.get_parameter('kd_linear_y').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.ki_angular = self.get_parameter('ki_angular').value
        self.kd_angular = self.get_parameter('kd_angular').value

        # self.x_goal = self.get_parameter('x_goal').value
        # self.y_goal = self.get_parameter('y_goal').value
        # self.silo_number = self.get_parameter('silo_number').value

        self.silo_1_x = self.get_parameter('silo_1_x').value
        self.silo_1_y = self.get_parameter('silo_1_y').value
        self.silo_2_x = self.get_parameter('silo_2_x').value
        self.silo_2_y = self.get_parameter('silo_2_y').value
        self.silo_3_x = self.get_parameter('silo_3_x').value
        self.silo_3_y = self.get_parameter('silo_3_y').value
        self.silo_4_x = self.get_parameter('silo_4_x').value
        self.silo_4_y = self.get_parameter('silo_4_y').value
        self.silo_5_x = self.get_parameter('silo_5_x').value
        self.silo_5_y = self.get_parameter('silo_5_y').value

        
        # All 6 lunar sensors subscribers
        
        self.luna_fl_subscriber = self.create_subscription(         #Front Left
            Range, '/distance/lidar_fl_1', self.luna_fl_callback, 10
        )
        
        self.luna_fr_subscriber = self.create_subscription(         #Front Right
            Range, '/distance/lidar_fr_1', self.luna_fr_callback, 10
        )
        
        # self.luna_lb_subscriber = self.create_subscription(         #Left Back      
        #     Range,
        #     '/distance/lidar_lb_1',
        #     self.luna_lb_callback,
        #     10
        # )
        
        # self.luna_lf_subscriber = self.create_subscription(         #Left Front       
        #     Range,
        #     '/distance/lidar_lf_1',
        #     self.luna_lf_callback,
        #     10
        # )
        
        
        self.luna_rf_subscriber = self.create_subscription(         #Right Front
            Range,   '/distance/lidar_rf_1', self.luna_rf_callback, 10
        )
        
        
        self.luna_rb_subscriber = self.create_subscription(         #Right Back           
            Range, '/distance/lidar_rb_1', self.luna_rb_callback, 10
        )        
        

        self.cmd_vel_publisher = self.create_publisher( Twist, 'cmd_vel', 10
        )
        
        self.create_service( SiloToGo,  'silo_to_go',  self.silo_to_go_callback)

        # Initialize the luna data
        self.luna_rb = 0.00     #Right Back
        self.luna_rf = 0.00     #Right Front
        self.luna_fl = 0.00     #Front Left
        self.luna_fr = 0.00     #Front Right
        self.luna_lf = 0.00     #Left Front
        self.luna_lb = 0.00     #Left Back
        
        
        #State Management Variable 
        self.correct_angle = True

        # Set initial integrals to zero
        self.int_error_linear_x = 0.0
        self.int_error_linear_y = 0.0
        self.int_error_angular_z = 0.0

        self.positions = {
            1: {'x': self.silo_1_x, 'y': self.silo_1_y},
            2: {'x': self.silo_2_x, 'y': self.silo_2_y},
            3: {'x': self.silo_3_x, 'y': self.silo_3_y},
            4: {'x': self.silo_4_x, 'y': self.silo_4_y},
            5: {'x': self.silo_5_x, 'y': self.silo_5_y},
        }
        
        self.silo_number = 0
        self.active = False



    def silo_to_go_callback(self, request, response):
        self.silo_number = request.silo_number
        self.x_goal = self.positions[self.silo_number]['x']
        self.y_goal = self.positions[self.silo_number]['y']

        self.get_logger().info(f'x: {self.x_goal}')
        self.get_logger().info(f'y: {self.y_goal}')

        self.active = True
        self.correct_angle = True
        self.service_response = response
        
        self.timer = self.create_timer(0.1, self.align_robot)
        return response
        
        


    def pid_controller(self, error, previous_error, int_error, kp, ki, kd, dt):
        control_action = kp * error + ki * int_error + kd * ((error - previous_error) / dt)
        return control_action
    
    def luna_fl_callback(self, msg: Range):
        try:
            self.luna_fl = float(msg.range)
        except Exception as e:
            self.get_logger().error(f"Error in luna_fl_callback: {e}")
            
        
    def luna_fr_callback(self, msg: Range):
        try:
            self.luna_fr = float(msg.range)
            print(self.luna_fr)
        except Exception as e:
            print(f"Error in luna_fr_callback: {e}")

    def luna_lb_callback(self, msg: Range):
        try:
            self.luna_lb = float(msg.range)
            print(self.luna_lb)
        except Exception as e:
            print(f"Error in luna_lb_callback: {e}")

    def luna_lf_callback(self, msg: Range):
        try:
            self.luna_lf = float(msg.range)
            print(self.luna_lf)
        except Exception as e:
            print(f"Error in luna_lf_callback: {e}")

    def luna_rf_callback(self, msg: Range):
        try:
            self.luna_rf = float(msg.range)
            print(self.luna_rf)
        except Exception as e:
            print(f"Error in luna_rf_callback: {e}")

    def luna_rb_callback(self, msg: Range):
        try:
            self.luna_rb = float(msg.range)
            print(self.luna_rb)
        except Exception as e:
            print(f"Error in luna_rb_callback: {e}")
            
            
    def align_robot(self):
        self.get_logger().info('x: %f' % self.x_goal)
        self.get_logger().info('y: %f' % self.y_goal)
        
        if not self.active:
            return

        # Calculate the difference between the sensor readings
        x_diff = self.luna_fl - self.luna_fr
        y_diff = self.luna_rf - self.luna_rb

        # Create a new Twist message
        twist = Twist()

        x_avg = (self.luna_fl + self.luna_fr) / 2
        y_avg = (self.luna_rf + self.luna_rb) / 2

        if self.correct_angle:
            # Calculate the time difference
            dt = 0.2  # Assuming fixed sample rate of 10 Hz

            # Check if angular z correction is required
            if abs(x_diff) <= 0.03:  # Or you can use abs(y_diff) <= 3
                self.correct_angle = False
                self.get_logger().info('Robot is aligned, adjusting linear velocities')

            # Apply PID controller for angular z
            ang_error = self.luna_fl - self.luna_fr
            self.int_error_angular_z += ang_error
            twist.angular.z = self.pid_controller(ang_error, self.prev_ang_error, self.int_error_angular_z, self.kp_angular, self.ki_angular, self.kd_angular, dt)
            self.prev_ang_error = ang_error

            if x_diff < 0:  # Luna front left < front right -> left side is closer to the wall
                twist.angular.z = abs(twist.angular.z)  # Rotate Anti-clockwise
            else:
                twist.angular.z = -abs(twist.angular.z) 

            self.get_logger().info('Angular z: %f' % twist.angular.z)
        
        else:
            self.get_logger().info('Entered linear velocity adjustment')

            if (abs(x_avg - self.x_goal) >= 0.05) or (abs(y_avg - self.y_goal) >= 0.05):
                self.get_logger().info("x_avg - self.x_goal = " + str(abs(x_avg - self.x_goal)))
                self.get_logger().info("x_goal : " + str(self.x_goal))
                self.get_logger().info("y_goal : " + str(self.y_goal))
                dt = 0.2
                prev_lin_error_x = 0
                prev_lin_error_y = 0
                lin_error_x = self.x_goal - x_avg               
                lin_error_y = self.y_goal - y_avg
                self.int_error_linear_x += lin_error_x
                self.int_error_linear_y += lin_error_y

                self.get_logger().info("self.y_goal - y_avg = " + str(abs(self.y_goal - y_avg)))

                ang_error = self.luna_fl - self.luna_fr
                self.int_error_angular_z += ang_error
                self.get_logger().info("ang_error = " + str(ang_error))

                twist.linear.x = self.pid_controller(lin_error_x, prev_lin_error_x, self.int_error_linear_x, self.kp_linear_x, self.ki_linear_x, self.kd_linear_x, dt)
                twist.linear.y = self.pid_controller(lin_error_y, prev_lin_error_y, self.int_error_linear_y, self.kp_linear_y, self.ki_linear_y, self.kd_linear_y, dt)
                twist.angular.z = -self.pid_controller(ang_error, self.prev_ang_error, self.int_error_angular_z, self.kp_angular, self.ki_angular, self.kd_angular, dt)
                self.prev_ang_error = ang_error
                prev_lin_error_x, prev_lin_error_y = lin_error_x, lin_error_y

                twist.linear.x = -max(min(twist.linear.x, self.linear_x_max), self.linear_x_min) 
                twist.linear.y = max(min(twist.linear.y, self.linear_y_max), self.linear_y_min) 
                twist.angular.z = max(min(twist.angular.z, self.angular_z_max), self.angular_z_min) 

                if abs(y_avg - self.y_goal) <= 0.02:
                    twist.linear.y = 0.0
                if abs(x_avg - self.x_goal) <= 0.02:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0

                self.get_logger().info('Linear x: %f' % twist.linear.x)
                self.get_logger().info('Linear y: %f' % twist.linear.y)

            else:
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.angular.z = 0.0
                self.get_logger().info('Robot is aligned to the goal')
                self.cmd_vel_publisher.publish(twist)
                self.active = False
                 
                if self.service_response:
                    self.service_response.success = True
                    self.get_logger().info('Sending response')
                    self.timer.cancel()
                    self.service_response = None
               

                
        # Publish the Twist message
        self.cmd_vel_publisher.publish(twist)
    

def main(args=None):
    rclpy.init(args=args)
    node = LunaWallAlignNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()