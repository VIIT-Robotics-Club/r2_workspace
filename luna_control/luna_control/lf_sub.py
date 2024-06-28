#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType

from std_msgs.msg import Int32, Bool
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist
import sys
import time

class LineFollowerService(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.get_logger().info('Line Follower Node Started')

        # Declare and get parameters
        self.declare_parameter("desired_value", 35.0)       # The desired sensor reading
        self.declare_parameter("Kp", 0.9)                  # Proportional gain  0.7
        self.declare_parameter("Ki", 0.0)                  # Integral gain
        self.declare_parameter("Kd", 0.3)                  # Derivative gain
        self.declare_parameter("base_speed", -2.0)           # Base speed (linear x)
        # self.declare_parameter("retry", False)
        # self.declare_parameter("arena_side", "blue")

        self.desired_value = self.get_parameter("desired_value").value
        self.Kp = self.get_parameter("Kp").value
        self.Ki = self.get_parameter("Ki").value
        self.Kd = self.get_parameter("Kd").value
        self.base_speed = self.get_parameter("base_speed").value
        # self.retry = self.get_parameter("retry").value
        # self.arena_side = self.get_parameter("arena_side").value

        self.error_sum = 0           # Sum of errors (for integral term)
        self.last_error = 0          # Last error (for derivative term)
        # self.state = "FOLLOWING"     # Initial State
        self.nodeCount = 0
        self.state = False
        self.delay = 0.01
        self.mulFac = 0.8

        self.node_1_state = False
        self.curr_junction_data = 0
        self.switch = True
        
        self.execute = False
        self.retry = False

        # Create a publisher for the cmd_vel topic
        self.publisher_ = self.create_publisher(
            Twist,
            '/nav_vel', 
            10)

        # Create a subscription to the lsa_08 topic
        self.subscription = self.create_subscription(
            Int32,
            '/line_lsa',
            self.lsa_callback,
            10)
        
        # create subscription for junctions
        self.subscription = self.create_subscription(
            Int32,
            '/junction',
            self.junction_callback,
            10)
        
        self.srv = self.create_service(
            SetBool, 
            "lf_service", 
            self.serviceCallback)
        
        self.status_pub = self.create_publisher(
            Bool,
            "status",
            10)
        
    def serviceCallback(self, request, response):
        if request.data:
            self.retry = True

        self.execute = True
        response.success = True
        return response
        
    def calculate_angular_velocity(self, current_sensor_reading):
        # Calculate the error
        error = current_sensor_reading - self.desired_value

        # Calculate the integral and derivative terms
        self.error_sum += error
        error_derivative = error - self.last_error

        # Calculate the control output
        output = (self.Kp * error) + (self.Ki * self.error_sum) + (self.Kd * error_derivative)

        divisor = (self.Kp*self.desired_value) + self.Kd + self.Ki

        normalized = -output/divisor    
        return normalized, error
    
    def junction_callback(self, msg:Int32):
        diff = msg.data - self.curr_junction_data

        if msg.data == self.curr_junction_data:
            self.switch = False

        if diff > 0 and self.switch:
            self.curr_junction_data = msg.data
            self.switch = False

        self.nodeCount = msg.data - self.curr_junction_data

        self.get_logger().info("junction = " + str(self.nodeCount))       

    def lsa_callback(self, msg):
        # This method is called when a new message is received on the lsa_08 topic
        current_sensor_reading = msg.data

        # self.get_logger().info('I heard: "%s"' % msg.data)

        # Line follower logics
        if self.retry and self.execute:
            self.retry_callback(current_sensor_reading)
        elif self.execute:
            self.lf_callback(current_sensor_reading)

        # for testing purpose

        # Calculate the control output
        # output_angular_z,error = self.calculate_angular_velocity(current_sensor_reading)

        # twist.angular.z = output_angular_z
        # twist.linear.x = -self.base_speed  
        # twist.linear.y = 0.0

        # # Publish the new Twist message
        # self.publisher_.publish(twist)
        # self.get_logger().info('Published nav_vel: linear.x = "%s", angular.z = "%s"' % (twist.linear.x, twist.angular.z))

        # # Update the last error
        # self.last_error = error

    def lf_callback(self, data):
        twist = Twist()

        # first junction
        if self.nodeCount == 1 and data == 255:
            if (not self.state):
                self.moveForward(self.delay)
                self.state = True
                self.node_1_state = True
            
            else:
                if self.last_error > 35:
                    # move right
                    twist.angular.z = 0.0     # try aligning it with IMU
                    twist.linear.x = 0.0 
                    twist.linear.y = -self.base_speed*self.mulFac

                else:
                    # move left
                    twist.angular.z = 0.0     # try aligning it with IMU
                    twist.linear.x = 0.0
                    twist.linear.y = self.base_speed*self.mulFac
        
        # second junction
        elif self.nodeCount >= 4:  # update for node in area 3
            twist.angular.z = 0.0
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            self.publisher_.publish(twist)

            status_msg = Bool()
            status_msg.data = True
            self.status_pub.publish(status_msg)

        else:
            
            if self.node_1_state:
                self.get_logger().info("entered")
                self.moveRight(0.7)
                self.node_1_state = False

            # Calculate the control output
            output_angular_z, error = self.calculate_angular_velocity(data)

            twist.angular.z = output_angular_z
            twist.linear.x = self.base_speed  
            twist.linear.y = 0.0

            self.last_error = error

        # Publish the new Twist message
        self.publisher_.publish(twist)
        self.get_logger().info('Published nav_vel: linear.x = "%s", angular.z = "%s"' % (twist.linear.x, twist.angular.z))

        # Update the last error
        # self.last_error = error

    def moveRight(self, delay):
        twist = Twist()

        current_time = time.time()
        while(time.time() - current_time < delay):
            twist.angular.z = self.base_speed
            twist.linear.x = self.base_speed*0.6
            twist.linear.y = -self.base_speed*self.mulFac

            # Publish the new Twist message
            self.publisher_.publish(twist)
            self.get_logger().info('Published nav_vel: linear.x = "%s", angular.z = "%s"' % (twist.linear.x, twist.angular.z))

        twist.angular.z = 0.0
        twist.linear.x = 0.0
        twist.linear.y = 0.0

        # Publish the new Twist message
        self.publisher_.publish(twist)
        self.get_logger().info('Published nav_vel: linear.x = "%s", angular.z = "%s"' % (twist.linear.x, twist.angular.z))


    def moveForward(self, delay):
        twist = Twist()

        current_time = time.time()
        while(time.time() - current_time < delay):
            twist.angular.z = 0.0
            twist.linear.x = self.base_speed 
            twist.linear.y = 0.0

            # Publish the new Twist message
            self.publisher_.publish(twist)
            self.get_logger().info('Published nav_vel: linear.x = "%s", angular.z = "%s"' % (twist.linear.x, twist.angular.z))

        twist.angular.z = 0.0
        twist.linear.x = 0.0
        twist.linear.y = 0.0

        # Publish the new Twist message
        self.publisher_.publish(twist)
        self.get_logger().info('Published nav_vel: linear.x = "%s", angular.z = "%s"' % (twist.linear.x, twist.angular.z))

    
    def retry_callback(self, data):
        twist = Twist()
        self.get_logger().info("entered retry logic") 
        if self.nodeCount == 1 and data == 255:
            if self.arena_side == "blue":
                # move right
                twist.angular.z = 0.0     # try aligning it with IMU
                twist.linear.x = 0.0 
                twist.linear.y = -self.base_speed
            else:
                # move left
                twist.angular.z = 0.0     # try aligning it with IMU
                twist.linear.x = 0.0 
                twist.linear.y = self.base_speed

        elif self.nodeCount >= 2 and data == 255:
            if self.arena_side == "blue":
                # move right
                twist.angular.z = 0.0     # try aligning it with IMU
                twist.linear.x = 0.0 
                twist.linear.y = -self.base_speed
            else:
                # move left
                twist.angular.z = 0.0     # try aligning it with IMU
                twist.linear.x = 0.0 
                twist.linear.y = self.base_speed
        
        elif self.nodeCount >= 2:
            # stop
            twist.angular.z = 0.0
            twist.linear.x = 0.0 
            twist.linear.y = 0.0

        else:
            # Calculate the control output
            output_angular_z, error = self.calculate_angular_velocity(data)

            twist.angular.z = output_angular_z
            twist.linear.x = self.base_speed  
            twist.linear.y = 0.0
        
        # Publish the new Twist message
        self.publisher_.publish(twist)
        self.get_logger().info('Published nav_vel: linear.x = "%s", angular.z = "%s"' % (twist.linear.x, twist.angular.z))

        # Update the last error
        self.last_error = error


def main(args=None):
    rclpy.init(args=args)
    line_follower = LineFollowerService()
    rclpy.spin(line_follower)
    line_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()