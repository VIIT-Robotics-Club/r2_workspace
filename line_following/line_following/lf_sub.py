#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType


from std_msgs.msg import Int32

from geometry_msgs.msg import Twist


class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.get_logger().info('Line Follower Node Started')

        # Declare and get parameters
        self.declare_parameter("desired_value", 35.0)       # The desired sensor reading
        self.declare_parameter("Kp", 0.7)                  # Proportional gain
        self.declare_parameter("Ki", 0.0)                  # Integral gain
        self.declare_parameter("Kd", 0.3)                  # Derivative gain
        self.declare_parameter("base_speed", 2.0)           # Base speed (linear x)

        self.desired_value = self.get_parameter("desired_value").value
        self.Kp = self.get_parameter("Kp").value
        self.Ki = self.get_parameter("Ki").value
        self.Kd = self.get_parameter("Kd").value
        self.base_speed = self.get_parameter("base_speed").value

        self.error_sum = 0           # Sum of errors (for integral term)
        self.last_error = 0          # Last error (for derivative term)
        # self.state = "FOLLOWING"     # Initial State


        # Create a publisher for the cmd_vel topic
        self.publisher_ = self.create_publisher(
            Twist,
            '/nav_vel', 
            10)

        # Create a subscription to the lsa_08 topic
        self.subscription = self.create_subscription(
            Int32,
            '/line_lsa',
            self.listener_callback,
            10)
        
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
        

    def listener_callback(self, msg):
        # This method is called when a new message is received on the lsa_08 topic
        current_sensor_reading = msg.data

        self.get_logger().info('I heard: "%s"' % msg.data)

        # Create a new Twist message
        twist = Twist()

        # Calculate the control output
        output_angular_z,error = self.calculate_angular_velocity(current_sensor_reading)

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
    line_follower = LineFollowerNode()
    rclpy.spin(line_follower)
    line_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()