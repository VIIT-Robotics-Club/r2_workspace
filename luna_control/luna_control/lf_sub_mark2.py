#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType

from std_msgs.msg import Int32, Bool
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist
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
        self.declare_parameter("arena_side", "blue")
        self.declare_parameter('logging', True)

        self.desired_value = self.get_parameter("desired_value").value
        self.Kp = self.get_parameter("Kp").value
        self.Ki = self.get_parameter("Ki").value
        self.Kd = self.get_parameter("Kd").value
        self.base_speed = self.get_parameter("base_speed").value
        # self.retry = self.get_parameter("retry").value
        self.arena_side = self.get_parameter("arena_side").value
        self.logging = self.get_parameter("logging").value


        self.error_sum = 0           # Sum of errors (for integral term)
        self.last_error = 0          # Last error (for derivative term)
        # self.state = "FOLLOWING"     # Initial State
        
        self.nodeCount = 0
        self.pitch = 1.5
        self.prev_junction_data = 0

        self.curr_junction_time = time.time()
        self.prev_junction_time = self.curr_junction_time

        self.state = False
        self.delay = 0.2

        self.node_2_state = False
        self.initial_junction_data = 0     # 0
        self.switch = False

        self.new_junction = True

        # self.false_junction_counter = 0
        
        self.execute = False
        self.retry = False

        self.prev_jnc_data = 0
        self.prev_data = 0

        # to see all logs
        # self.logging = True

        # Create a publisher for the nav_vel topic
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

        self.halt_sub = self.create_subscription(
            Bool,
            '/halt',
            self.halt_callback,
            10)
        
        # create subscription for junctions
        self.subscription = self.create_subscription(
            Int32,
            '/junction',
            self.junction_callback,
            10)
        
        self.srv = self.create_service(
            SetBool, 
            "/lf_srv", 
            self.serviceCallback)
        
        self.status_pub = self.create_publisher(
            Bool,
            "status",
            10)

    def halt_callback(self, msg:Bool):
        self.error_sum = 0           # Sum of errors (for integral term)
        self.last_error = 0          # Last error (for derivative term)
        # self.state = "FOLLOWING"     # Initial State
        
        self.nodeCount = 0
        self.pitch = 1.5
        self.prev_junction_data = 0

        self.curr_junction_time = time.time()
        self.prev_junction_time = self.curr_junction_time

        self.state = False
        self.delay = 0.2

        self.node_2_state = False
        self.initial_junction_data = 0     # 0
        self.switch = False

        self.new_junction = True

        # self.false_junction_counter = 0
        
        self.execute = False
        self.retry = True

        self.prev_jnc_data = 0
        self.prev_data = 0
        
    def serviceCallback(self, request, response):
        self.get_logger().info("line follower is active")
        
        # if request.data:
        #     self.retry = True

        self.execute = True
        self.retry = False
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
        # store initial junction count 
        if not self.switch:
            self.initial_junction_data = msg.data
            self.prev_junction_data = self.initial_junction_data

            # time stamp of initial data
            self.curr_junction_time = time.time()
            self.prev_junction_time = self.curr_junction_time
            self.switch = True
            # print("entered")

        diff = msg.data - self.prev_junction_data
        # self.get_logger().info("entered ****************************************** ")

        if diff > 0:
            self.curr_junction_time = time.time()

            if (self.curr_junction_time - self.prev_junction_time) > self.pitch:
                # self.get_logger().info("time diff = " + str(self.curr_junction_time - self.prev_junction_time))
                # use difference between current junction count (msg.data) and initial junction count (initial_junction_data) as nodeCount
                self.nodeCount = msg.data - self.initial_junction_data
                self.prev_junction_data = msg.data

                # if self.retry_1_state:
                #     self.get_logger().info("entered ****************************************** ")
                #     self.initial_junction_data += 1
            else:
                self.initial_junction_data += 1

        self.prev_junction_time = self.curr_junction_time
        self.prev_junction_data = msg.data

    def lsa_callback(self, msg):
        # This method is called when a new message is received on the lsa_08 topic
        current_sensor_reading = msg.data

        # self.get_logger().info('I heard: "%s"' % msg.data)
        self.initialize()

        # Line follower logics
        if self.retry and self.execute:
            self.retry_callback(current_sensor_reading)
        elif self.execute:
            self.vel_ctrl(current_sensor_reading)

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

    def initialize(self):
        pass

    def vel_ctrl(self, data):
        twist = Twist()

        # if self.logging and self.execute:
        self.get_logger().info("junction = " + str(self.nodeCount))

        if self.nodeCount >= 5:
            # stop the bot
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = 0.0
            self.execute = False

            msg = Bool()
            msg.data = True
            self.status_pub.publish(msg)

        elif self.nodeCount == 2 and data == 255:

            if self.arena_side == "blue":
                # rotate right
                self.rotate_left(twist)
            else:
                # rotate left
                self.rotate_right(twist)

        elif self.nodeCount == 3 and data == 255:

            if self.arena_side == "blue":
                # rotate right
                self.rotate_right(twist)
            else:
                # rotate left
                self.rotate_left(twist)

        else:
            # Calculate the control output
            output_angular_z, error = self.calculate_angular_velocity(data)

            twist.angular.z = output_angular_z
            twist.linear.x = self.base_speed
            twist.linear.y = 0.0

            # Update the last error
            self.last_error = error

        # Publish the new Twist message
        self.publisher_.publish(twist)
        # self.get_logger().info('Published nav_vel: linear.x = "%s", angular.z = "%s"' % (twist.linear.x, twist.angular.z))

        # if self.new_junction:
        #     self.prev_data = data
        #     self.prev_jnc_data = self.nodeCount
        #     self.new_junction = False

    def rotate_left(self, twist:Twist):
        # if self.logging:  
            # self.get_logger().info('Rotation started. ')
        
        # twist_msg = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        
        # if data == 255:
        twist.angular.z = 1.7
        self.publisher_.publish(twist)
        # else:
        #     self.new_junction = True
        #     self.prev_jnc_data = self.nodeCount

        # if self.logging:  
            # self.get_logger().info('Rotation ended. ')

    def rotate_right(self, twist:Twist):
        # if self.logging:  
            # self.get_logger().info('Rotation started. ')
        
        # twist_msg = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        
        # if data == 255:
        twist.angular.z = -1.7
        self.publisher_.publish(twist)
        # else:
        #     self.new_junction = True
        #     self.prev_jnc_data = self.nodeCount

        # if self.logging:
            # self.get_logger().info('Rotation ended. ')
    
    def retry_callback(self, data):
        twist = Twist()
        # self.get_logger().info("entered retry logic") 

        self.get_logger().info("nodeCount: " + str(self.nodeCount))
        # self.get_logger().info("retry 1 state: " + str(self.retry_1_state))

        if self.nodeCount >= 4:
            # stop the bot
            twist.angular.z = 0.0
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            self.execute = False

        elif self.nodeCount == 1 and data == 255:
            if self.arena_side == "blue":
                # rotate right
                self.rotate_left(twist)
            else:
                # rotate left
                self.rotate_right(twist)

        elif self.nodeCount == 2 and data == 255:
            if self.arena_side == "blue":
                # rotate right
                self.rotate_right(twist)
            else:
                # rotate left
                self.rotate_left(twist)

        # elif self.prev_jnc_data != self.nodeCount and data == 255:
        #     if self.last_error > 35:
        #         # rotate right
        #         self.rotate_right()

        #     else:
        #         # rotate left
        #         self.rotate_right()

        elif data == 255:
            if self.arena_side == "blue":
                # rotate right
                self.rotate_right()
            else:
                # rotate left
                self.rotate_left()

        else:
            # Calculate the control output
            output_angular_z, error = self.calculate_angular_velocity(data)

            twist.angular.z = output_angular_z
            twist.linear.x = self.base_speed  
            twist.linear.y = 0.0

            # Update the last error
            self.last_error = error

        # Publish the new Twist message
        self.publisher_.publish(twist)
        # self.get_logger().info('Published nav_vel: linear.x = "%s", linear.y = "%s", angular.z = "%s"' % (twist.linear.x, twist.linear.y, twist.angular.z))
        # self.prev_jnc_data = self.nodeCount

def main(args=None):
    rclpy.init(args=args)
    line_follower = LineFollowerService()
    rclpy.spin(line_follower)
    line_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()