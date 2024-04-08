#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class NumberSubscriberNode(Node):
    def __init__(self):
        super().__init__("Number_counter")

        self.number_count_publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.number_subscriber_ = self.create_subscription(Twist, "/silo_data", self.callback_number_counter, 10)
        self.get_logger().info("Number counter has been started")

    def callback_number_counter(self, new_msg):
    # Create a new Twist message
        cmd_msg = Twist()
        if new_msg.linear.z == 1.0:
    # Assign the desired values to the Twist message
            cmd_msg.linear.x = new_msg.linear.x
            cmd_msg.angular.z = new_msg.angular.z

        
            if new_msg.linear.x ==0.0 and new_msg.angular.z ==0.0:
                cmd_msg.linear.x = 0.0
                cmd_msg.angular.z = 0.0
                self.number_count_publisher_.publish(cmd_msg)
                cmd_msg.linear.x = 0.0
                cmd_msg.angular.z = 0.0
                self.number_count_publisher_.publish(cmd_msg)
                cmd_msg.linear.x = 0.0
                cmd_msg.angular.z = 0.0
                self.number_count_publisher_.publish(cmd_msg)
                cmd_msg.linear.x = 0.0
                cmd_msg.angular.z = 0.0
                self.number_count_publisher_.publish(cmd_msg)
            #     count = count +1
            # if count > 5 :
                self.destroy_node()
    # Publish the Twist message on the "/cmd_vel" topic
        self.number_count_publisher_.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = NumberSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
