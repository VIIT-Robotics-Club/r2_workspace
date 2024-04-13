#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class cmd_vel_slow_pub(Node):
    def __init__(self):
        super().__init__('cmd_vel_slow_pub')
       

        self.create_subscription(
            Twist,
            'cmd_vel_fast',
            self.cmd_vel_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.create_timer(0.1, self.timer_callback)

        self.lin_x = 0.0
        self.lin_y = 0.0
        self.ang_z = 0.0

    def cmd_vel_callback(self, msg):
        self.lin_x = msg.linear.x
        self.lin_y = msg.linear.y
        self.ang_z = msg.angular.z

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = self.lin_x 
        msg.linear.y = self.lin_y 
        msg.angular.z = self.ang_z 

        self.publisher.publish(msg)

def main(args=None):
    
    rclpy.init(args=args)

    cmd_vel_slow_pub_node = cmd_vel_slow_pub()

    rclpy.spin(cmd_vel_slow_pub_node)

    cmd_vel_slow_pub_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



    
    





