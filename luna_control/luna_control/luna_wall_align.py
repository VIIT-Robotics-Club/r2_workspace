# /usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray
from geometry_msgs.msg import Twist

class LunaWallAlignNode(Node):
    def __init__(self):
        super().__init__('luna_wall_align')
        self.get_logger().info('Initializing my_node')


        self.declare_parameters(
            namespace='',
            parameters=[
                ('linear_x_max', 0.5),
                ('linear_y_max', 0.5),
                ('linear_x_min', -0.5),
                ('linear_y_min', -0.5),
                ('angular_z_max', 1.0),
                ('angular_z_min', -1.0),
                ('kp_linear', 0.5),
                ('kp_angular', 0.5),
                ('x_goal', 40.0),
                ('y_goal', 50.0)                
                ]
        )

        self. linear_x_max = self.get_parameter('linear_x_max').value
        self.linear_y_max = self.get_parameter('linear_y_max').value
        self.linear_x_min = self.get_parameter('linear_x_min').value
        self.linear_y_min = self.get_parameter('linear_y_min').value
        self.angular_z_max = self.get_parameter('angular_z_max').value
        self.angular_z_min = self.get_parameter('angular_z_min').value
        self.kp_linear = self.get_parameter('kp_linear').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.x_goal = self.get_parameter('x_goal').value
        self.y_goal = self.get_parameter('y_goal').value


        self.luna_subscriber = self.create_subscription(
            Int64MultiArray,
            'luna_data',
            self.luna_callback,
            10
        )

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        # Initialize the luna data
        self.luna_1 = 0.00  
        self.luna_2 = 0.00  
        self.luna_3 = 0.00  
        self.luna_4 = 0.00  


    def luna_callback(self, msg):
        self.luna_1 = float(msg.data[3]) - 2         #Front Left -14     -  2 is the offset (Luna sensor error correction)
        self.luna_2 = float(msg.data[1])             #Front right -12
        self.luna_3 = float(msg.data[0])             #Side Left - 11
        self.luna_4 = float(msg.data[2])             #Side right -13 
        self.get_logger().info('Luna data received')

        #All 4 luna readings
        self.get_logger().info('Luna 1: %d' % self.luna_1)
        self.get_logger().info('Luna 2: %d' % self.luna_2)
        self.get_logger().info('Luna 3: %d' % self.luna_3)
        self.get_logger().info('Luna 4: %d' % self.luna_4)

        #Calculate the difference between the sensor readings
        x_diff = self.luna_1 - self.luna_2
        y_diff = self.luna_3 - self.luna_4

        #create a new Twist message
        twist = Twist()

        x_avg = (self.luna_1 + self.luna_2) / 2
        y_avg = (self.luna_3 + self.luna_4) / 2
        

        if (abs(x_diff) > 1):  #Or you can use abs(y_diff) > 1

        #Adjust the angular z velocity based on the difference between the sensor readings
            twist.angular.z = self.kp_angular * (x_diff)
            twist.angular.z = max(min(twist.angular.z, self.angular_z_max), self.angular_z_min)

            if x_diff > 0:   #Lune front left > front right 
                twist.angular.z = abs(twist.angular.z)    #Rotate Anti-clockwise
            else:
                twist.angular.z = -abs(twist.angular.z)     #Rotate clockwise

            self.get_logger().info('Angular z: %f' % twist.angular.z)

        else:            # If the robot is aligned, adjust the linear velocities

            self.get_logger().info('Robot is aligned it angular z, adjusting linear velocities')
            
            if (abs(x_avg-self.x_goal) >= 2) or (abs(y_avg - self.y_goal) >= 2): 
                twist.linear.x = self.kp_linear * (x_avg - self.x_goal) * self.linear_x_max
                twist.linear.y = -self.kp_linear * (y_avg - self.y_goal) * self.linear_y_max
                
                twist.linear.x = max(min(twist.linear.x, self.linear_x_max), self.linear_x_min)
                twist.linear.y = max(min(twist.linear.y, self.linear_y_max), self.linear_y_min)
                twist.angular.z = 0.0
                
                self.get_logger().info('Linear x: %f' % twist.linear.x)
                self.get_logger().info('Linear y: %f' % twist.linear.y)

            else:
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.angular.z = 0.0

                

        #Publish the Twist message
        self.cmd_vel_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = LunaWallAlignNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()