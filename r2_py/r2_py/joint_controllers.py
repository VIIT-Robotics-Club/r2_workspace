#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JointControllerNode(Node):
    def __init__(self):
        super().__init__('joint_controller_node')
        self.get_logger().info('Joint Controller Node started')
       
        # self.create_subscription(
        #     JointTrajectory,
        #     '/joint_trajectory_controller/joint_trajectory',
        #     self.joint_trajectory_callback,
        #     10
        # )
        
        self.joint_publisher = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10
        )
        
        self.create_timer(0.1, self.joint_trajectory_callback)
        
    def joint_trajectory_callback(self):
        # self.get_logger().info('Received Joint Trajectory')
        # x = JointTrajectory()
        # joint_names = [msg.joint_names[0], #lift
        #                msg.joint_names[1], #claw_left
        #                msg.joint_names[2]]  #claw_right

        # lift_position = msg.points[0].positions[0]
        # claw_left_position = msg.points[0].positions[1]
        # claw_right_position = msg.points[0].positions[2]
        
        # print('Lift Position: ', lift_position)
        # print('Claw Left Position: ', claw_left_position)
        # print('Claw Right Position: ', claw_right_position)
        
        #change the joint positions
        
        joint_trajectory_msg = JointTrajectory()
        joint_point_msg = JointTrajectoryPoint()
        
        joint_point_msg.positions = [0.5, 0.0, 0.0]  
        joint_point_msg.time_from_start.sec = 1
        joint_point_msg.time_from_start.nanosec = 20
        joint_trajectory_msg.points.append(joint_point_msg)
        
        self.joint_publisher.publish(joint_trajectory_msg)
        

def main(args=None):
    rclpy.init(args=args)
    joint_controller_node = JointControllerNode()
    rclpy.spin(joint_controller_node)
    joint_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



    
    





