#!/usr/bin/env python3

'''
 Class names:
0- Blue-ball
1- Purple-ball
2- Red-Ball
3- silo
'''

from rclpy.node import Node
import rclpy
import cv_bridge
import cv2
from time import time
import os
import numpy as np

from ultralytics import YOLO
from ultralytics import utils

from geometry_msgs.msg import Twist

from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

from r2_interfaces.msg import YoloResults
from r2_interfaces.msg import Xywh
from r2_interfaces.msg import XyXy


class SiloDetectionNode(Node):
    def __init__(self):
        super().__init__('silo_detection_node')
        self.get_logger().info("Silo Detection Node has been started")
        
        self.create_subscription(
            YoloResults,
            'yolo_results',
            self.yolo_results_callback,
            10
        )
        
        self.cmd_vel_pub = self.create_publisher(
            Twist, 
            'cmd_vel', 
            10
        )
        
        self.class_ids_list = []
        self.xyxys_list = []
        
        self.silos = []
        self.balls = []

    def yolo_results_callback(self, msg):
        self.get_logger().info("Yolo Results Received")
        
        self.class_ids_list.clear()
        self.xyxys_list.clear()
        
        self.class_ids_list.extend(msg.class_ids)
        for xyxy in msg.xyxy:
            self.xyxys_list.append([xyxy.tl_x, xyxy.tl_y, xyxy.br_x, xyxy.br_y])
        
        self.get_logger().info(f"Class IDs: {self.class_ids_list}")
        self.get_logger().info(f"XyXys: {self.xyxys_list}")
        
        self.silos = [self.xyxys_list[i] for i in range(len(self.class_ids_list)) if self.class_ids_list[i] == 3]
        self.balls = [(self.class_ids_list[i], self.xyxys_list[i]) for i in range(len(self.class_ids_list)) if self.class_ids_list[i] in [0, 1, 2]]

        self.get_logger().info(f"Silos: {self.silos}")
        self.get_logger().info(f"Balls: {self.balls}")

        if len(self.silos) >= 5:
            self.stop_robot()
            self.count_balls_in_silos()
        else:
            self.sweep_for_silos()

    def count_balls_in_silos(self):
        ball_counts = [0] * len(self.silos)
        blue_ball_counts = [0] * len(self.silos)
        red_ball_counts = [0] * len(self.silos)
        ball_colors = [[] for _ in range(len(self.silos))]
        color_map = {0: 'Blue', 1: 'Purple', 2: 'Red'}

        for ball_class, ball in self.balls:
            ball_center_x = (ball[0] + ball[2]) / 2
            ball_center_y = (ball[1] + ball[3]) / 2
            for i, silo in enumerate(self.silos):
                if (silo[0] <= ball_center_x <= silo[2]) and (silo[1] <= ball_center_y <= silo[3]):
                    if ball_counts[i] < 3:
                        ball_counts[i] += 1
                        ball_colors[i].append(color_map[ball_class])
                        if ball_class == 0:  # Blue ball
                            blue_ball_counts[i] += 1
                        elif ball_class == 2:  # Red ball
                            red_ball_counts[i] += 1

        # Save the results to a file
        with open("silo_ball_counts.txt", "w") as file:
            for i, count in enumerate(ball_counts):
                file.write(f"Silo {i + 1} contains {count} balls: {', '.join(ball_colors[i])}\n")

        for i, count in enumerate(ball_counts):
            self.get_logger().info(f"Silo {i + 1} contains {count} balls: {', '.join(ball_colors[i])}")

        if sum(ball_counts) == 15:
            self.get_logger().info("All silos are full. Stopping the robot.")
            self.stop_robot()
        else:
            best_silo = determine_best_silo(blue_ball_counts, red_ball_counts)
            self.get_logger().info(f"Best silo to place the next blue ball: Silo {best_silo}")

    def sweep_for_silos(self):
        twist_msg = Twist()
        twist_msg.angular.z = 2.0  #increased for simulation as it is quite slow in simulation
        twist_msg.linear.x = 0.0
        
        self.cmd_vel_pub.publish(twist_msg)
        self.get_logger().info(f"Sweeping for silos: angular_z = {twist_msg.angular.z}")

    def stop_robot(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        
        self.cmd_vel_pub.publish(twist_msg)
        self.get_logger().info("Stopping the robot.")

def determine_best_silo(blue_balls, red_balls):
    best_silo = -1
    max_blue_balls = max(blue_balls)

    # Prioritize avoiding silos that already have 3 balls
    available_silos = [i for i in range(len(blue_balls)) if blue_balls[i] + red_balls[i] < 3]

    if not available_silos:
        return best_silo  # No available silos

    # Check for immediate winning scenario
    for i in available_silos:
        if blue_balls[i] == 2:
            return i + 1

    # Prevent opponent from winning
    for i in available_silos:
        if red_balls[i] == 2:
            return i + 1

    # Check for potential winning scenario
    for i in available_silos:
        if blue_balls[i] == 1:
            return i + 1

    # Fallback to the nearest available silo to the center (index 2)
    min_distance = float('inf')
    for i in available_silos:
        distance = abs(i - 2)
        if distance < min_distance:
            min_distance = distance
            best_silo = i

    return best_silo + 1 if best_silo != -1 else -1

def main(args=None):
    rclpy.init(args=args)
    silo_detection_node = SiloDetectionNode()
    rclpy.spin(silo_detection_node)
    silo_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
