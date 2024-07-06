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
import time
import numpy as np
from std_msgs.msg import Bool

from ultralytics import YOLO

from geometry_msgs.msg import Twist
from r2_interfaces.msg import YoloResults
from r2_interfaces.srv import BestSilo
from std_msgs.msg import Int32
from rcl_interfaces.msg import SetParametersResult


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
        
        # self.cmd_vel_pub = self.create_publisher(
        #     Twist, 
        #     'cmd_vel', 
        #     10
        # )
        
    
        
        self.valid_silo_pub = self.create_publisher(Bool, "valid_silo", 10)
        
        
        self.declare_parameter('logging', False)
        self.logging = self.get_parameter('logging').value
        
        # Declare the parameter for the team ball color
        self.declare_parameter('our_ball_color', 'b')
        self.our_ball_color = self.get_parameter('our_ball_color').value
        
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        # Set opponent's ball color
        self.opponent_ball_color = 'r' if self.our_ball_color == 'b' else 'b'
        
        # Arrays to store values from the message
        self.class_ids_list = []
        self.contour_areas_list = []
        self.differences_list = []
        self.confidences_list = []  
        self.tracking_ids_list = []
        self.xyxys_list = []
        self.xywhs_list = []
        self.valid_silo = True

              
        # Indices for balls and silos
        self.blue_ball_indices = []
        self.red_ball_indices = []
        self.silo_indices = []
              
        # Dictionary to store the balls in each silo
        self.balls_in_silos = {        #Lowest ball is at index 0(first in the list)
            "s1": [],
            "s2": [],
            "s3": [],
            "s4": [],
            "s5": []
        }

        self.active = False
        #Silo Pt Difference: [No of silo pts of ours - No of silo pts of opponent]
        self.silo_pts_diff = 0
        
        print('init done')
        
        
    def parameters_callback(self, params):
        for param in params:
            param_name = param.name
            param_value = param.value
            setattr(self, param_name, param_value)
        return SetParametersResult(successful=True)



        
    def yolo_results_callback(self, msg):
        
        if self.active:
                
            if self.logging:
                self.get_logger().info("Yolo Results Received")
            
            # Clear Previous Values
            self.class_ids_list.clear()
            self.contour_areas_list.clear()
            self.differences_list.clear()
            self.confidences_list.clear()
            self.tracking_ids_list.clear()
            self.xyxys_list.clear()
            self.xywhs_list.clear()
            
            # Store the values from the message
            self.class_ids_list.extend(msg.class_ids)
            self.contour_areas_list.extend(msg.contour_area)
            self.differences_list.extend(msg.differences)
            self.confidences_list.extend(msg.confidence)
            self.tracking_ids_list.extend(msg.tracking_id)
            
            # Loop through the XyXy and Xywh data in the message
            for xyxy, xywh in zip(msg.xyxy, msg.xywh):
                # Append the data to the lists
                self.xyxys_list.append([xyxy.tl_x, xyxy.tl_y, xyxy.br_x, xyxy.br_y])
                self.xywhs_list.append([xywh.center_x, xywh.center_y, xywh.width, xywh.height])
        
        
            # Log the received values       
            if self.logging:
                self.get_logger().info(f"Class IDs: {self.class_ids_list}")
                # self.get_logger().info(f"Contour Areas: {self.contour_areas_list}")
                # self.get_logger().info(f"Differences: {self.differences_list}")
                # self.get_logger().info(f"Confidences: {self.confidences_list}")
                # self.get_logger().info(f"Tracking IDs: {self.tracking_ids_list}")
                # self.get_logger().info(f"XyXys: {self.xyxys_list}")
                # self.get_logger().info(f"Xywhs: {self.xywhs_list}")
            

            # Call the function to find balls in silos
            self.find_balls_in_silos()


    def find_balls_in_silos(self):
        # Indices for balls and silos
        self.silo_indices = [i for i, class_id in enumerate(self.class_ids_list) if class_id == 3]
        
        # update best silo only when all silos are visible
        if len(self.silo_indices) < 1:
            return
        
        self.blue_ball_indices = [i for i, class_id in enumerate(self.class_ids_list) if class_id == 0]
        self.red_ball_indices = [i for i, class_id in enumerate(self.class_ids_list) if class_id == 2]
        
        if self.logging:
            print("Blue Ball Indices: ", self.blue_ball_indices)
            print("Red Ball Indices: ", self.red_ball_indices)
            print("Silo Indices: ", self.silo_indices)        
        
        # Sort the silos by their x-coordinate (left to right)
        self.silo_indices.sort(key=lambda i: self.xyxys_list[i][0])
        
        # Reset the balls_in_silos dictionary
        self.balls_in_silos = {key: [] for key in self.balls_in_silos}
        
        # Loop over each silo and check if balls are within its bounds
        for idx, silo in enumerate(self.silo_indices):
            silo_coords = self.xyxys_list[silo]
            balls_in_silo = []

            for ball_idx in self.blue_ball_indices:
                ball_center_x, ball_center_y = self.xywhs_list[ball_idx][:2]
                if (silo_coords[0] <= ball_center_x <= silo_coords[2]) and (silo_coords[1] <= ball_center_y <= silo_coords[3]):
                    balls_in_silo.append(('b', ball_center_y))

            for ball_idx in self.red_ball_indices:
                ball_center_x, ball_center_y = self.xywhs_list[ball_idx][:2]
                if (silo_coords[0] <= ball_center_x <= silo_coords[2]) and (silo_coords[1] <= ball_center_y <= silo_coords[3]):
                    balls_in_silo.append(('r', ball_center_y))

            # Sort balls by their center_y coordinate (bottom to top)
            balls_in_silo.sort(key=lambda x: x[1], reverse=True)
            self.balls_in_silos[f"s{idx+1}"] = [ball[0] for ball in balls_in_silo]
            
        self.get_silo_pts_diff()
        
        if self.logging:
            self.get_logger().info(f"Balls in Silos: {self.balls_in_silos}")
            
        valid_silo = self.silo_decision()
        
        

        Bool_msg = Bool()
        Bool_msg.data = valid_silo
        self.valid_silo_pub.publish(Bool_msg)
        # self.active = False
        
        
    def get_silo_pts_diff(self):
        
        if self.logging:
            self.get_logger().info("Getting Silo Points Difference")
            
        our_points = 0
        opponent_points = 0
        
        # Check for the points in the silos
        for balls in self.balls_in_silos.values():
            if len(balls) == 3 and balls[-1] == self.our_ball_color and balls.count(self.our_ball_color) >= 2:      #Count of our balls is 2 or more
                our_points += 1
            elif len(balls) == 3 and balls[-1] == self.opponent_ball_color and balls.count(self.opponent_ball_color) >= 2:    #Count of opponent balls is 2 or more
                opponent_points += 1
        
        self.silo_pts_diff = our_points - opponent_points       #Silo Pt Difference: [No of silo pts of ours - No of silo pts of opponent]
        
        if self.logging:
            self.get_logger().info(f"Silo Points Difference: {self.silo_pts_diff}")
            self.get_logger().info(f"Our Points: {our_points}")
            self.get_logger().info(f"Opponent Points: {opponent_points}")
            
               
    def silo_decision(self):
        
        if self.logging:
            self.get_logger().info("Deciding the Best Silo")

        # Function to prioritize the silos based on the conditions
        def evaluate_silo(silo, balls):
            our_ball_count = balls.count(self.our_ball_color)
            opponent_ball_count = balls.count(self.opponent_ball_color)
            is_full = len(balls) == 3
            top_is_our_ball = balls[-1] == self.our_ball_color if balls else False

            #### CONDITION WHEN SILO IS FULL ####
            

            if (is_full==True):
                self.valid_silo = False
            else:
                self.valid_silo = True

            
            
            
        silo_scores = {silo: evaluate_silo(silo, balls) for silo, balls in self.balls_in_silos.items()}
        
        # Sort silos by their score and then by their number if scores are equal
        sorted_silos = sorted(silo_scores.keys(), key=lambda s: (silo_scores[s], s))
        
        # Log the scores and sorted silos
        if self.logging:
            self.get_logger().info(f"Silo scores: {silo_scores}")
            self.get_logger().info(f"Sorted silos: {sorted_silos}")

        # Return the silo with the highest priority (lowest score)
        return self.valid_silo
        

    # def sweep_for_silos(self):
    #     twist_msg = Twist()
    #     twist_msg.angular.z = 2.0  #increased for simulation as it is quite slow in simulation
    #     twist_msg.linear.x = 0.0
        
    #     self.cmd_vel_pub.publish(twist_msg)
        
    #     if self.logging:
    #         self.get_logger().info(f"Sweeping for silos: angular_z = {twist_msg.angular.z}")

    # def stop_robot(self):
    #     twist_msg = Twist()
    #     twist_msg.linear.x = 0.0
    #     twist_msg.angular.z = 0.0
        
    #     self.cmd_vel_pub.publish(twist_msg)
        
    #     if self.logging:
    #         self.get_logger().info("Stopping the robot.")


def main(args=None):
    rclpy.init(args=args)
    silo_detection_node = SiloDetectionNode()
    rclpy.spin(silo_detection_node)
    silo_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
