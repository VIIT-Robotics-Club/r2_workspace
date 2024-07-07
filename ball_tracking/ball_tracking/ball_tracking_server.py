#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from rcl_interfaces.msg import SetParametersResult
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist
from r2_interfaces.msg import YoloResults
from std_msgs.msg import Bool
from std_msgs.msg import Int32

import time
class BallTrackingNode(Node):
    def __init__(self):
        super().__init__('ball_tracking_node')
        self.get_logger().info("Ball Tracking Node has been started")
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('desired_contour_area', 364000), #change according to camera position
                ('linear_kp', 0.50),
                ('linear_ki', 0.000),
                ('linear_kd', 0.0),
                ('angular_kp', 0.2),
                ('angular_ki', 0.00000),
                ('angular_kd', 1.8),
                ('max_linear_speed', 2.0),
                ('max_integral', 100.0),
                ('max_angular_speed', 1.0),
                ('contour_area_threshold', 0000),
                ('difference_threshold', 600),
                ('blueSide', True),
                ('logging', True),
                ('contour_area_fluctuation_threshold', 6000),  # New parameter for fluctuation threshold
                # ('set_contour_area', 6000)  # New parameter for fluctuation threshold
            ]
        )
        
        self.desired_contour_area = self.get_parameter('desired_contour_area').value
        self.linear_kp = self.get_parameter('linear_kp').value
        self.linear_ki = self.get_parameter('linear_ki').value
        self.linear_kd = self.get_parameter('linear_kd').value
        self.angular_kp = self.get_parameter('angular_kp').value
        self.angular_ki = self.get_parameter('angular_ki').value
        self.angular_kd = self.get_parameter('angular_kd').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.max_integral = self.get_parameter('max_integral').value
        self.contour_area_threshold = self.get_parameter('contour_area_threshold').value
        self.difference_threshold = self.get_parameter('difference_threshold').value
        self.blueSide = self.get_parameter('blueSide').value
        self.logging = self.get_parameter("logging").value
        self.contour_area_fluctuation_threshold = self.get_parameter('contour_area_fluctuation_threshold').value  # New parameter
        # self.set_error_area = self.get_parameter('set_error_area').value  # New parameter

        self.linear_error_sum = 0.0
        self.linearX_last_error = 0.0
        self.angular_error_sum = 0.0
        self.angular_last_error = 0.0
        self.tracking_our_ball = False
        self.difference_error = 0.0
        self.contour_area_error = 0.0
        self.previous_contour_area = None  # New attribute to store the previous contour area
        self.contour_area=0
        self.publisher = self.create_publisher(Bool, 'status', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'nav_vel', 10)
        self.cmd_vel_pub_count = 0
        self.counter=0
        self.class_ids_list = []
        self.contour_areas_list = []
        self.contour_areas_list_2 = []
        self.differences_list = []
        self.confidences_list = []
        self.tracking_ids_list = []
        self.xyxys_list = []
        self.xywhs_list = []
        self.wall_dist=10
        self.wall_dist_align=80
        self.int_error_diff = 0
        self.closest_our_ball = {
            'class_id': None,
            'contour_area': None,
            'difference': None,
            'confidence': None,
            'tracking_id': None,
            'xyxy': None,
            'xywh': None
        }
        self.our_id = -1
        self.current_contour_area=0
        self.opp_id = -1
        self.reached = True
        self.last_seen_direction = None
        self.active = False
        self.luna_rf=0
        self.int_error_area=0
        self.enableEdgeCases = False
        # self.center_aligned = False 
        self.target_y_dist = 200

        self.create_service(SetBool, "/ball_tracking_srv", self.ball_tracking_callback)
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.create_subscription(YoloResults, 'yolo_results', self.yolo_results_callback, 10)
        self.create_subscription(Bool, 'halt', self.halt_callback, 10)
        # self.create_timer()
        self.luna_rf_subscriber = self.create_subscription(         #Right Front
            Int32,   'luna_rf', self.luna_rf_callback, 10
        )
        
        
        self.luna_rb_subscriber = self.create_subscription(         #Right Back           s
            Int32, 'luna_rb', self.luna_rb_callback, 10
        ) 
        
        self.luna_rf_subscriber = self.create_subscription(         #Right Front
            Int32,   'luna_lf', self.luna_lf_callback, 10
        )
        
        
        self.luna_rb_subscriber = self.create_subscription(         #Right Back           s
            Int32, 'luna_lb', self.luna_lb_callback, 10
        )        
        

    def halt_callback(self,msg:Bool):
        self.linear_error_sum = 0.0
        self.linearX_last_error = 0.0
        self.angular_error_sum = 0.0
        self.angular_last_error = 0.0
        self.tracking_our_ball = False
        self.difference_error = 0.0
        self.contour_area_error = 0.0
        self.previous_contour_area = None  # New attribute to store the previous contour area
        self.contour_area=0
        self.cmd_vel_pub_count = 0
        self.counter=0
        self.class_ids_list = []
        self.contour_areas_list = []
        self.contour_areas_list_2 = []
        self.differences_list = []
        self.confidences_list = []
        self.tracking_ids_list = []
        self.xyxys_list = []
        self.xywhs_list = []
        self.wall_dist=10
        self.wall_dist_align=80
        self.int_error_diff = 0
        self.closest_our_ball = {
            'class_id': None,
            'contour_area': None,
            'difference': None,
            'confidence': None,
            'tracking_id': None,
            'xyxy': None,
            'xywh': None
        }
        self.our_id = -1
        self.current_contour_area=0
        self.opp_id = -1
        self.reached = True
        self.last_seen_direction = None
        self.active = False
        self.luna_rf=0
        self.int_error_area=0
        self.enableEdgeCases = False
        # self.center_aligned = False 
        self.target_y_dist = 200
        
    def luna_lb_callback(self, msg: Int32):
        try:
            self.luna_lb = float(msg.data)
            # print(self.luna_lb)
        except Exception as e:
            print(f"Error in luna_lb_callback: {e}")

    def luna_lf_callback(self, msg: Int32):
        try:
            self.luna_lf = float(msg.data)
            # print(self.luna_lf)
        except Exception as e:
            print(f"Error in luna_lf_callback: {e}")

    def luna_rf_callback(self, msg: Int32):
        try:
            self.luna_rf = float(msg.data)
            # print(self.luna_rf)
        except Exception as e:
            print(f"Error in luna_rf_callback: {e}")

    def luna_rb_callback(self, msg: Int32):
        try:
            self.luna_rb = float(msg.data)
            # print(self.luna_rb)
        except Exception as e:
            print(f"Error in luna_rb_callback: {e}")
            


    def ball_tracking_callback(self, request, response):
        self.active = True  
        self.get_logger().info("Ball tracking is active")
        response.success = True
        response.message = "Ball tracking started"
        # self.center_aligned = False # Flag to activate center alignment during ball tracking using Lunas
        
        return response

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

            self.class_ids_list.clear()
            self.contour_areas_list.clear()
            self.contour_areas_list_2.clear()
            self.differences_list.clear()
            self.confidences_list.clear()
            self.tracking_ids_list.clear()
            self.xyxys_list.clear()
            self.xywhs_list.clear()
            self.current_contour_area=0

            self.class_ids_list.extend(msg.class_ids)
            self.contour_areas_list.extend(msg.contour_area)
            self.differences_list.extend(msg.differences)
            self.confidences_list.extend(msg.confidence)
            self.tracking_ids_list.extend(msg.tracking_id)


            for xyxy, xywh in zip(msg.xyxy, msg.xywh):
                self.xyxys_list.append([xyxy.tl_x, xyxy.tl_y, xyxy.br_x, xyxy.br_y])
                self.xywhs_list.append([xywh.center_x, xywh.center_y, xywh.width, xywh.height])


            def is_behind_other_ball(our_ball_coords, other_ball_coords):
                our_tl_x, our_tl_y, our_br_x, our_br_y = our_ball_coords
                other_tl_x, other_tl_y, other_br_x, other_br_y = other_ball_coords
                return (our_tl_x > other_tl_x and our_tl_y > other_tl_y and our_br_x < other_br_x and our_br_y < other_br_y)

            def is_inside_silo(our_ball_coords, silo_coords):
                our_tl_x, our_tl_y, our_br_x, our_br_y = our_ball_coords
                silo_tl_x, silo_tl_y, silo_br_x, silo_br_y = silo_coords
                return (our_tl_x >= silo_tl_x and our_tl_y >= silo_tl_y and our_br_x <= silo_br_x and our_br_y <= silo_br_y)

            # Update IDs based on ball color
            if self.blueSide == False:
                self.our_id = 2
                self.opp_id = 0
            elif self.blueSide:
                self.our_id = 0
                self.opp_id = 2

            our_ball_indices = [i for i, class_id in enumerate(self.class_ids_list) if class_id == self.our_id]
            silo_indices = [i for i, class_id in enumerate(self.class_ids_list) if class_id == 3]
            
            filtered_our_ball_indices = []
            for i in our_ball_indices:
                our_ball_coords = self.xyxys_list[i]
                behind_other_ball = False
                inside_silo = False
                # self.contour_area=self.xywhs_list[2]*self.xywhs_list[2]
                # self.contour_areas_list_2.extend(self.contour_area)

                


                for j, class_id in enumerate(self.class_ids_list):
                    if class_id in [1, self.opp_id]:  # Purple and Red ball or Purple or Blue Ball
                        other_ball_coords = self.xyxys_list[j]
                        if is_behind_other_ball(our_ball_coords, other_ball_coords):
                            behind_other_ball = True
                            break

                for k in silo_indices:
                    silo_coords = self.xyxys_list[k]
                    if is_inside_silo(our_ball_coords, silo_coords):
                        inside_silo = True
                        break

                if not behind_other_ball and not inside_silo:
                    filtered_our_ball_indices.append(i)

            if filtered_our_ball_indices:
                largest_contour_index = max(filtered_our_ball_indices, key=lambda i: self.contour_areas_list[i])
                devia = min(filtered_our_ball_indices, key=lambda i: self.differences_list[i])

                self.current_contour_area = self.contour_areas_list[largest_contour_index]

                # if self.previous_contour_area is None or abs(current_contour_area - self.previous_contour_area) < self.contour_area_fluctuation_threshold:
                #     self.previous_contour_area = current_contour_area
                # else:
                #     current_contour_area = self.previous_contour_area

                self.closest_our_ball = {
                    'class_id': self.class_ids_list[largest_contour_index],
                    'contour_area': self.contour_areas_list[largest_contour_index],
                    'difference': self.differences_list[largest_contour_index],
                    'confidence': self.confidences_list[largest_contour_index],
                    'tracking_id': self.tracking_ids_list[largest_contour_index] if self.tracking_ids_list else None,
                    'xyxy': self.xyxys_list[largest_contour_index],
                    'xywh': self.xywhs_list[largest_contour_index]
                }
                if self.logging:
                    self.get_logger().info(f"Closest our Ball: {self.closest_our_ball}")
                self.contour_area_error = self.desired_contour_area - self.closest_our_ball['contour_area']
                self.difference_error = self.closest_our_ball['difference']
                self.tracking_our_ball = True

                if self.difference_error < 0:
                    self.last_seen_direction = 'left'
                else:
                    self.last_seen_direction = 'right'

                
                self.move_robot()
            else:
                self.sweep_for_ball()

    def pid_controller(self, error, previous_error, int_error, ki, kd, dt, linear_kp):
        int_error = np.clip(int_error, -self.max_integral, self.max_integral)
        # self.get_logger().info("Intertial Error "+ str(int_error))
        control_action = linear_kp * error + ki * int_error + kd * ((error - previous_error))
        return control_action
    
    def move_afar(self, twist_msg):
        if self.luna_rf <= self.wall_dist_align:
            self.get_logger().info(f"luna {self.luna_rf}")

            # twist_msg.angular.z = 0.0
            # self.cmd_vel_pub.publish(twist_msg)
        # self.get_logger().info("aligned")

    # def move_to_center(self):
    #     twist = Twist()
    #     if not self.center_aligned :
    #         # Bot is on slope going downwards , time to align
    #         while (time.time() - curr_time <= 1.5):
    #             self.get_logger().info("Going Forwards")
    #             twist.linear.x = 2.0
    #             if self.blueSide: 
    #                 twist.linear.y = 1.0
    #             else:
    #                 twist.linear.y = -1.0
    #             self.cmd_vel_pub.publish(twist)

    #         if self.y_diff < 0 :
    #             self.get_logger().info("Reached Center Position")
    #             self.center_aligned = True

    #         # If Target Center postion is not reached aligning in Y direction
    #         right_avg = (self.luna_rf + self.luna_rb) / 2.0
    #         self.y_diff = right_avg - self.target_y_dist
    #         self.int_y_diff += self.y_diff
    #         print(self.y_diff)
    #         twist.linear.y = self.pid_controller(self.y_diff, self.prev_y_diff, self.int_y_diff, self.kp_linear_y, self.ki_linear_y, self.kd_linear_y, dt)
    #         self.prev_y_diff = self.y_diff
    #         twist.linear.y = max(min(twist.linear.y, self.linear_y_max), self.linear_y_min) 

    #         if not self.blueSide: # If Red side
    #             twist.linear.y = - twist.linear.y

    #         if self.logging:
    #             self.get_logger().info('Aligning in y : %f' % twist.linear.y) #Apply PID controller for angular Z
    #         self.cmd_vel_pub.publish(twist)
                        

    def move_robot(self):
        twist_msg = Twist()
        if self.enableEdgeCases and self.luna_rf >= self.wall_dist and self.luna_rf <= 34:
            self.get_logger().info(f"entered")
            twist_msg.linear.y = 0.5

            # self.move_afar(twist_msg)
        
        if self.closest_our_ball['class_id'] is not None:
            if self.logging:
                self.get_logger().info(f"Contour Area Error: {self.contour_area_error / 70000}")
                self.get_logger().info(f"Difference Error: {self.difference_error / 170}")
                
            # if self.previous_contour_area != None and abs(self.contour_area_error - self.previous_contour_area) < self.set_error_area :
            #     self.counter =self.counter+1
            #     if self.counter> 50 :
            #         self.counter=0
            #         twist_msg=Twist()
            #         twist_msg.linear.x = 0.0
            #         twist_msg.angular.z = 0.7
            #         self.cmd_vel_pub.publish(twist_msg)
            #         time.sleep(1.5)
            # self.previous_contour_area=self.contour_area_error        
            

            if  self.contour_area_error < self.contour_area_threshold and abs(self.difference_error) < self.difference_threshold:
                twist_msg = Twist()
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
                self.cmd_vel_pub.publish(twist_msg)
                self.active = False

                bool_msg = Bool()
                bool_msg.data = True
                self.publisher.publish(bool_msg)
                self.get_logger().info("Reached the ball. Stopping the robot.")

                self.tracking_our_ball = False
                self.closest_our_ball = {
                    'class_id': None,
                    'contour_area': None,
                    'difference': None,
                    'confidence': None,
                    'tracking_id': None,
                    'xyxy': None,
                    'xywh': None
                }
                return

            # twist_msg = Twist()
            self.int_error_area += self.contour_area_error/14000000
            self.int_error_diff += self.difference_error / 17000

            # self.int_error_diff = np.clip(self.int_error_diff, -self.max_integral, self.max_integral)
            twist_msg.linear.x = float(self.pid_controller(self.contour_area_error / 215000, self.linearX_last_error, self.int_error_area, self.linear_ki, self.linear_kd, 0.1, self.linear_kp))
            twist_msg.angular.z = -float(self.pid_controller((self.difference_error / 170), self.angular_last_error, self.int_error_diff, self.angular_ki, self.angular_kd, 0.1, self.angular_kp))
            # twist_msg.linear.y = float(self.pid_controller((self.difference_error / 170), self.angular_last_error, self.int_error_diff, self.angular_ki, self.angular_kd, 0.1, self.angular_kp))

            twist_msg.linear.x = max(min(twist_msg.linear.x, self.max_linear_speed), -self.max_linear_speed)
            twist_msg.angular.z = max(min(twist_msg.angular.z, self.max_angular_speed), -self.max_angular_speed)
            # twist_msg.linear.y = max(min(twist_msg.angular.y, self.max_angular_speed), -self.max_angular_speed)
            if self.current_contour_area <30000:
                cutoff_lin_x=twist_msg.linear.x/4
                cutoff_ang_z=twist_msg.angular.z/3
                twist_msg.angular.z=twist_msg.angular.z  +cutoff_ang_z
                twist_msg.linear.x=abs(cutoff_ang_z) + twist_msg.linear.x +cutoff_lin_x

            self.linearX_last_error = self.contour_area_error / 120000
            self.angular_last_error = self.difference_error / 170
            # self.cmd_vel_pub_count += 1
            self.cmd_vel_pub.publish(twist_msg)
                # self.cmd_vel_pub_count = 0

            if self.logging:
                self.get_logger().info(f"Publishing cmd_vel: linear_x = {twist_msg.linear.x}, angular_z = {twist_msg.angular.z}")

    def sweep_for_ball(self):
        twist_msg = Twist()
        if self.last_seen_direction == 'right':
            twist_msg.angular.z = -1.0
        else:
            twist_msg.angular.z = 1.0
        twist_msg.linear.x = 0.0
        
        self.cmd_vel_pub.publish(twist_msg)
        if self.logging:
            self.get_logger().info(f"Sweeping for ball: angular_z = {twist_msg.angular.z}")
    
def main(args=None):
    rclpy.init(args=args)
    ball_tracking_node = BallTrackingNode()
    rclpy.spin(ball_tracking_node)
    ball_tracking_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
