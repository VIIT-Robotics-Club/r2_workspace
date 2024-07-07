from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32
from rcl_interfaces.msg import SetParametersResult
from std_srvs.srv import SetBool
from threading import Thread
import rclpy
import time
from r2_interfaces.srv import SiloToGo, BestSilo
import queue

class Scatter(Node):
    
    def __init__(self):
        super().__init__("scatter_balls")
        
        self.luna_fl_subscriber = self.create_subscription(         #Front Left
            Int32, 'luna_fl', self.luna_fl_callback, 10
        )
        self.luna_fr_subscriber = self.create_subscription(         #Front Right
            Int32, 'luna_fr', self.luna_fr_callback, 10
        )
        self.luna_rf_subscriber = self.create_subscription(         #Right Front
            Int32,  'luna_rf', self.luna_rf_callback, 10
        ) 
        self.luna_rb_subscriber = self.create_subscription(         #Right Back           
            Int32, 'luna_rb', self.luna_rb_callback, 10
        ) 
        self.luna_lf_subscriber = self.create_subscription(         #Right Front
            Int32,  'luna_lf', self.luna_lf_callback, 10
        )
        self.luna_lb_subscriber = self.create_subscription(         #Right Back           s
            Int32, 'luna_lb', self.luna_lb_callback, 10
        )           
        self.cmd_vel_pub = self.create_publisher( Twist, 'nav_vel', 10)
        self.create_service(SetBool,"scatter_balls_srv",self.scatter_service_callback)
        
        self.prev_ang_error = 0
        self.declare_parameters(
            namespace='',
            parameters=[
                ('linear_x_max', 2.0),
                ('linear_y_max', 2.0),
                ('linear_x_min', -2.0),
                ('linear_y_min', -2.0),
                ('angular_z_max', 2.0),
                ('angular_z_min', -2.0),
                ('kp_linear_x', 1.8),
                ('ki_linear_x', 0.05),
                ('kd_linear_x', 0.03),
                ('kp_linear_y', 0.7),
                ('ki_linear_y', 0.05),
                ('kd_linear_y', 0.03),
                ('kp_angular', 0.1),
                ('ki_angular', 0.00), # 0.0028
                ('kd_angular', 0.03),          
                ('capture_x', 1.5),
                ('capture_y', 1.5),    
                ('silo_1_x', 0.090),
                ('silo_1_y', 3.120),
                ('silo_2_x', 0.090),
                ('silo_2_y', 2.400),
                ('silo_3_x', 0.090),
                ('silo_3_y', 1.660),
                ('silo_4_x', 0.090),
                ('silo_4_y', 0.980),
                ('silo_5_x', 0.090),
                ('silo_5_y', 0.250),    
                ('logging', True),
                ('blue_side', True)
                ]
        )
        
        self.linear_x_max = self.get_parameter('linear_x_max').value
        self.linear_y_max = self.get_parameter('linear_y_max').value
        self.linear_x_min = self.get_parameter('linear_x_min').value
        self.linear_y_min = self.get_parameter('linear_y_min').value
        self.angular_z_max = self.get_parameter('angular_z_max').value
        self.angular_z_min = self.get_parameter('angular_z_min').value
        self.kp_linear_x = self.get_parameter('kp_linear_x').value
        self.ki_linear_x = self.get_parameter('ki_linear_x').value
        self.kd_linear_x = self.get_parameter('kd_linear_x').value
        self.kp_linear_y = self.get_parameter('kp_linear_y').value
        self.ki_linear_y = self.get_parameter('ki_linear_y').value
        self.kd_linear_y = self.get_parameter('kd_linear_y').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.ki_angular = self.get_parameter('ki_angular').value
        self.kd_angular = self.get_parameter('kd_angular').value
        self.logging = self.get_parameter("logging").value
        self.blue_side = self.get_parameter('blue_side').value
        self.target_y_dist = 100 # cm   
        self.target_x_dist = 250 # cm    
        self.linear_x_time = 3.5 # seconds
        self.push_time = 1.0
        self.active = False
        self.y_diff = 0.0
        self.y_err=0.0
        self.loop_once = 0
        # Initialize the luna data
        self.luna_rb = 0.00     #Right Back
        self.luna_rf = 0.00     #Right Front
        self.luna_fl = 0.00     #Front Left
        self.luna_fr = 0.00     #Front Right
        self.luna_lf = 0.00     #Left Front
        self.luna_lb = 0.00     #Left Back
        
        # Set initial integrals to zero
        self.int_error_linear_x = 0.0
        self.int_error_linear_y = 0.0
        self.int_error_angular_z = 0.0

        self.correct_angle = None
        self.yaw_aligned = None
        self.lin_y_aligned = None
        self.lin_x_aligned = None
        self.x_push = None
        self.lin_y_diff = 1 # Diff between two lunas on same side
        self.left_avg = None
        self.y_push_dist = 250.0

        self.int_x_diff = 0
        self.int_y_diff = 0
        self.prev_x_diff = 0
        self.prev_y_diff = 0
        self.int_y_err = 0
        self.prev_y_err = 0



    def pid_controller(self, error, previous_error, int_error, kp, ki, kd, dt):
        control_action = kp * error + ki * int_error + kd * ((error - previous_error) / dt)
        return control_action
    
    def luna_fl_callback(self, msg: Int32):
        try:
            self.luna_fl = float(msg.data)
        except Exception as e:
            self.get_logger().error(f"Error in luna_fl_callback: {e}")
        
        
    def luna_fr_callback(self, msg: Int32):
        try:
            self.luna_fr = float(msg.data)
            # print(self.luna_fr)
        except Exception as e:
            print(f"Error in luna_fr_callback: {e}")
            
    def luna_lb_callback(self, msg: Int32):
        try:
            self.luna_lb = float(msg.data)
            # print(self.luna_lf)
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

        if self.active:
            self.scatter()

    def luna_rb_callback(self, msg: Int32):
        try:
            self.luna_rb = float(msg.data)
            # print(self.luna_rb)
        except Exception as e:
            print(f"Error in luna_rb_callback: {e}")
       
    def scatter(self):
        if not self.correct_angle:
            self.rotate_90()  
        elif not self.yaw_aligned :
            self.yaw_and_linear_y_fix()
        elif not self.lin_x_aligned :
            self.linear_x_fix()
        elif not self.x_push:
            self.push()
         
    def scatter_service_callback(self,request,response):
        self.get_logger().info("Scatter Balls service has been started")
        response.success = True
        response.message = "Scattering started"

        self.active = True # This will call scatter function now
        self.correct_angle = False
        self.yaw_aligned = False
        self.lin_y_aligned = False
        self.lin_x_aligned = False
        self.x_push = False

        return response
            
            
    def rotate_90(self):
        if self.logging:  
            self.get_logger().info('Rotation started. ')
        
        twist = Twist()
        curr_time = time.time()
        while (time.time() - curr_time <= 0.8):
            # If our side side colour is blue
            if self.blue_side :
                twist.angular.z = -2.0
            # If our side side colour is Red
            else :
                twist.angular.z = 2.0
            self.cmd_vel_pub.publish(twist) 
            self.get_logger().info('Angular z: %f' % twist.angular.z) #Apply PID controller for angular z

        self.correct_angle = True
        if self.logging:  
            self.get_logger().info('Rotation ended. ')
            
    def yaw_and_linear_y_fix(self):
        dt = 0.2  # Assuming fixed sample rate of 10 Hz
        twist = Twist()

        # Blue Side
        if self.blue_side:
            ang_error = self.luna_lf - self.luna_lb
            if self.y_diff < 0:  # Or you can use abs(y_diff) <= 3
                if abs(ang_error) <= self.lin_y_diff:
                    # self.yaw_and_lin_y_aligned = True
                    self.get_logger().info('Robot Yaw aligned for scatter')
                    self.yaw_aligned = True
                    self.get_logger().info('Linear Y alignment Successful for Scatter')

                # Yaw Alignment 
                self.int_error_angular_z += ang_error
                twist.angular.z = self.pid_controller(ang_error, self.prev_ang_error, self.int_error_angular_z, self.kp_angular, self.ki_angular, self.kd_angular, dt)
                self.prev_ang_error = ang_error

                twist.angular.z = max(min(twist.angular.z, self.angular_z_max), self.angular_z_min)
                if ang_error < 0: 
                    twist.angular.z = -abs(twist.angular.z)  # Rotate Clockwise
                else:
                    twist.angular.z = abs(twist.angular.z) 
                if self.logging:
                    self.get_logger().info('Angular z: %f' % twist.angular.z) #Apply PID controller for angular z
            else:
                # Linear Y fix
                left_avg = (self.luna_lf + self.luna_lb) / 2.0
                self.y_diff = left_avg - self.target_y_dist
                self.int_y_diff += self.y_diff
                print(self.y_diff)
                twist.linear.y = self.pid_controller(self.y_diff, self.prev_y_diff, self.int_y_diff, self.kp_linear_y, self.ki_linear_y, self.kd_linear_y, dt)
                self.prev_y_diff = self.y_diff
                twist.linear.y = max(min(twist.linear.y, self.linear_y_max), self.linear_y_min) # Consider negative value of linear y for Red side 
                if self.logging:
                    self.get_logger().info('Linear Y: %f' % twist.linear.y) #Apply PID controller for angular Z
   
        # Red Side 
        else:
            ang_error = self.luna_rf - self.luna_rb
            if self.y_diff < 0:  # Or you can use abs(y_diff) <= 3
                if abs(ang_error) <= self.lin_y_diff :
                    # self.yaw_and_lin_y_aligned = True
                    self.get_logger().info('Robot Yaw aligned for scatter')
                    self.yaw_aligned = True
                    self.get_logger().info('Linear Y alignment Successful for Scatter')

                # Yaw Alignment 
                self.int_error_angular_z += ang_error
                twist.angular.z = self.pid_controller(ang_error, self.prev_ang_error, self.int_error_angular_z, self.kp_angular, self.ki_angular, self.kd_angular, dt)
                self.prev_ang_error = ang_error

                twist.angular.z = max(min(twist.angular.z, self.angular_z_max), self.angular_z_min)
                if ang_error < 0: 
                    twist.angular.z = -abs(twist.angular.z)  # Rotate Clockwise
                else:
                    twist.angular.z = abs(twist.angular.z) 
                if self.logging:
                    self.get_logger().info('Angular z: %f' % twist.angular.z) #Apply PID controller for angular z

        
            
            # Linear Y fix
            right_avg = (self.luna_rf + self.luna_rb) / 2.0
            self.y_diff = right_avg - self.target_y_dist
            self.int_y_diff += self.y_diff
            print(self.y_diff)
            twist.linear.y = self.pid_controller(self.y_diff, self.prev_y_diff, self.int_y_diff, self.kp_linear_y, self.ki_linear_y, self.kd_linear_y, dt)
            self.prev_y_diff = self.y_diff
            twist.linear.y = -max(min(twist.linear.y, self.linear_y_max), self.linear_y_min) # Consider negative value of linear y for Red side 
            if self.logging:
                self.get_logger().info('Linear Y: %f' % twist.linear.y) #Apply PID controller for angular Z
   
            # Publish Velocity
        self.cmd_vel_pub.publish(twist)
        
    def linear_x_fix(self):
        dt = 0.2  # Assuming fixed sample rate of 10 Hz
        twist = Twist()

        curr_time = time.time()
        if self.loop_once == 0:
            while (time.time() - curr_time <= self.linear_x_time):
                self.get_logger().info("Going Forwards")
                twist.linear.x = 2.0
                self.cmd_vel_pub.publish(twist)
            self.loop_once = 1

        ang_error = self.luna_rf - self.luna_rb
        if self.y_diff < 0:  # Or you can use abs(y_diff) <= 3
            if abs(ang_error) <= self.lin_y_diff :
                # self.yaw_and_lin_y_aligned = True
                self.get_logger().info('Robot Yaw aligned for scatter')
                self.yaw_aligned = True
                self.get_logger().info('Linear Y alignment Successful for Scatter')

        # Yaw Alignment 
        # self.int_error_angular_z += ang_error
        # twist.angular.z = self.pid_controller(ang_error, self.prev_ang_error, self.int_error_angular_z, self.kp_angular, self.ki_angular, self.kd_angular, dt)
        # self.prev_ang_error = ang_error

        # twist.angular.z = max(min(twist.angular.z, self.angular_z_max), self.angular_z_min)
        # if ang_error < 0: 
        #     twist.angular.z = -abs(twist.angular.z)  # Rotate Clockwise
        # else:
        #     twist.angular.z = abs(twist.angular.z) 
        # if self.logging:
        #     self.get_logger().info('Angular z: %f' % twist.angular.z) #Apply PID controller for angular z

        
        # # Linear Y fix
        # right_avg = (self.luna_rf + self.luna_rb) / 2.0
        # self.y_diff = right_avg - self.target_y_dist
        # self.int_y_diff += self.y_diff
        # print(self.y_diff)
        # twist.linear.y = self.pid_controller(self.y_diff, self.prev_y_diff, self.int_y_diff, self.kp_linear_y, self.ki_linear_y, self.kd_linear_y, dt)
        # self.prev_y_diff = self.y_diff
        # twist.linear.y = -max(min(twist.linear.y, self.linear_y_max), self.linear_y_min) # Consider negative value of linear y for Red side 
        # if self.logging:
        #     self.get_logger().info('Linear Y: %f' % twist.linear.y) #Apply PID controller for angular Z

        # Linear X Fix
        front_avg = (self.luna_fl + self.luna_fr)/ 2.0
        if front_avg < self.target_x_dist :
            self.lin_x_aligned = True
            self.get_logger().info('Linear X alignment Successful for Scatter')
        else:
            x_diff = self.luna_fl - self.target_x_dist
            self.int_x_diff += x_diff
            twist.linear.x = self.pid_controller(x_diff, self.prev_x_diff, self.int_x_diff, self.kp_linear_x, self.ki_linear_x, self.kd_linear_x, dt)
            self.prev_x_diff = x_diff
            twist.linear.x = max(min(twist.linear.x, self.linear_x_max), self.linear_x_min) # Consider negative value of linear y for Red side 
            if self.logging:
                self.get_logger().info('Linear X: %f' % twist.linear.x) #Apply PID controller for angular z

        # if self.blue_side:

        # else:
            
        # Publish Velocity
        self.cmd_vel_pub.publish(twist)
                    
    def push(self):
        if not self.x_push:
            dt = 0.2  # Assuming fixed sample rate of 10 Hz
            self.get_logger().info("Entered Push")
            twist = Twist()
            print(self.y_err)

            if self.y_err >= 0.0: 
            # Linear Y fix for Blue
                self.left_avg = (self.luna_lf + self.luna_lb) / 2.0
                self.y_err =  self.y_push_dist - self.left_avg
                self.int_y_err += self.y_err
                print(self.y_err)
                twist.linear.y = self.pid_controller(self.y_err, self.prev_y_err, self.int_y_err, self.kp_linear_y, self.ki_linear_y, self.kd_linear_y, dt)
                self.prev_y_err = self.y_err
                twist.linear.y = max(min(twist.linear.y, self.linear_y_max), self.linear_y_min) # Consider negative value of linear y for Red side 
                if self.logging:
                    self.get_logger().info('Linear Y: %f' % twist.linear.y) #Apply PID controller for angular Z

                self.cmd_vel_pub.publish(twist)
            else:    
                twist.linear.y = 0.0
                self.cmd_vel_pub.publish(twist)
                self.x_push = True
                self.get_logger().info("Pushing Done")
            # curr_time = time.time()
            # while (time.time() - curr_time <= self.push_time):
            #     print(time.time() - curr_time)
            #     # self.get_logger().info("Pushing")
            #     if self.blue_side :
            #         twist.linear.y = -2.0
            #     else:
            #         twist.linear.y = 2.0
            #     self.cmd_vel_pub.publish(twist)

            # # # Pushing Balls done
            # twist.linear.y = 0.0
            # self.cmd_vel_pub.publish(twist)
            # self.get_logger().info("Pushing Done")
            # self.x_push = True

            
def main(args=None):
    rclpy.init(args=args)
    node = Scatter()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()