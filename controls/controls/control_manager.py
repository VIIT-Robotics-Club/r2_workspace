from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from threading import Thread
import rclpy
import time
from r2_interfaces.srv import SiloToGo

class ControlManager(Node):
    
    def __init__(self):
        super().__init__('control_manager')
        self.get_logger().info("control manager initialized")
        self.create_subscription(Twist,'line/nav_vel',self.line_follow_vel_callback, 10)
        self.create_subscription(Twist,'ball/nav_vel',self.ball_vel_callback, 10)
        self.create_subscription(Twist,'luna/nav_vel',self.luna_vel_callback, 10)
        self.create_subscription(Twist,'silo/nav_vel',self.silo_vel_callback, 10)
        self.create_subscription(Twist,'rnm/nav_vel',self.rnm_vel_callback, 10)
        self.create_subscription(Bool,'status',self.status_callback, 10)
        
        self.line_follower_client = self.create_client(SetBool,"lf_srv")
        while not self.line_follower_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().info('lf_srv Service not available, waiting again...')
            
        self.ball_tracking_client = self.create_client(SetBool ,"ball_tracking_srv")
        
        while not self.ball_tracking_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().info('ball_tracking_srv Service not available, waiting again...')
        
        self.silo_tracking_client = self.create_client(SetBool ,"silo_tracking_srv")
        while not self.silo_tracking_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().info('silo_tracking_srv Service not available, waiting again...')
            
        self.luna_align_client = self.create_client(SiloToGo ,"luna_align_srv")
        while not self.luna_align_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().info('luna_align_srv Service not available, waiting again...')
            
        self.rnm_client = self.create_client(SetBool ,"rnm_srv")
        # while not self.rnm_client.wait_for_service(timeout_sec=0.2):
        #     self.get_logger().info('Service not available, waiting again...')
        
            
        
        self.vel_pub = self.create_publisher(Twist,"nav_vel",10)

        self.lastStatus = True
        self.response_recvd = False
        self.index = -1
        
    def line_follow_vel_callback(self,msg: Twist):
        if(self.index ==  -1):
            self.vel_pub.publish(msg)
        
    def ball_vel_callback(self,msg : Twist):
        if(self.index ==  0):
            self.vel_pub.publish(msg)
            
    def silo_vel_callback(self,msg : Twist):
        if(self.index ==  1):
            self.vel_pub.publish(msg)
            
    def luna_vel_callback(self,msg : Twist):
        if(self.index ==  2):
            self.vel_pub.publish(msg)
            
    def rnm_vel_callback(self,msg : Twist):
        if(self.index ==  3):
            self.vel_pub.publish(msg)
            
    def status_callback(self,msg):
        self.get_logger().info("received completion status")
        self.lastStatus = msg.data
        self.response_recvd = True
        
    def runtime(self):

        req = SetBool.Request()
        siloReq = SiloToGo.Request()
        siloReq.silo_number = 1
        req.data = True
        self.line_follower_client.call_async(req) 
        self.get_logger().info("calling line follower service")
        
        
        while True:
           
            # self.ball_tracking_future = self.ball_tracking_client.call_async(req)
            # # self.ball_tracking_client.add_done_callback(self.generic_response_callback("ball_tracking_srv"))
            
            # self.silo_tracking_future.call_async(req)
            # # self.ball_tracking_client.add_done_callback(self.generic_response_callback("silo_tracking_srv"))
            
            # self.luna_align_future.call_async(req)           
            # # self.ball_tracking_client.add_done_callback(self.generic_response_callback("luna_align_srv"))
            
            # self.rnm_future.call_async(req)
            # # self.ball_tracking_client.add_done_callback(self.generic_response_callback("rnm_srv"))
            
            

            if self.response_recvd :
                
                self.index = (self.index + 1) % 3
                
                if self.index == 0:
                    self.ball_tracking_client.call_async(req)
                    self.get_logger().info("calling ball tracking service")
                    
                
                elif self.index == 1:
                    self.silo_tracking_client.call_async(req)
                    self.get_logger().info("calling silo tracking service")
                    
                elif self.index == 2:
                    self.luna_align_client.call_async(siloReq)      
                    self.get_logger().info("calling luna align service")
                         
                elif self.index == 3:
                    self.rnm_client.call_async(req)
                    self.get_logger().info("calling rotate and move service")
                    
                    
                self.response_recvd = False
            
            time.sleep(0.1)
        
    def generic_response_callback(self,future,service_name : str):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'{service_name} task completed successfully')
            else:
                self.get_logger().warn(f'{service_name} task failed')
        except Exception as e:
            self.get_logger().error(f'Service call to {service_name} failed: {e}')
            
def main():
    rclpy.init()
    cm = ControlManager()

    runThread = Thread(target=lambda : cm.runtime()).start()

    rclpy.spin(cm)
    cm.destroy_node()
    rclpy.shutdown() 
    
    
if __name__ == "__main__":
    main()