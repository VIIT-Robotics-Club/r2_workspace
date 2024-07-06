from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32
from rcl_interfaces.msg import SetParametersResult
from std_srvs.srv import SetBool
from threading import Thread
import rclpy
import time
from r2_interfaces.srv import SiloToGo, BestSilo

class ControlManager(Node):
    
    def __init__(self):
        super().__init__('control_manager')
        self.get_logger().info("control manager initialized")
        
        
        # parameters to enable and disable sections 
        self.declare_parameter("enableLineFollowing", True)
        self.enableLineFollowing =self.get_parameter("enableLineFollowing").value
        self.declare_parameter("enableBallTracking", True)
        self.enableBallTracking =self.get_parameter("enableBallTracking").value
        self.declare_parameter("enablesiloDetection", True)
        self.enablesiloDetection =self.get_parameter("enablesiloDetection").value
        self.declare_parameter("enableLunaAlignment", True)
        self.enableLunaAlignment =self.get_parameter("enableLunaAlignment").value
        self.declare_parameter("enableRnm", True)
        self.enableRnm =self.get_parameter("enableRnm").value

        self.add_on_set_parameters_callback(self.parameters_callback)
        self.active=False

        # velocities from each individual sections
        self.create_subscription(Twist,'line/nav_vel',self.line_follow_vel_callback, 10)
        self.create_subscription(Twist,'ball/nav_vel',self.ball_vel_callback, 10)
        self.create_subscription(Twist,'luna/nav_vel',self.luna_vel_callback, 10)
        self.create_subscription(Twist,'silo/nav_vel',self.silo_vel_callback, 10)
        self.create_subscription(Twist,'rnm/nav_vel',self.rnm_vel_callback, 10)
        self.vel_pub = self.create_publisher(Twist,"nav_vel",10)
        

        self.create_subscription(Bool,'status',self.status_callback, 10)
        self.create_subscription(Int32,'best_silo',self.best_silo_callback, 10)
        self.create_subscription(Bool,'valid_silo',self.valid_silo_callback, 10)
        
        # create clients to activated individual services
        self.line_follower_client = self.create_client(SetBool,"lf_srv")
        while not self.line_follower_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn('lf_srv Service not available, waiting !!')
        
        
        self.retry_service = self.create_service(SetBool, "/retry", self.retry_service_callback)
        
        
        self.ball_tracking_client = self.create_client(SetBool ,"ball_tracking_srv")
        while not self.ball_tracking_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn('ball_tracking_srv Service not available, waiting !!')
        
        self.silo_tracking_client = self.create_client(SetBool ,"silo_tracking_srv")
        while not self.silo_tracking_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn('silo_tracking_srv Service not available, waiting !!')
            
        self.luna_align_client = self.create_client(SiloToGo ,"luna_align_srv")
        while not self.luna_align_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn('luna_align_srv Service not available, waiting !!')
            
        self.rnm_client = self.create_client(SetBool ,"/rnm_srv")
        while not self.rnm_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn('rotate and move service not available, waiting !!')
        
        self.gripper_lift_client = self.create_client(SetBool ,"gripper_lift")
        while not self.gripper_lift_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn('gripper_lift Service not available, waiting !!')
        
        self.gripper_grab_client = self.create_client(SetBool ,"gripper_grab")
        while not self.gripper_grab_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn('gripper grab Service not available, waiting !!')    
    

        self.best_silo_client = self.create_client(BestSilo ,"best_silo",)
        while not self.gripper_grab_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn('best silo service not available, waiting !!')    



        self.bestSiloNum = 0
        self.siloUpdated = False
        self.lastStatus = True
        self.response_recvd = False
        self.index = -1
        
    
    def retry_service_callback(self, request, response):
        
        
        return response


    def parameters_callback(self, params):
        for param in params:
            param_name = param.name
            param_value = param.value
            setattr(self, param_name, param_value)
        return SetParametersResult(successful=True)
    

    def best_silo_callback(self,msg):
        self.siloUpdated = True
        self.bestSiloNum = msg.data
        

    def valid_silo_callback(self,msg):
        # self.siloUpdated = True
        self.valid_silo = msg.data

        
    def updateBestSilo(self):
        self.bestSiloNum = 0
        response = self.best_silo_client.call(BestSilo.Request())
    
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
        
    def setGripperConfiguration(self, lift : bool, grab : bool):
        liftMsg = SetBool.Request()
        grabMsg = SetBool.Request()
        
        liftMsg.data = lift
        grabMsg.data = grab

        self.gripper_grab_client.call_async(grabMsg)
        time.sleep(1)
        self.gripper_lift_client.call_async(liftMsg)
        time.sleep(1)
        
        
        
    def runtime(self):

        req = SetBool.Request()
        req.data = True
        
                            
        if self.enableLineFollowing:
            self.line_follower_client.call_async(req) 
            self.get_logger().info("calling line follower service")
        else:
            self.response_recvd = True
        
        while True:

            if self.response_recvd :
                self.index = (self.index + 1) % 4

                self.get_logger().info("index = " + str(self.index))

                
                if self.enableBallTracking and self.index == 0:
                    
                    
                    # move gripper down and open configuration
                    self.setGripperConfiguration(False, False)
                    self.ball_tracking_client.call_async(req)
                    req.data = False
                    self.grab_client.call_async(req)
                    self.get_logger().info("calling ball tracking service")
                    self.response_recvd = False
                    
                
                elif  self.enablesiloDetection and self.index == 1:
                    self.setGripperConfiguration(True, True)
                    self.silo_tracking_client.call_async(req)
                    # time.sleep(1)
                    self.get_logger().info("calling silo tracking service")
                    self.updateBestSilo()
                    self.response_recvd = False

                    
                    
                elif self.enableLunaAlignment and self.index == 2:

                    # decrement index so that next iteration calls luna again
                    if self.bestSiloNum == 0:
                        self.index = self.index - 1
                    
                    siloReq = SiloToGo.Request()
                    
                    if not self.siloUpdated:
                        # go to recapture position and wait for next healthy frame
                        siloReq.silo_number = 0
                        self.luna_align_client.call_async(siloReq)  
                                                
                        self.get_logger().info("going to recapture point ")
                        while not self.siloUpdated:
                            time.sleep(1)

                    # goto target silo
                    siloReq.silo_number = self.bestSiloNum
                    self.luna_align_client.call_async(siloReq)    
                    self.response_recvd = False
                    self.siloUpdated = False
                        
                    self.get_logger().info("going to silo " + str(self.bestSiloNum))
                    self.get_logger().info("calling luna align service")

                         
                elif self.enableRnm and self.index == 3:
                    # if self.valid_silo:
                    self.setGripperConfiguration(True, False)
                    self.rnm_client.call_async(req) 
                    self.get_logger().info("calling rotate and move service")
                    
                    self.response_recvd = False
                    # else:
                    #     # goto silo to right, fifth silo is mapped to first silo
                        
                    #     self.bestSiloNum= (self.bestSiloNum+1) % 5
                    #     siloReq.silo_number = self.bestSiloNum
                    #     self.luna_align_client.call_async(siloReq)   
                         
    

                else:
                    self.response_recvd = True
                    
                    
            
            time.sleep(1)
        
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