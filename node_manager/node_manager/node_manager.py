import rclpy
from rclpy.node import Node
from r2_interfaces.srv import PerformTask
from example_interfaces.srv import SetBool
import time

class NodeManager():
    def __init__(self):
        super().__init__("node_manager")
        self.declare_paremeters("side","blue") # blue zone or red zone
        self.declare_parameters("retry",0) # 0 -> Normal Start  , 1 -> Start from Retry
        self.side = self.get_parameter("side").value
        self.retry = self.get_parameter("retry").value
        self.success_flag = False  # Initialize a flag to track success

        self.clients = {}
        self.service_names = ["ball_tracking_srv","navigation_server","silo_to_go","rotateNmove"]

        for service_name in self.service_names:
            self.clients[service_name] = self.create_client(PerformTask, service_name)

        for service_name in self.service_names:
            self.wait_for_service(service_name)
            
        self.gripper_grab_client = self.create_client(SetBool,"gripper_grab")
        self.gripper_lift_client = self.create_client(SetBool,"gripper_lift")
        
        while not self.gripper_grab_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Service gripper_grab not available, waiting again...')
        while not self.gripper_lift_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Service gripper_lift not available, waiting again...')
            
    def wait_for_service(self, service_name):
        while not self.clients[service_name].wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Service {service_name} not available, waiting again...')
            
        
    def execute_sequentially(self):
        # Retry start
        if self.retry == 1:
            self.call_service("line_follow","retry")
        # Normal Start
        else :
            self.call_service("line_follow","start")
            
        # Area 3 Game on
        while True:
            # Red Zone import asyncio
            if self.side == "red":
                self.call_service("ball_tracking_srv","red")
                self.call_bool_service("gripper_grab_close",True)
                self.call_bool_service("gripper_lift_up",True)
                self.call_service("navigation_server","red")
                self.call_service("silo_to_go","red")
                self.call_bool_service("gripper_grab_close",False)
                self.call_bool_service("gripper_lift_up",False)
                self.call_service("rotateNmove","red")
            # Blue Zone
            elif self.side == "blue":
                self.call_service("ball_tracking_srv","red")
                self.call_bool_service("gripper_grab_close",True)
                self.call_bool_service("gripper_lift_up",True)
                self.call_service("navigation_server","red")
                self.call_service("silo_to_go","red")
                self.call_bool_service("gripper_grab_close",False)
                self.call_bool_service("gripper_lift_up",False)
                self.call_service("rotateNmove","red")
        
        
    def call_service(self,service_name,state):
        req = PerformTask()
        req.srv_name = service_name
        req.state = state
        future = service_name.call_async(req) # Call asynchronously
        future.add_done_callback(self.response_callback(service_name))
        
    def call_bool_service(self,service_name,state):
        req = SetBool()
        req.data = state
        future = service_name.call_async(req)
        future.add_done_callback(self.response_callback(service_name))
        
        
    def response_callback(self, service_name):
        def callback(future):
            try:
                response = future.result()
                if response.success:
                    self.get_logger().info(f'{service_name} task completed successfully')
                else:
                    self.get_logger().warn(f'{service_name} task failed')
            except Exception as e:
                self.get_logger().error(f'Service call to {service_name} failed: {e}')
        return callback

    

def main(args=None):
    rclpy.init(args=args)
    node = NodeManager()
    node.execute_sequentially()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    