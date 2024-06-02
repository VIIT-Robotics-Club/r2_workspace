#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
# from example_interfaces.srv import SetBool
from r2_interfaces.srv import GripperCmd

import time

class GripperCmdClient(Node):
    def __init__(self):
        super().__init__('set_bool_client')
        self.gripper_grab_client = self.create_client(GripperCmd, 'gripper_grab')
        self.gripper_lift_client = self.create_client(GripperCmd, 'gripper_lift')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('client_to_call', '/gripper_lift'),
                ('request_data', True)
            ]
        )

        while not self.gripper_grab_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('gripper_grab service not available, waiting again...')
        self.get_logger().info('gripper_grab service is available')

        while not self.gripper_lift_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('gripper_lift service not available, waiting again...')
        self.get_logger().info('gripper_lift service is available')

        self.get_logger().info('GripperCmd client has been started')
        self.req = GripperCmd.Request()


    def send_request(self):
        client_to_call = self.get_parameter('client_to_call').get_parameter_value().string_value
        request_data = self.get_parameter('request_data').get_parameter_value().bool_value

        self.req.data = request_data
        self.get_logger().info('Sending request\n%s' % self.req.data)

        if client_to_call == 'gripper_grab':
            future = self.gripper_grab_client.call_async(self.req)
        elif client_to_call == 'gripper_lift':
            future = self.gripper_lift_client.call_async(self.req)
        else:
            self.get_logger().info('Invalid client_to_call parameter value')
            return

        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info('Result received: %s' % future.result().result)
        else:
            self.get_logger().info('Service call failed')

def main(args=None):
    rclpy.init(args=args)

    set_bool_client = GripperCmdClient()
    set_bool_client.send_request()
    # while rclpy.ok():
    #     set_bool_client.send_request()
    #     time.sleep(7)

    set_bool_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()