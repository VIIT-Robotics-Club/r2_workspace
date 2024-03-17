#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool

class SetBoolClient(Node):
    def __init__(self):
        super().__init__('set_bool_client')
        self.cli = self.create_client(SetBool, 'setbool')
        self.get_logger().info('SetBool client has been started')
        self.req = SetBool.Request()

    def send_request(self):
        self.req.data = False
        self.get_logger().info('Sending request\n%s' % self.req.data)
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Result received: %s' % future.result().success)
        else:
            self.get_logger().info('Service call failed')

def main(args=None):
    rclpy.init(args=args)

    set_bool_client = SetBoolClient()
    set_bool_client.send_request()

    rclpy.spin(set_bool_client)

    set_bool_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()