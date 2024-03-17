#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool
import time

class SetBoolServer(Node):
    def __init__(self):
        super().__init__('set_bool_server')
        self.srv = self.create_service(SetBool, 'setbool', self.set_bool_callback)

    def set_bool_callback(self, request, response):
        time.sleep(3)
        response.success = not request.data
        response.message = 'Successfully inverted input boolean'
        self.get_logger().info('Incoming request\n%s' % request.data)
        self.get_logger().info('Sending response\n%s' % response.success)

        return response

def main(args=None):
    rclpy.init(args=args)

    set_bool_server = SetBoolServer()

    rclpy.spin(set_bool_server)

    set_bool_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()