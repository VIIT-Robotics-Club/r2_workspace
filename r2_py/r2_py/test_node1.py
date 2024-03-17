#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time

class TestNode1(Node):
    def __init__(self):
        super().__init__("test_node_1")
        self.get_logger().info("Test Node 1 has been started.")
        time.sleep(3)
        
def main(args=None):
    rclpy.init(args=args)
    node = TestNode1()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
