#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node

class TestNode2(Node):
    def __init__(self):
        super().__init__("test_node_2")
        self.get_logger().info("Test Node 2 has been started.")
        time.sleep(3)
        
def main(args=None):
    rclpy.init(args=args)
    node = TestNode2()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
