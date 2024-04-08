#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
import os
from ament_index_python.packages import get_package_share_directory


package_path = get_package_share_directory('r2_bringup')

file_path = os.path.join(package_path, 'launch', 'position_name.txt')

class TestNode2(Node):
    def __init__(self):
        super().__init__("test_node_2")
        self.get_logger().info("Test Node 2 has been started.")
        # time.sleep(3)

        position_name = os.getenv('POSITION_NAME', '.')
        self.get_logger().info(f"Position name: {position_name}")

        with open(file_path, 'r') as f:
            position_name = f.read().strip()
            self.get_logger().info(f"Position name: {position_name}")
        
        print(position_name)

        
def main(args=None):
    rclpy.init(args=args)
    node = TestNode2()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
