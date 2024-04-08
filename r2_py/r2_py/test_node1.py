#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import os
from ament_index_python.packages import get_package_share_directory


package_path = get_package_share_directory('r2_bringup')

file_path = os.path.join(package_path, 'launch', 'position_name.txt')
class TestNode1(Node):
    def __init__(self):
        super().__init__("test_node_1")
        with open(file_path, 'w') as f:
            f.write('silo2')

        


        
def main(args=None):
    rclpy.init(args=args)
    node = TestNode1()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
