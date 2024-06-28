#!/usr/bin/env python3

from rclpy.node import Node
import rclpy
from collections import deque
from geometry_msgs.msg import Vector3
from example_interfaces.msg import Bool


class RobotAltitudeCheckNode(Node):
    def __init__(self):
        super().__init__('robot_altitude_check_node')
        self.get_logger().info("Robot Altitude Check Node Started")

        self.create_subscription(
            Vector3,
            '/rpy',
            self.subscriber_callback,
            10
        )

        self.publisher = self.create_publisher(
            Bool,
            '/robot_altitude_state',
            10
        )
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('pitch_threshold', 2.0),                         # Pitch threshold for detecting climb/descent
                ('noise_threshold', 0.3),                # Threshold to consider the pitch stable around 0
                ('pitch_history_size', 7),                      # Size of pitch values (tuned for the robot's speed)
                ('slope_state', False),                          # Initial state 
                ('logging', False)
            ]
        )
        
        # Get parameters
        self.pitch_threshold = self.get_parameter('pitch_threshold').value
        self.noise_threshold = self.get_parameter('noise_threshold').value
        self.pitch_history_size = self.get_parameter('pitch_history_size').value
        self.slope_state = self.get_parameter('slope_state').value
        
        # Initialize pitch history
        self.pitch_history = deque(maxlen=self.pitch_history_size) 
                 
        # Initialize flags
        self.is_climbing = False
        self.is_descending = False

    def subscriber_callback(self, msg):

        # Get pitch value
        pitch = msg.y
        # pitch = msg.y
        
        # Append pitch value to history
        self.pitch_history.append(pitch)


        # Calculate average pitch
        pitch_avg = sum(self.pitch_history) / len(self.pitch_history)

        # Log pitch values
        if self.get_parameter('logging').value:
            self.get_logger().info(f"Average pitch: {pitch_avg}")
            # self.get_logger().info(f"Pitch history: {list(self.pitch_history)}")

        # Check if the robot is climbing or descending
        if not self.is_climbing and not self.is_descending:
            
            if abs(pitch_avg) >= self.pitch_threshold:     
                     
                if pitch_avg > 0:   
                    self.is_climbing = True
                    
                    # if self.get_parameter('logging').value:
                    #     self.get_logger().info("Started climbing")
                        
                else:
                    
                    self.is_descending = True
                    
                    # if self.get_parameter('logging').value:
                    #     self.get_logger().info("Started descending")

        # Check if the robot has climbed or descended the slope
        if self.is_climbing and abs(pitch_avg) < self.noise_threshold:
            self.is_climbing = False
            self.slope_state = not self.slope_state
            
            # if self.get_parameter('logging').value:
            #     self.get_logger().info("Climbed the slope, state flipped")

        # Check if the robot has climbed or descended the slope
        if self.is_descending and abs(pitch_avg) < self.noise_threshold:
            self.is_descending = False
            self.slope_state = not self.slope_state
            
            # if self.get_parameter('logging').value:
            #     self.get_logger().info("Descended the slope, state flipped")

        # Publish the state
        self.publish_state()

    def publish_state(self):
        msg = Bool()
        msg.data = self.slope_state
        self.publisher.publish(msg)
        
        # if self.get_parameter('logging').value:
        #     self.get_logger().info(f"Published state: {'on top' if self.slope_state else 'at bottom'}")

def main(args=None):
    rclpy.init(args=args)
    robot_altitude_check_node = RobotAltitudeCheckNode()
    rclpy.spin(robot_altitude_check_node)
    robot_altitude_check_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
