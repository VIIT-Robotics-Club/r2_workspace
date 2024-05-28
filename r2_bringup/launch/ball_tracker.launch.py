from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    
        
    yolo_results = Node(
        package="ball_tracking",
        executable="ball_detect_sim",
    )
    
    quaternion_to_rpy = Node(
        package="r2_py",
        executable="quat_to_rpy",
    )
    
    robot_altitude_check = Node(
        package="r2_py",
        executable="robot_altitude_check",
    )
    
    
    return LaunchDescription([
        yolo_results,
        quaternion_to_rpy,
        robot_altitude_check
    ])