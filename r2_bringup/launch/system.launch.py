from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros.actions
from launch.actions import RegisterEventHandler, EmitEvent, ExecuteProcess, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart, OnProcessIO
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.event_handlers import OnExecutionComplete
from launch import logging
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    
    system_params = os.path.join(get_package_share_directory('r2_bringup'),'config','r2_system_params.yaml')
    
    
    yolo_results = Node(
        package="ball_tracking",
        name='yolo_results',
        executable="ball_detect_sim",
        parameters=[system_params]
    )
    
    quaternion_to_rpy = Node(
        package="r2_py",
        name='quaternion_to_rpy',
        executable="quat_to_rpy",
        parameters=[system_params]

    )
    
    robot_altitude_check = Node(
        package="r2_navigation",
        name='robot_altitude_check',
        executable="robot_altitude_check",
        parameters=[system_params]
    )   
    
    luna_allignment_server = Node(
        package='luna_control',
        name='luna_allignment_server',
        executable='luna_align_sim',
        parameters=[system_params]
    )
    
    silo_deciding_server = Node(   
        package='silo_tracking',
        name='silo_deciding_server',
        executable='silo_deciding',
        parameters=[system_params]
    )
    
    
    return LaunchDescription([
        # yolo_results,
        quaternion_to_rpy,
        robot_altitude_check,
        # luna_allignment_server,
        # silo_deciding_server
    ])