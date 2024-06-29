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
        executable='luna_align_srv',
        namespace="luna",
        remappings=[
            ("/nav_vel", "/luna/nav_vel"),
            ("status", "/status")],
        parameters=[system_params]
    )
    
    silo_deciding_server = Node(   
        package='silo_tracking',
        name='silo_deciding_server',
        executable='silo_deciding',
        parameters=[system_params]
    )
    
    lineFollowerService = Node(
        package='luna_control',
        executable='lf_sub',
        namespace="line",
        remappings=[
            ("/nav_vel", "/line/nav_vel"),
            ("status", "/status")],
        parameters=[system_params]
    )
    
    
    controller = Node(
        package='controls',
        executable='control_manager',
        parameters=[system_params]
    )
    
    siloTrackingServer = Node(
        package='silo_tracking',
        name='silo_tracking_server',
        executable='silo_tracking_server',
        namespace="silo",
        remappings=[
            ("/nav_vel", "/silo/nav_vel"),
            ("/silo/yolo_results", "/yolo_results"),
            
            ("status", "/status")],
        parameters=[system_params]
    )
    
    
    ballTrackingServer  = Node(
        package='ball_tracking',
        name='ball_tracking_server',
        executable='ball_tracking_server',
        namespace="ball",
        remappings=[
            ("/nav_vel", "/ball/nav_vel"),
            ("/ball/yolo_results", "/yolo_results"),
            ("status", "/status")],
        parameters=[system_params]
    )
    
    return LaunchDescription([
        yolo_results,
        # quaternion_to_rpy,
        # robot_altitude_check,
        luna_allignment_server,
        # silo_deciding_server,
        lineFollowerService,
        controller,
        siloTrackingServer,
        ballTrackingServer
    ])