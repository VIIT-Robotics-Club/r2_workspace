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

    startup_params = os.path.join(get_package_share_directory('r2_bringup'),'config','r2_startup_params.yaml')

    ball_tracking = Node(
        package='ball_tracking',
        executable='ball_tracking_sim',
        name='ball_tracking',
        parameters=[startup_params],
        output='screen'
    )
    
    gripper_lift_up = Node(
        package='r2_py',
        executable='motor_client',
        name='motor_client',
        output='screen',
        shell=True,
        parameters=[
            {'client_to_call': 'gripper_lift', 'request_data': True}
        ]
    )
    
    gripper_lift_down = Node(
        package='r2_py',
        executable='motor_client',
        name='motor_client',
        output='screen',
        shell=True,
        parameters=[
            {'client_to_call': 'gripper_lift', 'request_data': False}
        ]
    )
    
    gripper_grab_close = Node(
        package='r2_py',
        executable='motor_client',
        name='motor_client',
        output='screen',
        shell=True,
        parameters=[
            {'client_to_call': 'gripper_grab', 'request_data': True}
        ]
    )
    
    gripper_grab_open = Node(
        package='r2_py',
        executable='motor_client',
        name='motor_client',
        output='screen',
        shell=True,
        parameters=[
            {'client_to_call': 'gripper_grab', 'request_data': False}
        ]
    )
    
    gripper_lift_up_and_close = [
        gripper_grab_close,
        
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gripper_grab_close,
                on_exit=gripper_lift_up
            )
        )
    ]
    
    navigation_server = Node(               #Robot has the ball -> go to silo
        package='r2_navigation',
        executable='navigation_server',
        parameters=[startup_params],
        output='screen'
    )
    
    


    return LaunchDescription([
        
        ball_tracking,
        
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=ball_tracking,
                on_exit=gripper_lift_up_and_close
            )
        ),
        
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=gripper_lift_up,
                on_start=navigation_server
            )
        ),
        
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=navigation_server,
                on_exit=gripper_grab_open
            )
        ),
        
    ])
