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


    # This is the Node for ball tracking update: Update the version of script to be used
    ball_tracking = Node(
        package='ball_tracking',
        executable='ball_tracking_sim',
        name='ball_tracking',
        parameters=[startup_params],
        output='screen'
    )
    
    # Object to call the motor_client node to lift the gripper
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
    
    # Object to call the motor_client node to lower the gripper
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
    
    # Object to call the motor_client node to close the gripper
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
    
    # Object to call the motor_client node to open the gripper
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
    
    # Sequence of actions to be executed after the ball tracking node is done: Gripper lift up and close
    gripper_lift_up_and_close = [
        gripper_grab_close,
        
        # Register the event handler to execute the gripper_lift_down node after the gripper_grab_close node is done
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gripper_grab_close,
                on_exit=gripper_lift_up
            )
        )
    ]
    
    # Start the navigation server node: This is the node that will be used to move the robot towards the silo 
    navigation_server = Node(              
        package='r2_navigation',
        executable='navigation_server',
        parameters=[startup_params],
        output='screen'
    )

    # Start the rotate_and_move node: This is the node that will be used to rotate the robot towards the balls and move it forward
    rotate_and_move = Node(
        package='r2_navigation',
        executable='rotate_and_move',
        name='rotate_and_move',
        parameters=[startup_params],
        output='screen'
    )   
    

    return LaunchDescription([
        
        # Robot tracks the balls -> Go towards a ball and stop
        ball_tracking,
        
        #Along with ball tracking node, gripper should also be lowered (When the robot is crossing the slope): Gripper claw should be opened at this point
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=ball_tracking,
                on_start=gripper_lift_down
            )
        ),

        # After robot is near the ball, lift the gripper and close it
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=ball_tracking,
                on_exit=gripper_lift_up_and_close
            )
        ),
        
        # Robot sweeps until the robot faces the silo -> Move forward until slope is climbed -> Luna Client is called with optimal silo position
        #                   |
        #                   |--> Whenever 5 silo is detected it will also call the silo decision server to get the optimal silo number for the luna script
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=gripper_lift_up,
                on_start=navigation_server
            )
        ),


        # After the robot is near the silo, open the gripper
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=navigation_server,
                on_exit=gripper_grab_open
            )
        ),

        # After the ball is dropped, rotate the robot towards the ball and move forward
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gripper_grab_open,
                on_exit=rotate_and_move
            )
        )
        
    ])
