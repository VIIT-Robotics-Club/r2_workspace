from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, EmitEvent, ExecuteProcess
from launch.event_handlers import OnProcessExit, OnProcessStart, OnProcessIO
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.event_handlers import OnExecutionComplete
from launch import logging
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    ball_tracking_params = os.path.join(get_package_share_directory('r2_bringup'),'config','ball_tracking.yaml')



    ball_following = Node(
        package='ball_tracking',
        executable='ball_tracker2',
        name='ball_tracker2',
        parameters=[ball_tracking_params]
    )


    line_follow = Node(
        package='line_following',
        executable='pid_tuning.py',
        name='line_follower'
    )

    # command to run ros2 service call /setbool std_srvs/srv/SetBool {data: True}
    motor_on = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/setbool', 'std_srvs/srv/SetBool', '{data: True}'],
        output='screen'
    )

    motor_off = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/setbool', 'std_srvs/srv/SetBool', '{data: False}'],
        output='screen'
    )

    silo_tracking = Node(
        package='silo_tracking',
        executable='silo_tracking',
        name='silo_tracking'
    )


    return LaunchDescription([
        line_follow,

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=line_follow,
                on_exit=ball_following
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=ball_following,
                on_exit=motor_off
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=motor_off,
                on_exit=motor_on
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=motor_on,
                on_exit=silo_tracking
            )
        ),
    ])