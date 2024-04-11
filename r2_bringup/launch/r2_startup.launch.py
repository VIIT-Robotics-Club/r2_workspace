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
    silo_luna_params = os.path.join(get_package_share_directory('r2_bringup'),'config','luna_allign.yaml')



    ball_following = Node(
        package='ball_tracking',
        executable='ball_tracker4',
        name='detect4',
        parameters=[
            {'Ball_Name': "Red-ball"},
            {'setupareasilo': -48000.00},
             {'setupdev': 180.00},
            #  {'weights': "weights/redballbest.pt"},

            
        ]
    )


    # line_follow = Node(
    #     package='line_following',
    #     executable='pid_tuning.py',
    #     name='line_follower'
    # )

    line_following_client = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/service_line_follow', 'std_srvs/srv/SetBool', '{data: True}'],
        output='screen'
    )

    # command to run ros2 service call /setbool std_srvs/srv/SetBool {data: True}
    lift_down = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/service_lift', 'std_srvs/srv/SetBool', '{data: False}'],
        output='screen'
    )

    lift_up = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/service_lift', 'std_srvs/srv/SetBool', '{data: True}'],
        output='screen'
    )

    claw_open = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/service_claw', 'std_srvs/srv/SetBool', '{data: False}'],
        output='screen'
    )

    claw_close = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/service_claw', 'std_srvs/srv/SetBool', '{data: True}'],
        output='screen'
    )

    silo_tracking = Node(
        package='silo_tracking',
        executable='silo_tracking2',
        name='silotracking2',
        parameters=[
            {'Ball_Name': "silo"},
            {'setupareasilo': -15000.00},
            # {'weights': "weights/redbluesilo3.pt"},
            
        ]
    )

    go_to_silo_1_luna = Node(
        package='luna_control',
        executable='luna_wall_align',
        name='luna_wall_align',
        parameters=[
            {'silo_number': 1},
            silo_luna_params
        ]
    )

    go_to_silo_2_luna = Node(
        package='luna_control',
        executable='luna_wall_align',
        name='luna_wall_align',
        parameters=[
            {'silo_number': 2},
            silo_luna_params
        ]
    )

    go_to_silo_3_luna = Node(
        package='luna_control',
        executable='luna_wall_align',
        name='luna_wall_align',
        parameters=[
            {'silo_number': 3},
            silo_luna_params
        ]
    )

    go_to_silo_4_luna = Node(
        package='luna_control',
        executable='luna_wall_align',
        name='luna_wall_align',
        parameters=[
            {'silo_number': 4},
            silo_luna_params
        ]
    )

    go_to_silo_5_luna = Node(
        package='luna_control',
        executable='luna_wall_align',
        name='luna_wall_align',
        parameters=[
            {'silo_number': 5},
            silo_luna_params
        ]
    )

    


    return LaunchDescription([

        # line_following_client,

        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=line_following_client,
        #         on_exit=ball_following
        #     )
        # ),

        ball_following,

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=ball_following,
                on_exit=claw_close
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=claw_close,
                on_exit=lift_up
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=lift_up,
                on_exit=silo_tracking
            )
        ),

        # silo_tracking,

        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=silo_tracking,
        #         on_exit=go_to_silo_2_luna
        #     )
        # ),
    ])