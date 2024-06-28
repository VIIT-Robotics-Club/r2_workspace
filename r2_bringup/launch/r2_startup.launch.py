from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    startup_params = os.path.join(get_package_share_directory('r2_bringup'), 'config', 'r2_startup_params.yaml')

    # Nodes
    

    silo_clients = Node(
        package='ball_tracking',
        executable='clients',
        name='clients',
        parameters=[{'track': "silo"}],
        output='screen'
    )
    line_follower = Node(
        package='luna_control',
        executable='line_follower',
        name='line_follower',
        # parameters=[{'track': "silo"}],
        output='screen'
    )
    luna_align_sim_4 = Node(
        package='luna_control',
        executable='luna_align_sim',
        name='luna_align_sim',
        # parameters=[{'track': "silo"}],
        output='screen'
    )
    luna_align_sim_1 = Node(
        package='luna_control',
        executable='luna_align_sim',
        name='luna_align_sim',
        # parameters=[{'track': "silo"}],
        output='screen'
    )
    luna_align_sim_2 = Node(
        package='luna_control',
        executable='luna_align_sim',
        name='luna_align_sim',
        # parameters=[{'track': "silo"}],
        output='screen'
    )
    luna_align_sim_3 = Node(
        package='luna_control',
        executable='luna_align_sim',
        name='luna_align_sim',
        # parameters=[{'track': "silo"}],
        output='screen'
    )

    ball_clients = Node(
        package='ball_tracking',
        executable='clients',
        name='clients',
        parameters=[{'track': "ball"}],
        output='screen'
    )

    gripper_lift_up = Node(
        package='r2_py',
        executable='motor_client',
        name='motor_client',
        output='screen',
        parameters=[
            {'client_to_call': 'gripper_lift', 'request_data': True}
        ]
    )

    gripper_lift_down = Node(
        package='r2_py',
        executable='motor_client',
        name='motor_client',
        output='screen',
        parameters=[
            {'client_to_call': 'gripper_lift', 'request_data': False}
        ]
    )

    gripper_grab_close = Node(
        package='r2_py',
        executable='motor_client',
        name='motor_client',
        output='screen',
        parameters=[
            {'client_to_call': 'gripper_grab', 'request_data': True}
        ]
    )

    gripper_grab_open = Node(
        package='r2_py',
        executable='motor_client',
        name='motor_client',
        output='screen',
        parameters=[
            {'client_to_call': 'gripper_grab', 'request_data': False}
        ]
    )
    ball_tracking_4 = Node(
        package='ball_tracking',
        executable='ball_tracking_sim_v4',
        name='ball_tracking',
        # parameters=[startup_params],
        output='screen'
    )
    silo_tracking_4 = Node(
        package='silo_tracking',
        executable='silo_tracking',
        name='silo_tracking',
        # parameters=[startup_params],
        output='screen'
    )
    ball_tracking_1 = Node(
        package='ball_tracking',
        executable='ball_tracking_sim_v4',
        name='ball_tracking',
        # parameters=[startup_params],
        output='screen'
    )
    silo_tracking_1 = Node(
        package='silo_tracking',
        executable='silo_tracking',
        name='silo_tracking',
        # parameters=[startup_params],
        output='screen'
    )
    ball_tracking_2 = Node(
        package='ball_tracking',
        executable='ball_tracking_sim_v4',
        name='ball_tracking',
        # parameters=[startup_params],
        output='screen'
    )
    silo_tracking_2 = Node(
        package='silo_tracking',
        executable='silo_tracking',
        name='silo_tracking',
        # parameters=[startup_params],
        output='screen'
    )
    ball_tracking_3 = Node(
        package='ball_tracking',
        executable='ball_tracking_sim_v4',
        name='ball_tracking',
        # parameters=[startup_params],
        output='screen'
    )
    silo_tracking_3 = Node(
        package='silo_tracking',
        executable='silo_tracking',
        name='silo_tracking',
        # parameters=[startup_params],
        output='screen'
    )
    navigation_server = Node(
        package='r2_navigation',
        executable='navigation_server',
        parameters=[startup_params],
        output='screen'
    )

    rotate_and_move = Node(
        package='r2_navigation',
        executable='rotate_and_move',
        name='rotate_and_move',
        parameters=[startup_params],
        output='screen'
    )

    # Event Handlers
    gripper_lift_up_and_close = [
        gripper_grab_close,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gripper_grab_close,
                on_exit=gripper_lift_up
            )
        )
    ]

    # return LaunchDescription([
        
    #     ball_clients,
    #     RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=ball_clients,
    #             on_exit=gripper_lift_up_and_close
    #         )
    #     ),
    #     RegisterEventHandler(
    #         event_handler=OnProcessStart(
    #             target_action=gripper_lift_up,
    #             on_start=silo_clients
    #         )
    #     ),
    #     RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=silo_clients,
    #             on_exit=gripper_grab_open
    #         )
    #     ),
    #     RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=gripper_grab_open,
    #             on_exit=rotate_and_move
    #         )
    #     ),
    #     RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=rotate_and_move,
    #             on_exit=ball_clients
    #         )
    #     ),
    #     RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=ball_clients,
    #             on_exit=gripper_lift_up_and_close
    #         )
    #     ),
    #     RegisterEventHandler(
    #         event_handler=OnProcessStart(
    #             target_action=rotate_and_move,
    #             on_start=gripper_lift_down
    #         )
    #     ),
    # ])
    return LaunchDescription([
            line_follower,
            # ball_tracking_1,
            RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=line_follower,
                        on_exit=ball_tracking_1
                    )
                ),
            RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=ball_tracking_1,
                        on_exit=silo_tracking_1
                    )
                ),

                # RegisterEventHandler(
                #     event_handler=OnProcessStart(
                #         target_action=gripper_lift_up,
                #         on_start=silo_tracking
                #     )
                # ),
            RegisterEventHandler(
                    event_handler=OnProcessStart(
                        target_action=silo_tracking_1,
                        on_start=luna_align_sim_1
                    )
                ),

            RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=luna_align_sim_1,
                        on_exit=ball_tracking_2
                    )
                ),
          
            RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=ball_tracking_2,
                        on_exit=silo_tracking_2
                    )
                ),

                # RegisterEventHandler(
                #     event_handler=OnProcessStart(
                #         target_action=gripper_lift_up,
                #         on_start=silo_tracking
                #     )
                # ),
            RegisterEventHandler(
                    event_handler=OnProcessStart(
                        target_action=silo_tracking_2,
                        on_start=luna_align_sim_2
                    )
                ),

            RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=luna_align_sim_2,
                        on_exit=ball_tracking_3
                    )
                ),
            RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=ball_tracking_3,
                        on_exit=silo_tracking_3
                    )
                ),

                # RegisterEventHandler(
                #     event_handler=OnProcessStart(
                #         target_action=gripper_lift_up,
                #         on_start=silo_tracking
                #     )
                # ),
            RegisterEventHandler(
                    event_handler=OnProcessStart(
                        target_action=silo_tracking_3,
                        on_start=luna_align_sim_3
                    )
                ),

            RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=luna_align_sim_3,
                        on_exit=ball_tracking_4
                    )
                ),
            RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=ball_tracking_4,
                        on_exit=silo_tracking_4
                    )
                ),

                # RegisterEventHandler(
                #     event_handler=OnProcessStart(
                #         target_action=gripper_lift_up,
                #         on_start=silo_tracking
                #     )
                # ),
            RegisterEventHandler(
                    event_handler=OnProcessStart(
                        target_action=silo_tracking_4,
                        on_start=luna_align_sim_4
                    )
                ),

            # RegisterEventHandler(
            #         event_handler=OnProcessExit(
            #             target_action=luna_align_sim_4,
            #             on_exit=ball_tracking_4
            #         )
            #     ),
          


    ])