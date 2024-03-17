from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, EmitEvent, ExecuteProcess
from launch.event_handlers import OnProcessExit, OnProcessStart, OnProcessIO
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.event_handlers import OnExecutionComplete
from launch import logging

logger = logging.get_logger('launch.user')

def generate_launch_description():
    motor_server_fake = Node(
        package='r2_py',
        executable='motor_server_fake',
        name='motor_server_fake',
        output='screen',
        shell=True,

    )

    test_node1 = Node(
        package='r2_py',
        executable='test_node1',
        name='test_node1',
        output='screen',
        shell=True,

    )

    test_node2 = Node(
        package='r2_py',
        executable='test_node2',
        name='test_node2',
        output='screen',
        shell=True,
    )

    call_service = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/setbool', 'example_interfaces/srv/SetBool', '{data: true}'],
        output='screen',
    )

    log_info = ExecuteProcess(
        cmd=['ros2', 'topic', 'list'],
        output='screen',
    )

    return LaunchDescription([
        motor_server_fake,
        # RegisterEventHandler(
        #     OnProcessStart(
        #         target_action=motor_server_fake,
        #         on_start=[call_service],
        #     )
        # ),
        call_service,
        RegisterEventHandler(
            OnProcessIO(
                target_action=call_service,
                on_stdout=lambda event: 
                    logger.info(event.text.decode())
                    # test_node1 if 'success: True' in event.text.decode() else test_node2
            )
        ),
    ])