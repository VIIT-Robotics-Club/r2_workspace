
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import LaunchConfiguration
import launch.actions


def generate_launch_description():


    imu_madwick_node = launch_ros.actions.Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('r2_bringup'), 'config', 'imu_filter.yaml')],
    )

    return launch.LaunchDescription(
        [
            imu_madwick_node
        ]
    )


