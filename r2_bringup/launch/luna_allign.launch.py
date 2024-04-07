
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import LaunchConfiguration
import launch.actions


def generate_launch_description():


    luna_align = launch_ros.actions.Node(
        package='luna_control',
        executable='luna_wall_align',
        name='luna_wall_align',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('r2_bringup'), 'config', 'luna_align.yaml')],
    )

    return launch.LaunchDescription(
        [
            luna_align
        ]
    )


