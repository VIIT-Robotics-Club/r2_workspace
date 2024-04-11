
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

    imu_complimentary_node = launch_ros.actions.Node(
                package='imu_complementary_filter',
                executable='complementary_filter_node',
                name='complementary_filter_gain_node',
                output='screen',
                parameters=[
                    {'do_bias_estimation': True},
                    {'do_adaptive_gain': True},
                    {'use_mag': False},
                    {'gain_acc': 0.01},
                    {'gain_mag': 0.01},
                ]
    )



    return launch.LaunchDescription(
        [
            imu_complimentary_node
        ]
    )


