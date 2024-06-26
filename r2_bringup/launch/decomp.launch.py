from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import *
from launch_ros.actions import *
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue
from math import pi
from launch.conditions import IfCondition 


def generate_launch_description():
    
    camStreamDecompress = Node(
        package="image_transport",
        executable="republish",
        arguments=["compressed", "raw"],
        remappings=[
            ("in/compressed", "/camera/image_raw/annotated_compressed"),
            ("/out", "/uncompressed_stream")
        ]
    )

    return LaunchDescription(
        {
            camStreamDecompress            
        }
    )