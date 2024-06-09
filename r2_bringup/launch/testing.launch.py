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
    
    share = FindPackageShare("r2_bringup")
    
    joy_params = PathJoinSubstitution([share, 'config', 'joystick.yaml'])

    joy_node = Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[joy_params],
        )
    
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[joy_params],
        remappings=[('cmd_vel', 'joy_vel')],
    )
    
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        remappings=[
            ('/cmd_vel_out', '/cmd_vel')
        ],
        parameters= [PathJoinSubstitution([share, "config", "twist_mux.yaml"])]
    )

    
    
    camera_params = PathJoinSubstitution([share ,"config", "cam.yaml"])
    
    camera_node = Node(package="camera_ros", executable="camera_node",
                        arguments=[
                        "--ros-args",
                        "--params-file",
                        camera_params],
                        
                       remappings=[
                           ("/camera/image_raw", "/image_raw"),
                            ("/camera/camera_info", "/image_raw/camera_info"),
                           ("/camera/image_raw/compressed", "/image_raw/compressed")
                           ])

    nodes = {
        # launch arguments,        
        joy_node,
        teleop_node,
        twist_mux,
        camera_node
    }
    
    otherNode = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                share, 'launch/system.launch.py'
            ])]))

    nodes.add(otherNode)

    return LaunchDescription(nodes)