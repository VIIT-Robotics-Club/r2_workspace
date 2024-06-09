from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import *
from launch_ros.actions import *
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    
    share = FindPackageShare("r2_bringup")

    system_params = os.path.join(get_package_share_directory('r2_bringup'),'config','r2_system_params.yaml')
    
    
    yolo_results = Node(
        package="ball_tracking",
        name='yolo_results',
        executable="ball_detect_sim",
        parameters=[system_params]
    )
    
    quaternion_to_rpy = Node(
        package="r2_py",
        name='quaternion_to_rpy',
        executable="quat_to_rpy",
        parameters=[system_params]

    )
    
    robot_altitude_check = Node(
        package="r2_navigation",
        name='robot_altitude_check',
        executable="robot_altitude_check",
        parameters=[system_params]
    )   
    
    luna_allignment_server = Node(
        package='luna_control',
        name='luna_allignment_server',
        executable='luna_align_sim',
        parameters=[system_params]
    )
    
    silo_deciding_server = Node(   
        package='silo_tracking',
        name='silo_deciding_server',
        executable='silo_deciding',
        parameters=[system_params]
    )



    # common nodes for system operations




    # robot_state_pub = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     parameters=[{
    #         'robot_description': ParameterValue(Command(['xacro ', urdfPath]), value_type=str)
    # }]
    # )
    
    camera_params = PathJoinSubstitution([share ,"config", "cam.yaml"])
    
    camera_node = Node(package="camera_ros", executable="camera_node",
                        # arguments=[
                        # "--ros-args",
                        # "--params-file",
                        # camera_params],
                        
                        # name='camera_driver',
                       remappings=[
                           ("/camera/image_raw", "/image_raw"),
                            ("/camera/camera_info", "/image_raw/camera_info"),
                           ("/camera/image_raw/compressed", "/image_raw/compressed")
                           ])

    joy_params = PathJoinSubstitution([share, 'config', 'joystick.yaml'])

    joy_node = Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[joy_params],
        )
    
    teleop_joy = Node(
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

    
    camStreamCompress = Node(
        package="image_transport",
        executable="republish",
        arguments=["raw", "compressed"],
        remappings=[
            ("in", "/image_raw/annotated"),
            ("/out/compressed", "/image_raw/annotated_compressed")
        ]
    )

    return LaunchDescription([  
        yolo_results,
        quaternion_to_rpy,
        robot_altitude_check,
        luna_allignment_server,
        silo_deciding_server,
        camera_node,
        camStreamCompress,
        twist_mux,
        
        joy_node,
        teleop_joy
    ])