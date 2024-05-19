from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import *
from launch_ros.actions import *
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.parameter_descriptions import ParameterValue



def generate_launch_description():
    
    share = FindPackageShare("r2_bringup")
    descShare = FindPackageShare("r2_description")
    
    urdfPath = PathJoinSubstitution([FindPackageShare("r2_description") , "description", "r2.urdf.xacro"])
    
    
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', urdfPath]), value_type=str)
    }]
    )
    
    
    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'launch/gazebo.launch.py'
                ]) ]),
        launch_arguments={
            "world" : PathJoinSubstitution([share, 'worlds', 'arena.world'])
        }.items()
    )
    
    pi = 3.1415
    gazebo_spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "/robot_description",
                   "-entity", "tarm","-x 0.1","-y 0.3","-z 0.4","-R 0","-P 0","-Y " + str(pi / 2)]
    )
    
    
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
    
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
        "-d",
        PathJoinSubstitution([descShare, "rviz", "config.rviz"])
        ]
    )
    

    jointStatePubGui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )
    
    return LaunchDescription({
        gazebo,
        gazebo_spawn,
        robot_state_pub,
        rviz,
        
        
        joy_node,
        teleop_node,
        twist_mux,
        
        jointStatePubGui
    }
        
    )