
from launch import LaunchDescription
from launch_ros.actions import *


from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess
from math import pi


def generate_launch_description():
    
    
    gazebo_spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "/robot_description",
                   "-entity", "tarm","-x 0.1","-y 0.3","-z 0.4","-R 0","-P 0","-Y " + str(pi / 2)]
    )


    ld = LaunchDescription({
        # gazebo_spawn
    })
    
    ld.add_action(
        ExecuteProcess(
            cmd=[
                [
                    FindExecutable(name="ros2"),
                    " service call ",
                    "/delete_entity ",
                    "gazebo_msgs/srv/DeleteEntity ",
                    "\"{name: 'tarm'}\"",
                ]
            ],
            shell=True,
        )
    )
    
    
    return ld