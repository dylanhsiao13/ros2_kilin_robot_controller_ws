import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # VM-specific environment variables for new Gazebo (Ignition)
    os.environ["IGN_GAZEBO_SYSTEM_PLUGIN_PATH"] = "/opt/ros/humble/lib"
    
    # Path to URDF
    robot_description_path = PathJoinSubstitution([
        FindPackageShare("kilin_description"),
        "urdf",
        "URDF.urdf"
    ])

    # Ignition Gazebo 6 (new Gazebo) - headless server for VM compatibility
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            ])
        ]),
        launch_arguments={
            "gz_args": "-s -r -v 4 empty.sdf"
        }.items()
    )

    # robot_state_publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": ParameterValue(
                Command(["xacro ", robot_description_path]),
                value_type=str
            )
        }],
        output="screen"
    )

    # Spawn entity using gazebo plugin (works with Ignition Gazebo 6)
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "kilin",
            "-topic", "robot_description"
        ],
        output="screen"
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])
