import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    # Path to URDF
    robot_description_path = PathJoinSubstitution([
        FindPackageShare("kilin_description"),
        "urdf",
        "URDF.urdf"
    ])

    # robot_state_publisher - publishes TF transforms
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

    # RViz for visualization
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare("kilin_description"),
        "rviz",
        "kilin.rviz"
    ])
    
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_path],
        output="screen"
    )

    return LaunchDescription([
        robot_state_publisher,
        rviz
    ])
