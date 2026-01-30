from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # Path to URDF
    robot_description_path = PathJoinSubstitution([
        FindPackageShare("kilin_description"),
        "urdf",
        "AMRV2_only_0115_URDF.urdf"
    ])

    # robot_state_publisher node
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": ParameterValue(
                Command(["cat ", robot_description_path]),
                value_type=str
            )
        }]
    )

    # joint_commander for controlling joints
    joint_commander = Node(
        package="kilin_controller",
        executable="joint_commander",
        name="joint_commander",
        output="screen",
        parameters=[{
            "joint_names": [
    "FL_hip", "FL_steering", "FL_suspension", "FL_wheel",
    "FR_hip", "FR_steering", "FR_suspension", "FR_wheel",
    "RL_hip", "RL_steering", "RL_suspension", "RL_wheel",
    "RR_hip", "RR_steering", "RR_suspension", "RR_wheel",
]
        }]
    )

    # RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen"
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_commander,
        rviz
    ])
