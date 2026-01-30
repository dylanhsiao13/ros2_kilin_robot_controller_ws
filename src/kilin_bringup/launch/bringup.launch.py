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
        "kilin.urdf"
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
    "FL_Joint0", "FL_Joint1", "FL_Joint2", "FL_Joint3",
    "FR_Joint0", "FR_Joint1", "FR_Joint2", "FR_Joint3",
    "RL_Joint0", "RL_Joint1", "RL_Joint2", "RL_Joint3",
    "RR_Joint0", "RR_Joint1", "RR_Joint2", "RR_Joint3"
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
