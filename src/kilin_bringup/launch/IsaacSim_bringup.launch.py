from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():



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

    # whole_body_controller (upper-level gait controller)
    whole_body_controller = Node(
        package="kilin_controller",
        executable="whole_body_controller",
        name="whole_body_controller",
        output="screen"
    )


    return LaunchDescription([
        
        joint_commander,
        whole_body_controller,
    ])
