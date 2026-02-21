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
    base_attitude_controller = Node(
        package="kilin_controller",
        executable="base_attitude_controller",
        name="base_attitude_controller",
        output="screen"
    )
    base_attitude_gui = Node(
        package="kilin_controller",
        executable="base_attitude_gui",         
        name="base_attitude_gui",
        output="screen"
    )
    whole_body_controller = Node(
        package="kilin_controller",
        executable="whole_body_controller",
        name="whole_body_controller",
        output="screen"
    )
    joint_trajectory_planner = Node(
        package="kilin_controller",
        executable="joint_trajectory_planner",
        name="joint_trajectory_planner",
        output="screen"
    )
    custom_controller = Node(
        package="kilin_controller",
        executable="custom_controller",
        name="custom_controller",
        output="screen"
    )
    custom_controller_gui = Node(
        package="kilin_controller",
        executable="custom_controller_gui",
        name="custom_controller_gui",
        output="screen"
    )
    joint_state_plotter = Node(
        package="kilin_controller",
        executable="joint_state_plotter",
        name="joint_state_plotter",
        output="screen"
    )
    state_estimator = Node(
        package="kilin_controller",
        executable="state_estimator",
        name="state_estimator",                         
        output="screen"
    )
    state_estimation_visualizer = Node(
        package="kilin_controller",
        executable="state_estimation_visualizer",
        name="state_estimation_visualizer",
        output="screen"
    )

    return LaunchDescription([
        
        joint_commander,
        #whole_body_controller,
        state_estimator,
        state_estimation_visualizer,
        base_attitude_controller,
        base_attitude_gui,
        joint_trajectory_planner,
        joint_state_plotter,
        #custom_controller,
        #custom_controller_gui,
      
        

    ])
