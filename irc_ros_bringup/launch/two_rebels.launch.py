from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    irc_ros_bringup_launch_dir = PathJoinSubstitution(
        [
            FindPackageShare("irc_ros_bringup"),
            "launch",
        ]
    )

    # We need a different controller config for this setup
    robot_controller_file_1 = PathJoinSubstitution(
        [
            FindPackageShare("irc_ros_bringup"),
            "config",
            "controller_two_igus_rebel_6dof_first.yaml",
        ]
    )
    robot_controller_file_2 = PathJoinSubstitution(
        [
            FindPackageShare("irc_ros_bringup"),
            "config",
            "controller_two_igus_rebel_6dof_second.yaml",
        ]
    )
    rebel_1_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([irc_ros_bringup_launch_dir, "/rebel.launch.py"]),
        launch_arguments={
            "namespace": "/rebel_1",
            "prefix": "rebel_1_",
            "controller_manager_name": "/rebel_1/controller_manager",
            "gripper": "none",
            "robot_controller_config": robot_controller_file_1,
            "launch_dashboard_controller": "false",
            "launch_dio_controller": "false",
            "use_rviz": "false",
        }.items(),
    )
    rebel_2_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([irc_ros_bringup_launch_dir, "/rebel.launch.py"]),
        launch_arguments={
            "namespace": "/rebel_2",
            "prefix": "rebel_2_",
            "controller_manager_name": "/rebel_2/controller_manager",
            "gripper": "none",
            "robot_controller_config": robot_controller_file_2,
            "launch_dashboard_controller": "false",
            "launch_dio_controller": "false",
            "use_rviz": "false",
        }.items(),
    )
    description = LaunchDescription()
    description.add_action(rebel_1_nodes)
    description.add_action(rebel_2_nodes)
    return description