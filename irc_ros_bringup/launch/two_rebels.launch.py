from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
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

    # We need different controller configs for this setup
    robot_controller_file_1 = PathJoinSubstitution(
        [
            FindPackageShare("irc_ros_bringup"),
            "config",
            "controller_igus_rebel_6dof_namespace_1.yaml",
        ]
    )
    robot_controller_file_2 = PathJoinSubstitution(
        [
            FindPackageShare("irc_ros_bringup"),
            "config",
            "controller_igus_rebel_6dof_namespace_2.yaml",
        ]
    )
    rebel_1_nodes = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [irc_ros_bringup_launch_dir, "/rebel.launch.py"]
                ),
                launch_arguments={
                    "namespace": "/rebel_1",
                    "prefix": "rebel_1_",
                    "gripper": "none",
                    "launch_dashboard_controller": "false",
                    "launch_dio_controller": "false",
                    "use_rviz": "false",
                }.items(),
            )
        ],
        scoped=True,
    )
    rebel_2_nodes = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [irc_ros_bringup_launch_dir, "/rebel.launch.py"]
                ),
                launch_arguments={
                    "namespace": "/rebel_2",
                    "prefix": "rebel_2_",
                    "gripper": "none",
                    "launch_dashboard_controller": "false",
                    "launch_dio_controller": "false",
                    "use_rviz": "false",
                }.items(),
            )
        ],
        scoped=True,
    )

    description = LaunchDescription()
    description.add_action(rebel_1_nodes)
    description.add_action(rebel_2_nodes)
    return description
