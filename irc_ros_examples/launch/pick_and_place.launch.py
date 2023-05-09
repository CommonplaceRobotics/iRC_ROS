from pathlib import Path
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_param_builder import load_yaml
from nav2_common.launch import ReplaceString


def load_pnp(context, *args, **kwargs):
    joint_limits_file = PathJoinSubstitution(
        [
            FindPackageShare("irc_ros_moveit_config"),
            "config",
            "joint_limits.yaml",
        ]
    )
    joint_limits = ReplaceString(
        source_file=joint_limits_file,
        replacements={
            "<namespace>": "",
            "<prefix>": "",
        },
    )

    pick_and_place_node = Node(
        package="irc_ros_examples",
        executable="pick_and_place",
        name="pick_and_place",
        parameters=[
            load_yaml(Path(joint_limits.perform(context))),
        ],
    )

    return [pick_and_place_node]


def generate_launch_description():
    irc_ros_moveit_dir = PathJoinSubstitution(
        [
            FindPackageShare("irc_ros_moveit_config"),
            "launch",
        ]
    )

    moveit_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([irc_ros_moveit_dir, "/rebel.launch.py"]),
        launch_arguments={
            "gripper": "ext_dio_gripper",
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_entity(moveit_node)
    # ld.add_entity(pick_and_place_node)
    ld.add_action(OpaqueFunction(function=load_pnp))
    return ld
