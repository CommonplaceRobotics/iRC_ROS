from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


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
            "gripper": "schmalz_ecbpmi",
        }.items(),
    )

    # Publish tray frames
    tray1_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tray_1_static_broadcaster",
        arguments=["0.209", "-0.090", "0.318", "0", "0", "0", "base_link", "tray_1"],
    )

    tray2_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tray_2_static_broadcaster",
        arguments=["0.209", "-0.015", "0.318", "0", "0", "0", "base_link", "tray_2"],
    )

    pick_and_place_node = Node(
        package="irc_ros_examples",
        executable="pick_and_place_vacuum",
        name="pick_and_place_vacuum",
    )

    ld = LaunchDescription()
    ld.add_entity(moveit_node)
    ld.add_entity(tray1_tf_node)
    ld.add_entity(tray2_tf_node)
    ld.add_entity(pick_and_place_node)

    return ld
