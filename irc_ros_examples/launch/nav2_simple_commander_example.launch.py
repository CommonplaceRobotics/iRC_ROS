from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    irc_ros_nav2_dir = PathJoinSubstitution(
        [
            FindPackageShare("irc_ros_navigation2"),
            "launch",
        ]
    )

    nav2_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([irc_ros_nav2_dir, "/cpr_platform.launch.py"]),
    )

    nav2_sc_node = Node(
        package="irc_ros_examples",
        executable="nav2_simple_commander_example.py",
        name="nav2_sc",
    )

    ld = LaunchDescription()
    ld.add_entity(nav2_node)
    ld.add_entity(nav2_sc_node)

    return ld
