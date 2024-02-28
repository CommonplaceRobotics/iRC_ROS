from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node


def generate_launch_description():

    launch_dio_controller_arg = DeclareLaunchArgument(
        "launch_dio_controller", default_value="true"
    )
    dio_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        namespace=LaunchConfiguration("namespace"),
        arguments=[
            "dio_controller",
            "-c",
            LaunchConfiguration("controller_manager_name"),
        ],
    )
    description = LaunchDescription()
    description.add_action(launch_dio_controller_arg)

    description.add_action(dio_controller_node)


    return description
