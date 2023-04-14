from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node


def generate_launch_description():
    launch_dashboard_controller_arg = DeclareLaunchArgument(
        "launch_dashboard_controller", default_value="true"
    )
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
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("hardware_protocol"),
                    "' == 'cprcanv2' ",
                    "and '",
                    LaunchConfiguration("launch_dio_controller"),
                    "' in ['1', 'true', 'True']",
                ]
            )
        ),
    )

    external_dio_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        namespace=LaunchConfiguration("namespace"),
        arguments=[
            "external_dio_controller",
            "-c",
            LaunchConfiguration("controller_manager_name"),
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("hardware_protocol"),
                    "' == 'cprcanv2' ",
                    "and '",
                    LaunchConfiguration("gripper"),
                    "' == 'ext_dio_gripper' ",
                ]
            )
        ),
    )

    ecbpmi_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        namespace=LaunchConfiguration("namespace"),
        arguments=[
            "ecbpmi_controller",
            "-c",
            LaunchConfiguration("controller_manager_name"),
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("hardware_protocol"),
                    "' == 'cprcanv2' ",
                    "and '",
                    LaunchConfiguration("gripper"),
                    "' == 'schmalz_ecbpmi' ",
                ]
            )
        ),
    )

    dashboard_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        namespace=LaunchConfiguration("namespace"),
        arguments=[
            "dashboard_controller",
            "-c",
            LaunchConfiguration("controller_manager_name"),
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("hardware_protocol"),
                    "' == 'cprcanv2' ",
                    "and '",
                    LaunchConfiguration("launch_dashboard_controller"),
                    "' in ['1', 'true', 'True']",
                ]
            )
        ),
    )

    description = LaunchDescription()
    description.add_action(launch_dashboard_controller_arg)
    description.add_action(launch_dio_controller_arg)

    description.add_action(dio_controller_node)
    description.add_action(external_dio_controller_node)
    description.add_action(ecbpmi_controller_node)
    description.add_action(dashboard_controller_node)

    return description
