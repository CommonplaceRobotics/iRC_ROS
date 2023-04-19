from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from nav2_common.launch import ReplaceString


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument("namespace", default_value="")
    prefix_arg = DeclareLaunchArgument("prefix", default_value="")
    controller_manager_name_arg = DeclareLaunchArgument(
        "controller_manager_name",
        default_value=[LaunchConfiguration("namespace"), "/controller_manager"],
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        choices=["0", "1", "false", "true", "False", "True"],
        description="Whether to start rviz with the launch file",
    )
    hardware_protocol_arg = DeclareLaunchArgument(
        "hardware_protocol",
        default_value="cprcanv2",
        choices=["mock_hardware", "gazebo", "cprcanv2", "cri"],
        description="Which hardware protocol or mock hardware should be used",
    )

    namespace = LaunchConfiguration("namespace")
    prefix = LaunchConfiguration("prefix")
    controller_manager_name = LaunchConfiguration("controller_manager_name")
    use_rviz = LaunchConfiguration("use_rviz")
    hardware_protocol = LaunchConfiguration("hardware_protocol")

    xacro_file = PathJoinSubstitution(
        [
            FindPackageShare("irc_ros_description"),
            "urdf",
            "igus_robolink_rl_dp_5.urdf.xacro",
        ]
    )

    robot_description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            xacro_file,
            " prefix:=",
            prefix,
            " hardware_protocol:=",
            hardware_protocol,
        ]
    )

    rviz_file = PathJoinSubstitution(
        [FindPackageShare("irc_ros_description"), "rviz", "rebel.rviz"]
    )

    robot_controller_config_file = PathJoinSubstitution(
        [
            FindPackageShare("irc_ros_bringup"),
            "config",
            "controller_igus_robolink_rl_dp_5.yaml",
        ]
    )

    robot_controller_config = ReplaceString(
        source_file=robot_controller_config_file,
        replacements={
            "<namespace>": namespace,
            "<prefix>": prefix,
        },
    )

    # Node declarations:
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )
    joint_state_pub = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[
            {
                "source_list": [
                    "/joint_states",
                ],
                "rate": 30,
            }
        ],
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description}, robot_controller_config],
    )
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", controller_manager_name],
    )

    robot_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", controller_manager_name],
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[robot_controller_node],
            )
        )
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_file],
        condition=IfCondition(use_rviz),
    )

    description = LaunchDescription()
    description.add_action(namespace_arg)
    description.add_action(prefix_arg)
    description.add_action(controller_manager_name_arg)
    description.add_action(use_rviz_arg)
    description.add_action(hardware_protocol_arg)
    description.add_action(control_node)
    description.add_action(robot_state_pub)
    description.add_action(joint_state_pub)
    description.add_action(joint_state_broadcaster)
    description.add_action(
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner
    )
    description.add_action(rviz_node)
    return description
