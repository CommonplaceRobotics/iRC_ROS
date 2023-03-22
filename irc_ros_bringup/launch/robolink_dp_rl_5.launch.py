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


def generate_launch_description():
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        choices=["0", "1", "false", "true", "False", "True"],
    )
    use_rviz = LaunchConfiguration("use_rviz")

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
            " use_cprcanv2:=true"
        ]
    )

    rviz_file = PathJoinSubstitution(
        [FindPackageShare("irc_ros_description"), "rviz", "rebel.rviz"]
    )

    igus_rebel_controllers = PathJoinSubstitution(
        [
            FindPackageShare("irc_ros_bringup"),
            "config",
            "controller_igus_robolink_rl_dp_5.yaml",
        ]
    )

    # Node declarations:
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
        # arguments=['--ros-args', '--log-level', 'DEBUG'],
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
        parameters=[{"robot_description": robot_description}, igus_rebel_controllers],
        output="both",
    )
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        # arguments=['joint_state_broadcaster'],
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    robot_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
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
        output="log",
        arguments=["-d", rviz_file],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            use_rviz_arg,
            control_node,
            robot_state_pub,
            joint_state_pub,
            joint_state_broadcaster,
            delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
            rviz_node,
        ]
    )
