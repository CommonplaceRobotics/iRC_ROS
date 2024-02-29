from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    irc_ros_bringup_launch_dir = PathJoinSubstitution(
        [
            FindPackageShare("irc_ros_bringup"),
            "launch",
        ]
    )
    default_rviz_file = PathJoinSubstitution(
        [FindPackageShare("irc_ros_description"), "rviz", "platform.rviz"]
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        choices=["0", "1", "false", "true", "False", "True"],
        description="Whether to start rviz with the launch file",
    )
    rviz_file_arg = DeclareLaunchArgument(
        "rviz_file",
        default_value=default_rviz_file,
        description="The path to the rviz configuration file",
    )
    use_rqt_robot_steering_arg = DeclareLaunchArgument(
        "use_rqt_robot_steering",
        default_value="false",
        choices=["0", "1", "false", "true", "False", "True"],
        description="Whether to start RqtRobotSteering with the launch file",
    )
    use_laserscanners_arg = DeclareLaunchArgument(
        "use_laserscanners",
        default_value="false",
        choices=["0", "1", "false", "true", "False", "True"],
        description="Whether to launch the laserscanner specific nodes",
    )

    hardware_protocol_arg = DeclareLaunchArgument(
        "hardware_protocol",
        default_value="cri",
        choices=["mock_hardware", "gazebo", "cprcanv2", "cri"],
        description="Which hardware protocol or mock hardware should be used",
    )

    use_rviz = LaunchConfiguration("use_rviz")
    rviz_file = LaunchConfiguration("rviz_file")

    # Use GroupActions for scoping the launch arguments, e.g. to not overwrite rviz arg
    rebel_stack = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [irc_ros_bringup_launch_dir, "/rebel.launch.py"]
                ),
                launch_arguments={
                    "namespace": "/rebel_1",
                    "prefix": "",
                    "controller_manager_name": "/rebel_1/controller_manager",
                    "gripper": "none",
                    "use_rviz": "false",
                    "launch_dashboard_controller": "false",
                    "launch_dio_controller": "true",
                }.items(),
            )
        ]
    )

    platform_stack = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [irc_ros_bringup_launch_dir, "/cpr_platform_debug.launch.py"]
                ),
                launch_arguments={
                    "namespace": "/platform_1",
                    "prefix": "",
                    "launch_dashboard_controller": "false",
                    "launch_dio_controller": "false",
                    "use_rviz": "false",
                    "use_rqt_robot_steering": "false",
                    "hardware_protocol": "cri",
                    "platform_name": "cpr_platform_mini",
                }.items(),
            )
        ]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_file],
        condition=IfCondition(use_rviz),
    )
    description = LaunchDescription()

    # Launch args
    description.add_action(use_rviz_arg)
    description.add_action(rviz_file_arg)
    description.add_action(use_rqt_robot_steering_arg)
    description.add_action(use_laserscanners_arg)
    description.add_action(hardware_protocol_arg)

    description.add_action(rebel_stack)
    description.add_action(platform_stack)

    # UI nodes
    description.add_action(rviz_node)
    return description
