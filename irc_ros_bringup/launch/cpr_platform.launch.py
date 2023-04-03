from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_rviz_file = PathJoinSubstitution(
        [FindPackageShare("irc_ros_description"), "rviz", "platform.rviz"]
    )
    default_urdf_filename_arg = DeclareLaunchArgument(
        "default_urdf_filename",
        default_value=[LaunchConfiguration("platform_name"), ".urdf.xacro"],
        description="Name of the robot description file",
    )
    default_urdf_file = PathJoinSubstitution(
        [
            FindPackageShare("irc_ros_description"),
            "urdf",
            LaunchConfiguration("default_urdf_filename"),
        ]
    )
    default_platform_controller_filename_arg = DeclareLaunchArgument(
        "default_platform_controller_filename",
        default_value=["controller_", LaunchConfiguration("platform_name"), ".yaml"],
        description="Name of the controller configuration",
    )
    default_platform_controller_file = PathJoinSubstitution(
        [
            FindPackageShare("irc_ros_bringup"),
            "config",
            LaunchConfiguration("default_platform_controller_filename"),
        ]
    )

    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="The namespace to use for all nodes started by this launch file",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
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
        default_value="true",
        choices=["0", "1", "false", "true", "False", "True"],
        description="Whether to start RqtRobotSteering with the launch file",
    )
    platform_name_arg = DeclareLaunchArgument(
        "platform_name",
        default_value="cpr_platform_medium",
        choices=["cpr_platform_medium"],
        description="The product name of the mobile platform. Currently only one model is available",
    )
    platform_urdf_arg = DeclareLaunchArgument(
        "platform_urdf",
        default_value=default_urdf_file,
        description="Path of the platform's robot description file",
    )
    platform_controller_config_arg = DeclareLaunchArgument(
        "platform_controller_config",
        default_value=default_platform_controller_file,
        description="Path of the platform's controller file",
    )
    use_laserscanners_arg = DeclareLaunchArgument(
        "use_laserscanners",
        default_value="true",
        choices=["0", "1", "false", "true", "False", "True"],
        description="Whether to launch the laserscanner specific nodes",
    )
    hardware_protocol_arg = DeclareLaunchArgument(
        "hardware_protocol", default_value="cprcanv2",
        choices=["mock_hardware", "gazebo", "cprcanv2", "cri"],
        description="TODO",
    )

    use_rviz = LaunchConfiguration("use_rviz")
    rviz_file = LaunchConfiguration("rviz_file")
    use_rqt_robot_steering = LaunchConfiguration("use_rqt_robot_steering")
    # Not required as variable
    # platform_name = LaunchConfiguration('platform_name')
    platform_urdf = LaunchConfiguration("platform_urdf")
    platform_controller_config = LaunchConfiguration("platform_controller_config")
    use_laserscanners = LaunchConfiguration("use_laserscanners")
    namespace = LaunchConfiguration("namespace")
    hardware_protocol = LaunchConfiguration("hardware_protocol")

    robot_description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            platform_urdf,
            " hardware_protocol:=",
            hardware_protocol,
        ]
    )

    # Node declarations:
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=namespace,
        output="screen",
        parameters=[{"robot_description": robot_description}],
        # arguments=['--ros-args', '--log-level', 'DEBUG'],
    )
    joint_state_pub = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        namespace=namespace,
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
        namespace=namespace,
        parameters=[
            {"robot_description": robot_description},
            platform_controller_config,
        ],
        output="both",
    )
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
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
        namespace=namespace,
        arguments=["cpr_platform_controller", "-c", "/controller_manager"],
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    # delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = (
    #     RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=joint_state_broadcaster,
    #             on_exit=[robot_controller_node],
    #         )
    #     )
    # )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_file],
        condition=IfCondition(use_rviz),
    )

    rqt_robot_steering_node = Node(
        package="rqt_robot_steering",
        executable="rqt_robot_steering",
        name="rqt_robot_steering",
        condition=IfCondition(use_rqt_robot_steering),
    )

    # Sick S300 Laser scanners
    irc_ros_bringup_launch_dir = PathJoinSubstitution(
        [
            FindPackageShare("irc_ros_bringup"),
            "launch",
        ]
    )

    sick_s300_params = PathJoinSubstitution(
        [
            FindPackageShare("irc_ros_bringup"),
            "params",
            "sick_s300.yaml",
        ]
    )

    sicks300_2_stack_front = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [irc_ros_bringup_launch_dir, "/sick_s300_2.launch.py"]
        ),
        launch_arguments={
            "laserscanner_name": "laserscanner_front",
            "params_file": sick_s300_params,
        }.items(),
        condition=IfCondition(use_laserscanners),
    )

    sicks300_2_stack_back = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [irc_ros_bringup_launch_dir, "/sick_s300_2.launch.py"]
        ),
        launch_arguments={
            "laserscanner_name": "laserscanner_back",
            "params_file": sick_s300_params,
        }.items(),
        condition=IfCondition(use_laserscanners),
    )
    laser_merger_node = Node(
        package="ira_laser_tools",
        namespace=namespace,
        executable="laserscan_multi_merger",
        name="laserscan_multi_merger",
        parameters=[
            {
                "destination_frame": "base_link",
                "scan_destination_topic": "/scan",
                "laserscan_topics": "/scan_front /scan_back",
            }
        ],
        condition=IfCondition(use_laserscanners),
    )

    description = LaunchDescription()

    # Launch args
    description.add_action(platform_name_arg)

    description.add_action(default_urdf_filename_arg)
    description.add_action(default_platform_controller_filename_arg)

    description.add_action(namespace_arg)
    description.add_action(use_rviz_arg)
    description.add_action(rviz_file_arg)
    description.add_action(use_rqt_robot_steering_arg)
    description.add_action(platform_urdf_arg)
    description.add_action(platform_controller_config_arg)
    description.add_action(use_laserscanners_arg)
    description.add_action(hardware_protocol_arg)

    # Robot nodes
    description.add_action(control_node)
    description.add_action(robot_state_pub)
    description.add_action(joint_state_pub)
    description.add_action(joint_state_broadcaster)
    description.add_action(robot_controller_node)
    # description.add_action(
    #    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner
    # )

    # UI nodes
    description.add_action(rviz_node)
    description.add_action(rqt_robot_steering_node)

    # Laser scan front
    description.add_action(sicks300_2_stack_front)

    # Laser scan back
    description.add_action(sicks300_2_stack_back)

    # Laser scan merger
    description.add_action(laser_merger_node)

    return description
