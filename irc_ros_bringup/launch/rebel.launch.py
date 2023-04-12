from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument("namespace", default_value="")
    prefix_arg = DeclareLaunchArgument("prefix", default_value="")
    controller_manager_name_arg = DeclareLaunchArgument(
        "controller_manager_name", default_value="/controller_manager"
    )
    default_rviz_file = PathJoinSubstitution(
        [FindPackageShare("irc_ros_description"), "rviz", "rebel.rviz"]
    )
    default_urdf_filename_arg = DeclareLaunchArgument(
        "default_urdf_filename",
        default_value=[LaunchConfiguration("robot_name"), ".urdf.xacro"],
        description="Name of the robot description file",
    )
    default_urdf_file = PathJoinSubstitution(
        [
            FindPackageShare("irc_ros_description"),
            "urdf",
            LaunchConfiguration("default_urdf_filename"),
        ]
    )

    default_robot_controller_filename_arg = DeclareLaunchArgument(
        "default_robot_controller_filename",
        default_value=["controller_", LaunchConfiguration("robot_name"), ".yaml"],
        description="Name of the robot controller configuration file",
    )
    default_robot_controller_file = PathJoinSubstitution(
        [
            FindPackageShare("irc_ros_bringup"),
            "config",
            LaunchConfiguration("default_robot_controller_filename"),
        ]
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
    robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        default_value="igus_rebel_6dof",
        choices=["igus_rebel_6dof", "igus_rebel_4dof"],
        description="Which igus ReBeL type to use",
    )
    robot_urdf_arg = DeclareLaunchArgument(
        "robot_urdf",
        default_value=default_urdf_file,
        description="Path of the robot description file",
    )
    robot_controller_config_arg = DeclareLaunchArgument(
        "robot_controller_config",
        default_value=default_robot_controller_file,
        description="Path of the robot's description file",
    )
    rebel_version_arg = DeclareLaunchArgument(
        "rebel_version",
        default_value="01",
        choices=["pre", "00", "01"],
        description="Which version of the igus ReBeL to use",
    )
    gripper_arg = DeclareLaunchArgument(
        "gripper",
        default_value="none",
        choices=["none", "schmalz_ecbpmi", "ext_dio_gripper"],
        description="Which gripper to attach to the flange",
    )
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="The namespace to use for all nodes started by this launch file",
    )
    launch_dashboard_controller_arg = DeclareLaunchArgument(
        "launch_dashboard_controller", default_value="true"
    )
    launch_dio_controller_arg = DeclareLaunchArgument(
        "launch_dio_controller", default_value="true"
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
    rviz_file = LaunchConfiguration("rviz_file")
    robot_urdf = LaunchConfiguration("robot_urdf")
    robot_controller_config = LaunchConfiguration("robot_controller_config")
    rebel_version = LaunchConfiguration("rebel_version")
    gripper = LaunchConfiguration("gripper")
    hardware_protocol = LaunchConfiguration("hardware_protocol")

    # Not required as variables
    # robot_name = LaunchConfiguration('robot_name')
    # launch_dashboard_controller = LaunchConfiguration("launch_dashboard_controller")
    # launch_dio_controller = LaunchConfiguration("launch_dio_controller")

    robot_description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            robot_urdf,
            " rebel_version:=",
            rebel_version,
            " gripper:=",
            gripper,
            " prefix:=",
            prefix,
            " hardware_protocol:=",
            hardware_protocol,
        ]
    )

    external_dio_controllers = PathJoinSubstitution(
        [
            FindPackageShare("irc_ros_bringup"),
            "config",
            "controller_dio_module.yaml",
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
            robot_controller_config,
            external_dio_controllers,
        ],
        output="both",
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["joint_state_broadcaster", "-c", controller_manager_name],
        output="screen",
    )

    robot_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["joint_trajectory_controller", "-c", controller_manager_name],
    )

    dio_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["dio_controller", "-c", controller_manager_name],
        condition=LaunchConfigurationEquals("launch_dio_controller", "true"),
    )

    external_dio_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["external_dio_controller", "-c", controller_manager_name],
        condition=LaunchConfigurationEquals("gripper", "ext_dio_gripper"),
    )

    ecbpmi_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["ecbpmi_controller", "-c", controller_manager_name],
        condition=LaunchConfigurationEquals("gripper", "schmalz_ecbpmi"),
    )

    dashboard_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["dashboard_controller", "-c", controller_manager_name],
        condition=LaunchConfigurationEquals("launch_dashboard_controller", "true"),
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    # delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = (
    #     RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=joint_state_broadcaster,
    #             on_exit=[
    #                 robot_controller_node,
    #                 dio_controller_node,
    #                 external_dio_controller_node,
    #                 ecbpmi_controller_node,
    #                 dashboard_controller_node,
    #             ],
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

    description = LaunchDescription()

    # Launch args
    description.add_action(namespace_arg)
    description.add_action(prefix_arg)
    description.add_action(controller_manager_name_arg)
    description.add_action(robot_name_arg)
    description.add_action(default_urdf_filename_arg)
    description.add_action(default_robot_controller_filename_arg)
    description.add_action(use_rviz_arg)
    description.add_action(rviz_file_arg)
    description.add_action(robot_urdf_arg)
    description.add_action(robot_controller_config_arg)
    description.add_action(rebel_version_arg)
    description.add_action(gripper_arg)
    description.add_action(launch_dashboard_controller_arg)
    description.add_action(launch_dio_controller_arg)
    description.add_action(hardware_protocol_arg)

    # Robot nodes
    description.add_action(control_node)
    description.add_action(robot_state_pub)
    description.add_action(joint_state_pub)
    description.add_action(joint_state_broadcaster)

    # Robot nodes that were previously started after JSB
    description.add_action(robot_controller_node)
    description.add_action(dio_controller_node)
    description.add_action(external_dio_controller_node)
    description.add_action(ecbpmi_controller_node)
    description.add_action(dashboard_controller_node)

    #    description.add_action(
    #        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner
    #    )

    # UI nodes
    description.add_action(rviz_node)

    return description
