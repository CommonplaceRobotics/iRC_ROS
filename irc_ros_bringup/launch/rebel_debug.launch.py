from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit

from nav2_common.launch import ReplaceString

def generate_launch_description():
    namespace_arg = DeclareLaunchArgument("namespace", default_value="")
    prefix_arg = DeclareLaunchArgument("prefix", default_value="")
    controller_manager_name_arg = DeclareLaunchArgument(
        "controller_manager_name",
        default_value=[LaunchConfiguration("namespace"), "/controller_manager"],
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

    hardware_protocol_arg = DeclareLaunchArgument(
        "hardware_protocol",
        default_value="cri",
        choices=["mock_hardware", "gazebo", "cprcanv2", "cri"],
        description="Which hardware protocol or mock hardware should be used",
    )
    launch_dashboard_controller = DeclareLaunchArgument(
        "launch_dashboard_controller",
        default_value="false",
        choices=["0", "1", "false", "true", "False", "True"],
    )

    launch_dio_controller = DeclareLaunchArgument(
        "launch_dio_controller",
        default_value="true",
        choices=["0", "1", "false", "true", "False", "True"],
    )


    namespace = LaunchConfiguration("namespace")
    prefix = LaunchConfiguration("prefix")
    controller_manager_name = LaunchConfiguration("controller_manager_name")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_file = LaunchConfiguration("rviz_file")
    robot_urdf = LaunchConfiguration("robot_urdf")
    robot_controller_config_file = LaunchConfiguration("robot_controller_config")
    rebel_version = LaunchConfiguration("rebel_version")
    gripper = LaunchConfiguration("gripper")
    hardware_protocol = LaunchConfiguration("hardware_protocol")

    # Not required as variables
    # robot_name = LaunchConfiguration('robot_name')
    # launch_dashboard_controller = LaunchConfiguration("launch_dashboard_controller")
    # launch_dio_controller = LaunchConfiguration("launch_dio_controller")

    robot_controller_config = ReplaceString(
        source_file=robot_controller_config_file,
        replacements={
            "<namespace>": namespace,
            "<prefix>": prefix,
        }
    )

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

    external_dio_controllers_file = PathJoinSubstitution(
        [
            FindPackageShare("irc_ros_bringup"),
            "config",
            "controller_dio_module.yaml",
        ]
    )

    external_dio_controllers = ReplaceString(
        source_file=external_dio_controllers_file,
        replacements={
            "<namespace>": namespace,
            "<prefix>": prefix,
        }
    )


    # Node declarations:
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=namespace,
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
        prefix=["gdbserver localhost:3000"],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["joint_state_broadcaster", "-c", controller_manager_name],
    )

    # robot_controller_node = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     namespace=namespace,
    #     arguments=["joint_trajectory_controller", "-c", controller_manager_name],
    # )

    robot_controller_node = Node(
    package="controller_manager",
    executable="spawner",
    namespace=namespace,
    arguments=["joint_velocity_controller", "-c", controller_manager_name],
    )
    
    irc_ros_bringup_launch_dir = PathJoinSubstitution(
        [
            FindPackageShare("irc_ros_bringup"),
            "launch",
        ]
    )
    additional_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [irc_ros_bringup_launch_dir, "/additional_controllers.launch.py"]
        ),
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

    description.add_action(hardware_protocol_arg)

    # Robot nodes
    description.add_action(robot_state_pub)
    description.add_action(joint_state_pub)

    # ROS2 Control nodes
    description.add_action(control_node)
    description.add_action(joint_state_broadcaster)
    # Dont delay start of the following nodes after `joint_state_broadcaster` as the EventHandler
    # causes issues with LaunchConfigurations
    description.add_action(robot_controller_node)
    # description.add_action(additional_controllers)

    # UI nodes
    description.add_action(rviz_node)

    return description
