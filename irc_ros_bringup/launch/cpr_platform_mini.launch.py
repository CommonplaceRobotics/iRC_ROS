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
    PythonExpression,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString


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
        choices=["", "/platform_1"],
        description="The namespace to use for all nodes started by this launch file",
    )
    prefix_arg = DeclareLaunchArgument(
        "prefix",
        default_value="",
        choices=["", "platform_1_"],
        description="Prefix for joints and links",
    )
    controller_manager_name_arg = DeclareLaunchArgument(
        "controller_manager_name",
        default_value=[LaunchConfiguration("namespace"), "/controller_manager"],
        description="Name of the controller manager, changes with the namespace",
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
        default_value="cpr_platform_mini",
        choices=["cpr_platform_mini"],
        description="The product name of the mobile platform",
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
    use_rqt_robot_steering = LaunchConfiguration("use_rqt_robot_steering")
    # Not required as variable
    # platform_name = LaunchConfiguration('platform_name')
    platform_urdf = LaunchConfiguration("platform_urdf")
    platform_controller_config_file = LaunchConfiguration("platform_controller_config")
    # use_laserscanners = LaunchConfiguration("use_laserscanners")
    hardware_protocol = LaunchConfiguration("hardware_protocol")
    
    platform_controller_config = ReplaceString(
        source_file=platform_controller_config_file,
        replacements={
            "<namespace>": namespace,
            "<prefix>": prefix,
        },
    )
    robot_description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            platform_urdf,
            " prefix:=",
            prefix,
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
            platform_controller_config           
        ],
    )
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["joint_state_broadcaster", "-c", controller_manager_name],
    )

    robot_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["cpr_platform_controller", "-c", controller_manager_name],
    )

    irc_ros_bringup_launch_dir = PathJoinSubstitution(
    [
        FindPackageShare("irc_ros_bringup"),
        "launch",
    ]
    )

    
    additional_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [irc_ros_bringup_launch_dir, "/additional_controllers_pltfrm.launch.py"]
        ),
    )


    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_file],
        condition=IfCondition(use_rviz),
    )

    rqt_robot_steering_node = Node(
        package="rqt_robot_steering",
        executable="rqt_robot_steering",
        name="rqt_robot_steering",
        condition=IfCondition(use_rqt_robot_steering),
    )


    keyboard_ctrl_node = Node(
        package="irc_ros_examples",
        executable="keyboard_ctrl",
        name="keyboard_ctrl",
        # prefix=["gnome-terminal --command"],
        # prefix=["xterm -e"],
        #  output="screen",
    )
    
    # Sick S300 Laser scanners


    sicks300_2_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [irc_ros_bringup_launch_dir, "/sick_s300_2_two_scanners_merged.launch.py"]
        ),
        launch_arguments={
            "namespace": namespace,
            "prefix": prefix,
            # TODO: Temporary workaround since slam_toolbox settings dont seem to work
            "out_topic": "/scan",
        }.items(),
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("hardware_protocol"),
                    "' == 'cprcanv2' ",
                    "and '",
                    LaunchConfiguration("use_laserscanners"),
                    "' in ['1', 'true', 'True']",
                ]
            )
        ),
    )

    # Since the odometry topics from the diffdrive controllers output to frames with a
    # namespace instead the prefix we need tfs between those two if we use a namespace
    # TODO: Use namespace & prefix LaunchConfiguration instead of hardcoding it
    # TODO: Remove once https://github.com/ros-controls/ros2_controllers/pull/533 is merged
    odom_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="odom_tf_static_broadcaster",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "platform_1/platform_1_base_link",
            "platform_1_base_link",
        ],
    )

    base_link_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_tf_static_broadcaster",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "platform_1/platform_1_odom",
            "platform_1_odom",
        ],
    )
    description = LaunchDescription()

    # Launch args
    description.add_action(platform_name_arg)

    description.add_action(default_urdf_filename_arg)
    description.add_action(default_platform_controller_filename_arg)

    description.add_action(namespace_arg)
    description.add_action(prefix_arg)
    description.add_action(controller_manager_name_arg)

    description.add_action(use_rviz_arg)
    description.add_action(rviz_file_arg)
    description.add_action(use_rqt_robot_steering_arg)
    
    description.add_action(platform_urdf_arg)
    description.add_action(platform_controller_config_arg)
    description.add_action(use_laserscanners_arg)
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
    description.add_action(additional_controllers)


    # UI nodes
    description.add_action(rviz_node)
    description.add_action(rqt_robot_steering_node)
    #description.add_action(rqt_service_caller_node)
    

    #description.add_action(keyboard_ctrl_node)    
    # Laser scan front
    description.add_action(sicks300_2_stack)

    # Diff drive tfs
    description.add_action(odom_tf_node)
    description.add_action(base_link_tf_node)

    return description
