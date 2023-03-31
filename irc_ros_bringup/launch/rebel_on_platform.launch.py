from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
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
    default_urdf_filename_arg = DeclareLaunchArgument(
        "default_urdf_filename",
        default_value=["rebel_on_platform.urdf.xacro"],
        description="Name of the robot description file",
    )
    default_urdf_file = PathJoinSubstitution(
        [
            FindPackageShare("irc_ros_bringup"),
            "launch",
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
    default_platform_controller_filename_arg = DeclareLaunchArgument(
        "default_platform_controller_filename",
        default_value=["controller_", LaunchConfiguration("platform_name"), ".yaml"],
        description="Name of the platform controller configuration file",
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
        description="Path of the robot description file",
    )
    platform_controller_config_arg = DeclareLaunchArgument(
        "platform_controller_config",
        default_value=default_platform_controller_file,
        description="Path of the platform's controller file",
    )
    robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        default_value="igus_rebel_6dof",
        choices=["igus_rebel_6dof", "igus_rebel_4dof"],
        description="The product name of the robot. Currently only igus rebels are supported on the platform",
    )
    robot_controller_config_arg = DeclareLaunchArgument(
        "robot_controller_config",
        default_value=default_robot_controller_file,
        description="Path of the robot's description file",
    )
    use_laserscanners_arg = DeclareLaunchArgument(
        "use_laserscanners",
        default_value="true",
        choices=["0", "1", "false", "true", "False", "True"],
        description="Whether to launch the laserscanner specific nodes",
    )

    use_rqt_robot_steering = LaunchConfiguration("use_rqt_robot_steering")
    robot_controller_config = LaunchConfiguration("robot_controller_config")
    use_laserscanners = LaunchConfiguration("use_laserscanners")

    rebel_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([irc_ros_bringup_launch_dir, "/rebel.launch.py"]),
        launch_arguments={
            "namespace": "/rebel_1",
            "prefix": "rebel_1_",
            "controller_manager_name": "/rebel_1/controller_manager",
            "gripper": "none",
            "robot_controller_config": robot_controller_config,
            "launch_dashboard_controller": "false",
            "launch_dio_controller": "false",
            "use_rviz": "false",
        }.items(),
    )

    platform_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [irc_ros_bringup_launch_dir, "/cpr_platform.launch.py"]
        ),
        launch_arguments={
            "namespace": "",
            # "namespace": "/platform",
            # "prefix": "platform_",
            # "controller_manager_name" : "/platform/controller_manager",
            # "platform_controller_config": platform_controller_config,
            "launch_dashboard_controller": "false",
            "launch_dio_controller": "false",
            "use_laserscanners": use_laserscanners,
            "use_rviz": "false",
            "use_rqt_robot_steering": use_rqt_robot_steering,
        }.items(),
    )

    description = LaunchDescription()

    # Launch args
    description.add_action(platform_name_arg)
    description.add_action(robot_name_arg)
    
    description.add_action(default_urdf_filename_arg)
    description.add_action(default_robot_controller_filename_arg)
    description.add_action(default_platform_controller_filename_arg)

    description.add_action(namespace_arg)
    description.add_action(use_rviz_arg)
    description.add_action(rviz_file_arg)
    description.add_action(use_rqt_robot_steering_arg)
    description.add_action(platform_urdf_arg)
    description.add_action(platform_controller_config_arg)
    description.add_action(robot_controller_config_arg)
    description.add_action(use_laserscanners_arg)

    description.add_action(rebel_stack)
    description.add_action(platform_stack)
    return description
