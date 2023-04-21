from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_file = PathJoinSubstitution(
        [FindPackageShare("irc_ros_navigation2"), "rviz", "platform.rviz"]
    )

    base_launch_file_arg = DeclareLaunchArgument(
        "base_launch_file",
        default_value=["cpr_platform.launch.py"],
        choices=["cpr_platform.launch.py", "rebel_on_platform.launch.py"],
        description="Name of the robot description file",
    )

    base_launch_file = PathJoinSubstitution(
        [
            FindPackageShare("irc_ros_bringup"),
            "launch",
            LaunchConfiguration("base_launch_file"),
        ]
    )

    irc_ros_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch_file),
        launch_arguments={
            "use_rviz": "true",
            "rviz_file": rviz_file,
        }.items(),
    )

    params_file = PathJoinSubstitution(
        [
            FindPackageShare("irc_ros_navigation2"),
            "params",
            "cpr_platform_medium.yaml",
        ]
    )

    map_file = PathJoinSubstitution(
        [
            FindPackageShare("irc_ros_navigation2"),
            "map",
            "map.yaml",
        ]
    )

    nav2_launch_file_dir = PathJoinSubstitution(
        [
            FindPackageShare("nav2_bringup"),
            "launch",
        ]
    )

    nav2_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch_file_dir, "/bringup_launch.py"]),
        launch_arguments={
            # While we use SLAM and dont use this map, it is required to pass this argument
            "map": map_file,
            "params_file": params_file,
            "slam": "True",
        }.items(),
    )

    return LaunchDescription(
        [
            base_launch_file_arg,
            irc_ros_stack,
            # /cmd_vel (comes from either rqt_robot_steering, Navigation2 goal_pose)
            # ->
            # /cpr_platform_controller/cmd_vel_unstamped (ros2_control input)
            SetRemap(src="/cmd_vel", dst="/cpr_platform_controller/cmd_vel_unstamped"),
            # SetRemap(src="amcl/get_state", dst="/amcl/get_state"),
            nav2_stack,
        ]
    )
