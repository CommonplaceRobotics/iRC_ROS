from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString


def generate_launch_description():
    destination_frame_arg = DeclareLaunchArgument(
        "destination_frame_arg",
        default_value=[LaunchConfiguration("prefix"), "base_link"],
        description="The frame to which the scans are transformed",
    )
    in_topics_arg = DeclareLaunchArgument(
        "in_topics",
        default_value=[
            LaunchConfiguration("namespace"),
            "/scan_front",
            " ",
            LaunchConfiguration("namespace"),
            "/scan_back",
        ],
        description="The topics from which to read the scans",
    )
    out_topic_arg = DeclareLaunchArgument(
        "out_topic",
        default_value=[LaunchConfiguration("namespace"), "/scan"],
        description="The topic where the transformed and merged scans are published",
    )
    namespace = LaunchConfiguration("namespace")
    prefix = LaunchConfiguration("prefix")

    destination_frame = LaunchConfiguration("destination_frame_arg")
    in_topics = LaunchConfiguration("in_topics")
    out_topic = LaunchConfiguration("out_topic")

    # Sick S300 Laser scanners
    irc_ros_bringup_launch_dir = PathJoinSubstitution(
        [
            FindPackageShare("irc_ros_bringup"),
            "launch",
        ]
    )

    sick_s300_params_file = PathJoinSubstitution(
        [
            FindPackageShare("irc_ros_bringup"),
            "params",
            "sick_s300.yaml",
        ]
    )

    sick_s300_params = ReplaceString(
        source_file=sick_s300_params_file,
        replacements={
            "<namespace>": namespace,
            "<prefix>": prefix,
        },
    )

    sicks300_2_stack_front = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [irc_ros_bringup_launch_dir, "/sick_s300_2.launch.py"]
        ),
        launch_arguments={
            "laserscanner_name": "laserscanner_front",
            "namespace": namespace,
            "params_file": sick_s300_params,
        }.items(),
    )

    sicks300_2_stack_back = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [irc_ros_bringup_launch_dir, "/sick_s300_2.launch.py"]
        ),
        launch_arguments={
            "laserscanner_name": "laserscanner_back",
            "namespace": namespace,
            "params_file": sick_s300_params,
        }.items(),
    )
    laser_merger_node = Node(
        package="ira_laser_tools",
        namespace=namespace,
        executable="laserscan_multi_merger",
        name="laserscan_multi_merger",
        parameters=[
            {
                "destination_frame": destination_frame,
                "scan_destination_topic": out_topic,
                "laserscan_topics": in_topics,
            }
        ],
    )

    description = LaunchDescription()
    description.add_action(destination_frame_arg)
    description.add_action(in_topics_arg)
    description.add_action(out_topic_arg)

    # description.add_action(namespace_arg)
    # description.add_action(prefix_arg)

    description.add_action(sicks300_2_stack_front)
    description.add_action(sicks300_2_stack_back)

    description.add_action(laser_merger_node)

    return description
