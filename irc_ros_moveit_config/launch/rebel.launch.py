# Based on the panda moveit launch file from ros-planning
# https://github.com/ros-planning/moveit_resources/blob/humble/panda_moveit_config/launch/demo.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_param_builder import load_yaml
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
    gripper_arg = DeclareLaunchArgument(
        "gripper",
        default_value="none",
        choices=["none", "schmalz_ecbpmi", "ext_dio_gripper"],
        description="Which gripper to attach to the flange",
    )
    launch_dashboard_controller_arg = DeclareLaunchArgument(
        "launch_dashboard_controller",
        default_value="false",
        description="NOTE: Requires adding settings to ros2_controllers.yaml",
    )
    launch_dio_controller_arg = DeclareLaunchArgument(
        "launch_dio_controller",
        default_value="true",
        description="Whether to launch the DIO controller",
    )
    hardware_protocol_arg = DeclareLaunchArgument(
        "hardware_protocol",
        default_value="cprcanv2",
        choices=["mock_hardware", "gazebo", "cprcanv2", "cri"],
        description="Which hardware protocol or mock hardware should be used",
    )
    use_rviz = LaunchConfiguration("use_rviz")
    # gripper = LaunchConfiguration("gripper")
    namespace = LaunchConfiguration("namespace")
    prefix = LaunchConfiguration("prefix")
    controller_manager_name = LaunchConfiguration("controller_manager_name")

    rviz_file = PathJoinSubstitution(
        [FindPackageShare("irc_ros_moveit_config"), "rviz", "moveit.rviz"]
    )

    ros2_controllers_file = PathJoinSubstitution(
        [
            FindPackageShare("irc_ros_moveit_config"),
            "config",
            "ros2_controllers.yaml",
        ]
    )
    ros2_controllers = ReplaceString(
        source_file=ros2_controllers_file,
        replacements={
            "<namespace>": namespace,
            "<prefix>": prefix,
        },
    )

    robot_description_file = PathJoinSubstitution(
        [
            FindPackageShare("irc_ros_description"),
            "urdf",
            "igus_rebel_6dof.urdf.xacro",
        ]
    )
    robot_description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            robot_description_file,
            " prefix:=",
            prefix,
            " hardware_protocol:=",
            "cprcanv2",
        ]
    )
    robot_description_semantic_file = PathJoinSubstitution(
        [
            FindPackageShare("irc_ros_moveit_config"),
            "config",
            "igus_rebel_6dof.srdf.xacro",
        ]
    )
    robot_description_semantic = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            robot_description_semantic_file,
            " prefix:=",
            prefix,
        ]
    )

    # Requires the os.path.join way instead of PathJoinSubstitution here
    controllers_file = os.path.join(
        get_package_share_directory("irc_ros_moveit_config"),
        "config",
        "controllers.yaml",
    )
    # TODO: This needs to return a path or the line below it wont accept it
    # Until then the namespace/prefix replacements won't work
    # controllers = ReplaceString(
    #     source_file=controllers_file,
    #     replacements={
    #         "<namespace>": namespace,
    #         "<prefix>": prefix,
    #     }
    # )
    controllers_dict = load_yaml(Path(controllers_file))

    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_dict,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    robot_description_kinematics_file = PathJoinSubstitution(
        [
            FindPackageShare("irc_ros_moveit_config"),
            "config",
            "kinematics.yaml",
        ]
    )
    robot_description_kinematics = ReplaceString(
        source_file=robot_description_kinematics_file,
        replacements={
            "<namespace>": namespace,
            "<prefix>": prefix,
        },
    )

    # TODO: Stolen from UR ROS2 for testing purposes:
    planning_pipeline = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        parameters=[
            {"robot_description": robot_description},
            {"robot_description_semantic": robot_description_semantic},
            robot_description_kinematics,
            planning_pipeline,
            moveit_controllers,
            {"publish_robot_description_semantic": True},
            planning_scene_monitor_parameters,
        ],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            ros2_controllers,
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        # namespace=namespace,
        parameters=[
            {"robot_description": robot_description},
        ],
    )

    joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", controller_manager_name],
    )

    rebel_6dof_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rebel_6dof_controller", "-c", "/controller_manager"],
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

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_node,
                on_exit=[
                    rebel_6dof_controller_node,
                    additional_controllers,
                ],
            )
        )
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_file],
        parameters=[
            {"robot_description": robot_description},
            {"robot_description_semantic": robot_description_semantic},
            planning_pipeline,
            robot_description_kinematics,
        ],
        condition=IfCondition(use_rviz),
    )
    ld = LaunchDescription()
    ld.add_action(namespace_arg)
    ld.add_action(prefix_arg)
    ld.add_action(controller_manager_name_arg)
    ld.add_action(use_rviz_arg)
    ld.add_action(gripper_arg)
    ld.add_action(launch_dashboard_controller_arg)
    ld.add_action(launch_dio_controller_arg)
    ld.add_action(hardware_protocol_arg)

    # Robot nodes
    ld.add_action(move_group_node)
    ld.add_action(control_node)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_broadcaster_node)
    ld.add_action(delay_robot_controller_spawner_after_joint_state_broadcaster_spawner)

    # UI nodes
    ld.add_action(rviz_node)
    return ld


#
