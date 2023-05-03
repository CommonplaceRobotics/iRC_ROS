# Based on the panda moveit launch file from ros-planning
# https://github.com/ros-planning/moveit_resources/blob/humble/panda_moveit_config/launch/demo.launch.py
# and the moveit_py example
# https://github.com/peterdavidfagan/moveit2_tutorials/blob/moveit_py_motion_planning_python_api_tutorial/doc/examples/motion_planning_python_api/launch/motion_planning_python_api_tutorial.launch.py

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_param_builder import load_yaml
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString


def opaque_test(context, *args, **kwargs):
    use_rviz = LaunchConfiguration("use_rviz")
    gripper = LaunchConfiguration("gripper")
    namespace = LaunchConfiguration("namespace")
    prefix = LaunchConfiguration("prefix")
    controller_manager_name = LaunchConfiguration("controller_manager_name")
    hardware_protocol = LaunchConfiguration("hardware_protocol")
    rebel_version = LaunchConfiguration("rebel_version")

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

    joint_limits_file = PathJoinSubstitution(
        [
            FindPackageShare("irc_ros_moveit_config"),
            "config",
            "joint_limits.yaml",
        ]
    )
    joint_limits = ReplaceString(
        source_file=joint_limits_file,
        replacements={
            "<namespace>": namespace,
            "<prefix>": prefix,
        },
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
            hardware_protocol,
            " gripper:=",
            gripper,
            " rebel_version:=",
            rebel_version,
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

    controllers = ReplaceString(
        source_file=controllers_file,
        replacements={
            "<namespace>": namespace,
            "<prefix>": prefix,
        },
    )
    controllers_dict = load_yaml(Path(controllers.perform(context)))

    ompl_file = PathJoinSubstitution(
        [
            FindPackageShare("irc_ros_moveit_config"),
            "config",
            "ompl.yaml",
        ]
    )
    ompl = {"ompl": load_yaml(Path(ompl_file.perform(context)))}

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

    planning_pipeline = {
        "move_group": {
            # TODO: Copied from UR ROS2 for testing purposes, update configuration for the rebel
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        },
        # "move_group": {
        #     "planning_plugin": "pilz_industrial_motion_planner/CommandPlanner",
        #     "request_adapters": "default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints",
        #     "default_planner_config": "PTP",
        #     "capabilities": "pilz_industrial_motion_planner/MoveGroupSequenceAction pilz_industrial_motion_planner/MoveGroupSequenceService",
        # },
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    moveit_args_not_concatenated = [
        {"robot_description": robot_description.perform(context)},
        {"robot_description_semantic": robot_description_semantic.perform(context)},
        load_yaml(Path(robot_description_kinematics.perform(context))),
        load_yaml(Path(joint_limits.perform(context))),
        moveit_controllers,
        planning_scene_monitor_parameters,
        planning_pipeline,
        {
            "publish_robot_description": True,
            "publish_robot_description_semantic": True,
            "publish_geometry_updates": True,
            "publish_state_updates": True,
            "publish_transforms_updates": True,
        },
        ompl,
        # {"planning_pipeline": {"planning_plugin": "ompl_rrt_star"}},
    ]

    # Concatenate all dictionaries together, else moveitpy won't read all parameters
    moveit_args = dict()
    for d in moveit_args_not_concatenated:
        moveit_args.update(d)

    print(moveit_args)

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=namespace,
        parameters=[
            moveit_args,
        ],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=namespace,
        parameters=[
            moveit_args,
            ros2_controllers,
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=namespace,
        parameters=[
            moveit_args,
        ],
    )

    joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["joint_state_broadcaster", "-c", controller_manager_name],
    )

    rebel_6dof_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["rebel_6dof_controller", "-c", controller_manager_name],
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
        parameters=[
            # Passing the entire dict to rviz results in an error with the joint limits
            {"robot_description": robot_description},
        ],
        condition=IfCondition(use_rviz),
    )

    return [
        move_group_node,
        control_node,
        robot_state_publisher,
        joint_state_broadcaster_node,
        rebel_6dof_controller_node,
        additional_controllers,
        rviz_node,
    ]


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
        default_value="ext_dio_gripper",
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
    rebel_version_arg = DeclareLaunchArgument(
        "rebel_version",
        default_value="01",
        choices=["pre", "00", "01"],
        description="Which version of the igus ReBeL to use",
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
    ld.add_action(rebel_version_arg)

    ld.add_action(OpaqueFunction(function=opaque_test))

    return ld
