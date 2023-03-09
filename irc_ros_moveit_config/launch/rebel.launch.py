# Based on the panda moveit launch file from ros-planning
# https://github.com/ros-planning/moveit_resources/blob/humble/panda_moveit_config/launch/demo.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        choices=["0", "1", "false", "true", "False", "True"],
    )
    gripper_arg = DeclareLaunchArgument(
        "gripper",
        default_value="none",
        choices=["none", "schmalz_ecbpmi", "ext_dio_gripper"],
    )

    use_rviz = LaunchConfiguration("use_rviz")
    # gripper = LaunchConfiguration("gripper")

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

    # TODO: How to set urdf/xacro parameters?
    moveit_config = (
        MoveItConfigsBuilder("irc_ros")
        .robot_description(file_path="config/igus_rebel_6dof.urdf.xacro")
        .robot_description_semantic(file_path="config/igus_rebel_6dof.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"publish_robot_description_semantic": True},
            planning_scene_monitor_parameters,
        ],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_file,
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        # namespace=namespace,
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    rebel_6dof_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rebel_6dof_controller", "-c", "/controller_manager"],
    )

    dio_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        # namespace=namespace,
        arguments=["dio_controller", "-c", "/controller_manager"],
        condition=LaunchConfigurationEquals("gripper", "none"),
    )

    external_dio_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        # namespace=namespace,
        arguments=["external_dio_controller", "-c", "/controller_manager"],
        condition=LaunchConfigurationEquals("gripper", "ext_dio_gripper"),
    )

    ecbpmi_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        # namespace=namespace,
        arguments=["ecbpmi_controller", "-c", "/controller_manager"],
        condition=LaunchConfigurationEquals("gripper", "schmalz_ecbpmi"),
    )

    joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_node,
            on_exit=[
                rebel_6dof_controller_node,
                dio_controller_node,
                external_dio_controller_node,
                ecbpmi_controller_node,
                # dashboard_controller_node,
            ],
        )
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
        condition=IfCondition(use_rviz),
    )
    ld = LaunchDescription()

    ld.add_action(use_rviz_arg)
    ld.add_action(gripper_arg)

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
