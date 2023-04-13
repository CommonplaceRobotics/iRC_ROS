from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent, DeclareLaunchArgument
from launch.events import matches_action
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState

import lifecycle_msgs.msg


def generate_launch_description():
    # Parameters to make including this multiple times over IncludeLaunchDescription easy
    laserscanner_name = LaunchConfiguration("laserscanner_name")
    laserscanner_name_arg = DeclareLaunchArgument(
        "laserscanner_name",
        default_value="sick_s300",
        description="Name of the laserscanner node",
    )

    namespace = LaunchConfiguration("namespace")
    namespace_arg = DeclareLaunchArgument(
        "namespace", default_value="", description="Namespace for the laserscanner node"
    )

    default_param_file = PathJoinSubstitution(
        [
            FindPackageShare("sicks300_2"),
            "params",
            "default.yaml",
        ]
    )
    params_file = LaunchConfiguration("params_file")
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_param_file,
        description="Path to the parameter file containing the laser scanner's config",
    )

    sicks300_2_node = LifecycleNode(
        package="sicks300_2",
        namespace=namespace,
        executable="sicks300_2",
        name=laserscanner_name,
        parameters=[params_file],
        emulate_tty=True,
    )

    # When the sick node reaches the 'inactive' state, make it take the 'activate' transition.
    register_event_handler_for_sick_reaches_inactive_state = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=sicks300_2_node,
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(sicks300_2_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    # Make the sicks300_2 node take the 'configure' transition.
    emit_event_to_request_that_sick_does_configure_transition = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(sicks300_2_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    description = LaunchDescription()

    description.add_action(laserscanner_name_arg)
    description.add_action(namespace_arg)
    description.add_action(params_file_arg)

    description.add_action(sicks300_2_node)
    description.add_action(register_event_handler_for_sick_reaches_inactive_state)
    description.add_action(emit_event_to_request_that_sick_does_configure_transition)

    return description
