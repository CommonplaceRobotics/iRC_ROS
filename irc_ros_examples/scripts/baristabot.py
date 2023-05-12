#!/usr/bin/env python3

import copy
import time
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from moveit.planning import MoveItPy  # Requires MoveIt2 for rolling
from geometry_msgs.msg import PoseStamped
from irc_ros_msgs.msg import DioCommand


class NavigationInterface:
    def __init__(self) -> None:
        self.nav = BasicNavigator()

        # TODO: Add this back once slam_toolbox is a lifecycle node
        # if autostarted, else use `lifecycleStartup()`
        # nav.waitUntilNav2Active(localizer="slam_toolbox")

        self.frame_id = "map"

        # Define poses
        self.pose_coffee_maker = PoseStamped()
        self.pose_coffee_maker.header.frame_id = self.frame_id
        self.pose_coffee_maker.pose.position.x = 0.0
        self.pose_coffee_maker.pose.position.y = 2.0
        self.pose_coffee_maker.pose.position.z = 0.0

        self.pose_serving_area = PoseStamped()
        self.pose_serving_area.header.frame_id = self.frame_id
        self.pose_serving_area.pose.position.x = 2.0
        self.pose_serving_area.pose.position.y = 0.0
        self.pose_serving_area.pose.position.z = 0.0

    def move_to_pose(self, pose: PoseStamped) -> None:
        self.nav.goToPose(pose)

        while not self.nav.isTaskComplete():
            # feedback = self.nav.getFeedback()
            # if feedback.navigation_time > 600:
            #     nav.cancelTask()

            result = self.nav.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info("Goal succeeded")
            elif result == TaskResult.CANCELED:
                self.get_logger().info("Goal canceled")
            elif result == TaskResult.FAILED:
                self.get_logger().info("Goal failed")


class MoveItInterface:
    def __init__(self) -> None:
        time.sleep(1)
        self.moveit_py_instance = MoveItPy(node_name="moveit_py")
        self.rebel = self.moveit_py_instance.get_planning_component("rebel_6dof")

        # The name of the base frame
        # TODO: Add rebel prefix to it
        self.frame_id = "base_link"

        self.pose_cup_pickup = PoseStamped()
        self.pose_cup_pickup.header.frame_id = self.frame_id
        self.pose_cup_pickup.pose.position.x = 0.45
        self.pose_cup_pickup.pose.position.y = 0.0
        self.pose_cup_pickup.pose.position.z = 0.30
        self.pose_cup_pickup.pose.orientation.w = 1.0

        self.pose_cup_pickup_offset = PoseStamped()
        self.pose_cup_pickup_offset.header.frame_id = self.frame_id
        self.pose_cup_pickup_offset.pose.position.x = 0.40
        self.pose_cup_pickup_offset.pose.position.y = 0.0
        self.pose_cup_pickup_offset.pose.position.z = 0.40
        self.pose_cup_pickup_offset.pose.orientation.w = 1.0

        self.pose_transport = PoseStamped()
        self.pose_transport.header.frame_id = self.frame_id
        self.pose_transport.pose.position.x = 0.35
        self.pose_transport.pose.position.y = 0.0
        self.pose_transport.pose.position.z = 0.35
        self.pose_transport.pose.orientation.w = 1.0

        self.pose_coffee_maker = PoseStamped()
        self.pose_coffee_maker.header.frame_id = self.frame_id
        self.pose_coffee_maker.pose.position.x = 0.61
        self.pose_coffee_maker.pose.position.y = 0.0
        self.pose_coffee_maker.pose.position.z = 0.36
        self.pose_coffee_maker.pose.orientation.w = 1.0

        # Position slightly in front and above of the cup placement position
        self.pose_coffee_maker_offset = copy.deepcopy(self.pose_coffee_maker)
        self.pose_coffee_maker_offset.pose.position.x -= 0.05
        self.pose_coffee_maker_offset.pose.position.z += 0.03

        # Position at which the button is pressed by the gripper
        self.pose_coffee_maker_button = copy.deepcopy(self.pose_coffee_maker)
        self.pose_coffee_maker_button.pose.position.x -= 0.05
        self.pose_coffee_maker_button.pose.position.z = 0.52

        # Position slightly in front of the button
        self.pose_coffee_maker_button_offset = copy.deepcopy(
            self.pose_coffee_maker_button
        )
        self.pose_coffee_maker_button_offset.pose.position.x -= 0.05

    def move_to_pose(self, pose: PoseStamped) -> None:
        self.rebel.set_start_state_to_current_state()
        self.rebel.set_goal_state(pose_stamped_msg=pose, pose_link="tcp")
        plan_result = self.rebel.plan()

        if plan_result:
            robot_trajectory = plan_result.trajectory
            self.moveit_py_instance.execute(
                robot_trajectory, blocking=True, controllers=[]
            )


class GripperInterface(Node):
    def __init__(self) -> None:
        super().__init__("Node")
        self.publisher_ = self.create_publisher(
            DioCommand, "/external_dio_controller/set_outputs", 10
        )

    def set_gripper(self, state: bool) -> None:
        msg = DioCommand()
        msg.names = [
            "dio_ext/digital_output_1",
        ]
        msg.outputs = [
            state,
        ]

        self.publisher_.publish(msg)


if __name__ == "__main__":
    time.sleep(5)
    rclpy.init()

    # nav_int = NavigationInterface()
    moveit_int = MoveItInterface()
    gripper_int = GripperInterface()

    while rclpy.ok():
        # TODO: Battery low?

        # TODO: If coffee machine (cm) not ready notify assistant to refill, empty machine

        # TODO: Coffee ordered? (Via web ui?)

        # Grab empty cup
        moveit_int.move_to_pose(moveit_int.pose_cup_pickup_offset)
        moveit_int.move_to_pose(moveit_int.pose_cup_pickup)
        gripper_int.set_gripper(True)
        time.sleep(2)
        moveit_int.move_to_pose(moveit_int.pose_cup_pickup_offset)

        # Set arm to transport position
        # moveit_int.move_to_pose(moveit_int.pose_transport)
        # time.sleep(1)

        # TODO: No empty cups left? Notify assistant

        # Move to CM
        # nav_int.move_to_pose(nav_int.pose_coffee_maker)

        # Bring empty cup in position
        moveit_int.move_to_pose(moveit_int.pose_coffee_maker_offset)
        moveit_int.move_to_pose(moveit_int.pose_coffee_maker)
        gripper_int.set_gripper(False)
        time.sleep(2)
        moveit_int.move_to_pose(moveit_int.pose_coffee_maker_offset)

        # Start selected CM program
        moveit_int.move_to_pose(moveit_int.pose_coffee_maker_button_offset)
        gripper_int.set_gripper(True)
        time.sleep(2)
        moveit_int.move_to_pose(moveit_int.pose_coffee_maker_button)
        moveit_int.move_to_pose(moveit_int.pose_coffee_maker_button_offset)
        gripper_int.set_gripper(False)

        # Wait for CM
        time.sleep(35)

        # Grab cup again
        moveit_int.move_to_pose(moveit_int.pose_coffee_maker_offset)
        moveit_int.move_to_pose(moveit_int.pose_coffee_maker)
        gripper_int.set_gripper(True)
        time.sleep(2)
        moveit_int.move_to_pose(moveit_int.pose_coffee_maker_offset)

        # Drive slowly to counter
        # moveit_int.move_to_pose(moveit_int.pose_transport)
        # nav_int.move_to_pose(nav_int.pose_serving_area)

        # Place cup there
        # moveit_int.move_to_pose(moveit_int.pose_serving_tray)
        # moveit_int.move_to_pose(moveit_int.pose_cup_pickup_offset)
        # moveit_int.move_to_pose(moveit_int.pose_cup_pickup)
        # gripper_int.set_gripper(False)
        # time.sleep(2)
        # moveit_int.move_to_pose(moveit_int.pose_cup_pickup_offset)

        # Drive back to waiting position
        # nav_int.move_to_pose(nav_int.pose_waiting_area)

        moveit_int.move_to_pose(moveit_int.pose_transport)

        # TODO: Wait until new order
        while True:
            time.sleep(1)
