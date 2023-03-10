import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# moveit python library
from moveit.planning import MoveItPy

from geometry_msgs.msg import PoseStamped


class NavigationInterface:
    def __init__(self) -> None:
        self.nav = BasicNavigator()

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

    def move_to_pose(self, pose: PoseStamped):
        self.nav.goToPose(pose)

        while not self.nav.isTaskComplete():
            #feedback = self.nav.getFeedback()
            # if feedback.navigation_time > 600:
            #     nav.cancelTask()

            result = self.nav.getResult()
            if result == TaskResult.SUCCEEDED:
                print("Goal succeeded!")
            elif result == TaskResult.CANCELED:
                print("Goal was canceled!")
            elif result == TaskResult.FAILED:
                print("Goal failed!")

class MoveItInterface:
    def __init__(self) -> None:
        self.moveit_py_instance = MoveItPy(node_name="moveit_py")
        self.rebel = self.moveit_py_instance.get_planning_component("rebel_6dof")

    def move_to_pose(self, pose: PoseStamped):
        self.rebel.set_start_state_to_current_state()
        self.rebel.set_goal_state(pose_stamped_msg=pose, pose_link="tcp")
        plan_result = self.rebel.plan()

        if plan_result:
            robot_trajectory = plan_result.trajectory
            self.rebel.execute(robot_trajectory, blocking=True, controllers=[])


if __name__ == '__main__':
    rclpy.init()
    nav_int = NavigationInterface()
    moveit_int = MoveItInterface()

    while rclpy.ok():
        # TODO: Battery low?
    
        # TODO: If coffee machine (cm) not ready notify assistant to refill, empty machine

        # TODO: Coffee ordered? (Via web ui?)

        # TODO: Grab empty cup
        # gripper_service.call(close)

        # TODO: No empty cups left? Notify assistant
        
        # Move to CM
        nav_int.move_to_pose(nav_int.pose_coffee_maker)

        # TODO: Bring empty cup in position
        #moveit_int.move_to_pose(moveit_int.pose_coffe_maker)
        # gripper_service.call(open)
    
        # TODO: Start selected CM program
        
        # Wait for CM
    
        # Grab cup again
        # gripper_service.call(close)
    
        # Drive slowly to counter
        nav_int.move_to_pose(nav_int.pose_serving_area)

        # TODO: Place cup there
        #moveit_int.move_to_pose(moveit_int.pose_serving_tray)
        # gripper_service.call(open)

        # Drive back to waiting position
        #nav_int.move_to_pose(nav_int.pose_waiting_area)



