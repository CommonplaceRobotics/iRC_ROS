#!/usr/bin/env python3

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from geometry_msgs.msg import PoseStamped

rclpy.init()

nav = BasicNavigator()

frame_id = "map"

# Four poses forming a square

init_pose = PoseStamped()
init_pose.header.frame_id = frame_id
init_pose.pose.position.x = 0.0
init_pose.pose.position.y = 2.0
init_pose.pose.position.z = 0.0

goal_pose1 = PoseStamped()
goal_pose1.header.frame_id = frame_id
goal_pose1.pose.position.x = 0.0
goal_pose1.pose.position.y = 3.0
goal_pose1.pose.position.z = 0.0

goal_pose2 = PoseStamped()
goal_pose2.header.frame_id = frame_id
goal_pose2.pose.position.x = 1.0
goal_pose2.pose.position.y = 3.0
goal_pose2.pose.position.z = 0.0

goal_pose3 = PoseStamped()
goal_pose3.header.frame_id = frame_id
goal_pose3.pose.position.x = 1.0
goal_pose3.pose.position.y = 2.0
goal_pose3.pose.position.z = 0.0

poses = [init_pose, goal_pose1, goal_pose2, goal_pose3, init_pose]

nav.setInitialPose(init_pose)

# if autostarted, else use `lifecycleStartup()`
# nav.waitUntilNav2Active(localizer="slam_toolbox")

nav.followWaypoints(poses)


while not nav.isTaskComplete():
    feedback = nav.getFeedback()

    # if feedback.navigation_time > 600:
    #     nav.cancelTask()

result = nav.getResult()
if result == TaskResult.SUCCEEDED:
    print("Goal succeeded!")
elif result == TaskResult.CANCELED:
    print("Goal was canceled!")
elif result == TaskResult.FAILED:
    print("Goal failed!")
