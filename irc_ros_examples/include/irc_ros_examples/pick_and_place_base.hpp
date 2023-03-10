// Loosely based on the moveit tutorial by PickNik
// https://moveit.picknik.ai/humble/doc/examples/motion_planning_api/motion_planning_api_tutorial.html

#pragma once

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "Eigen/Geometry"
#include "geometry_msgs/msg/pose.hpp"
#include "irc_ros_msgs/srv/gripper_command.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("PickAndPlace");

class PickAndPlaceBase : public rclcpp::Node
{
public:
  explicit PickAndPlaceBase(std::string node_name) : rclcpp::Node(node_name)
  {
    // MoveIt init
    move_group_node = rclcpp::Node::make_shared("move_group_interface");

    executor.add_node(move_group_node);
    move_group_thread = std::thread([this]() { executor.spin(); });
    move_group_thread.detach();

    move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      move_group_node, PLANNING_GROUP);

    // Set default values
    move_group->setPoseReferenceFrame("base_link");
    move_group->setPlanningTime(10);
    move_group->setNumPlanningAttempts(3);
    move_group->setMaxVelocityScalingFactor(0.1);

    RCLCPP_INFO(LOGGER, "Starting pose:");
    print_pose(move_group->getCurrentPose().pose);
  }

  // TODO: Does not exit cleanly with CTRL+C, requires a SIGKILL
  ~PickAndPlaceBase()
  {
    if (process_thread.joinable()) {
      process_thread.join();
    }

    executor.cancel();

    if (move_group_thread.joinable()) {
      move_group_thread.join();
    }
  }

  /**
   * @brief Prints the given pose's position and rotation to the log
   * @param pose Unstamped pose to print
   */
  void print_pose(geometry_msgs::msg::Pose pose)
  {
    RCLCPP_INFO(
      LOGGER, "Pos: %lf %lf %lf Rot: %lf %lf %lf %lf", pose.position.x, pose.position.y,
      pose.position.z, pose.orientation.w, pose.orientation.x, pose.orientation.y,
      pose.orientation.z);
  }

  /**
   * @brief Calculates and executes a linear movement to a given pose if possible,. Otherwise will use a point-to-point motion.
   * @param move_group Pointer to the MoveGroupInterface that shall be used for the movement
   * @param posestamped The pose that should be moved to from the move_groups current position
   */
  void lin(geometry_msgs::msg::PoseStamped posestamped)
  {
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.00;
    const double eef_step = 0.005;
    double fraction;

    std::vector<geometry_msgs::msg::Pose> waypoints = {
      move_group->getCurrentPose().pose, posestamped.pose};

    print_pose(posestamped.pose);
    fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    if (fraction < 1.0) {
      RCLCPP_WARN(
        LOGGER, "The accuracy of the linear movement is only %lf, using p2p motion instead!",
        fraction);
      move_group->setPoseTarget(posestamped, "hand");
      move_group->move();
    } else {
      RCLCPP_INFO(LOGGER, "Accuracy %lf", fraction);
      move_group->execute(trajectory);
    }
  }

protected:
  // MoveIt specifics
  const std::string PLANNING_GROUP = "rebel_6dof";
  std::shared_ptr<rclcpp::Node> move_group_node;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::thread process_thread;
  std::thread move_group_thread;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
};