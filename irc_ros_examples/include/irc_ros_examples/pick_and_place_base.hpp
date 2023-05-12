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
#include "moveit/trajectory_processing/time_optimal_trajectory_generation.h"

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
    move_group->setPoseReferenceFrame(planning_frame_);
    move_group->setPlanningTime(10);
    move_group->setNumPlanningAttempts(3);
    move_group->setMaxVelocityScalingFactor(0.1);

    RCLCPP_INFO(LOGGER, "Starting pose:");
    print_pose(move_group->getCurrentPose().pose);
  }

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
  void lin(
    geometry_msgs::msg::PoseStamped posestamped, const double velocity_scale = 0.3,
    const double acceleration_scale = 0.3)
  {
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 1.40;
    const double eef_step = 0.010;
    double fraction;

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    std::vector<geometry_msgs::msg::Pose> waypoints = {
      move_group->getCurrentPose().pose, posestamped.pose};

    print_pose(posestamped.pose);
    fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    if (fraction < 1.0) {
      RCLCPP_WARN(
        LOGGER, "The accuracy of the linear movement is only %lf, using p2p motion instead!",
        fraction);
      p2p(posestamped);
    } else {
      // Successful planning
      RCLCPP_INFO(LOGGER, "Accuracy %lf", fraction);

      // Scale the trajectory
      if (velocity_scale != 1.0 || acceleration_scale != 1.0) {
        trajectory_processing::TimeOptimalTrajectoryGeneration totg;
        robot_trajectory::RobotTrajectory rt(move_group->getRobotModel(), PLANNING_GROUP);
        rt.setRobotTrajectoryMsg(*move_group->getCurrentState(), trajectory);
        bool scaling_successful = totg.computeTimeStamps(rt, velocity_scale, acceleration_scale);
        if (!scaling_successful) {
          RCLCPP_WARN(LOGGER, "Applying velocity or acceleration constraints failed!");
        } else {
          rt.getRobotTrajectoryMsg(trajectory);
        }
      }

      move_group->execute(trajectory);
    }
  }

  void p2p(geometry_msgs::msg::PoseStamped posestamped)
  {
    move_group->setPoseTarget(posestamped, eef_frame);
    move_group->move();
  }

protected:
  // MoveIt specifics
  std::string planning_frame_ = "base_link";
  std::string eef_frame = "tcp";
  const std::string PLANNING_GROUP = "rebel_6dof";
  std::shared_ptr<rclcpp::Node> move_group_node;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::thread process_thread;
  std::thread move_group_thread;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
};
