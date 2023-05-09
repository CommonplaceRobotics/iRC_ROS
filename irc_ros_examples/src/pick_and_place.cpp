#include <algorithm>
#include <chrono>
#include <thread>
#include <vector>

#include "Eigen/Geometry"
#include "geometry_msgs/msg/pose.hpp"
#include "irc_ros_examples/pick_and_place_base.hpp"
#include "irc_ros_msgs/msg/dio_command.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "std_msgs/msg/bool.hpp"

class PickAndPlace : public PickAndPlaceBase
{
public:
  explicit PickAndPlace() : PickAndPlaceBase("pick_and_place")
  {
    gripper_publisher = move_group_node->create_publisher<irc_ros_msgs::msg::DioCommand>(
      "external_dio_controller/outputs", rclcpp::SystemDefaultsQoS());

    // Start the process loop in a new thread
    process_thread = std::thread([this]() {
      // Wait a bit before sending commands so the initialisation can finish
      // TODO: Replace this with waiting for MoveIt/ros2_control to finish their inits
      std::this_thread::sleep_for(std::chrono::seconds(2));

      // Run the process as long as possible
      while (rclcpp::ok()) {
        pick_and_place();
      }
    });

    process_thread.detach();
  }

  /**
   * @brief Calculates a quaternion pointing away from the center of the robot for a given position
   * @param position The position relative to (0, 0, 0). May not lie on the z axis.
   * @return A Quaternion msg that can be used directly for the pose's orientation
  */
  geometry_msgs::msg::Quaternion calculate_rotation(geometry_msgs::msg::Point position)
  {
    // Corrosponds to the unit quaternion, meaning no rotation
    Eigen::Vector3d unit(1.0, 0.0, 0.0);

    // The alignment should always be parallel to the ground plane and independent from the TCPs height
    Eigen::Vector3d tcp(position.x, position.y, /*position.z*/ 0.0);

    // Calculates a quaternion which rotates the first vector to the second one
    // Both vectors are defined as (0 0 0) -> (x y z)
    Eigen::Quaterniond q = Eigen::Quaterniond().setFromTwoVectors(unit, tcp);

    // Eigen quaternions are not normalized by default, but moveIt requires that
    q.normalize();

    RCLCPP_INFO(
      LOGGER, "Rot for %lf %lf 0.0: %lf %lf %lf %lf", position.x, position.y, q.w(), q.x(), q.y(),
      q.z());

    geometry_msgs::msg::Quaternion quat;
    quat.w = q.w();
    quat.x = q.x();
    quat.y = q.y();
    quat.z = q.z();

    return quat;
  }

  void pick_and_place()
  {
    geometry_msgs::msg::PoseStamped posestamped;

    const int num_of_blocks = 2;

    const double start_height = 0.37;
    const double height_offset = 0.0;  //0.1;
    const double block_height = 0.030;

    const double rotation_angle = 0.30;

    auto joint_names = move_group->getJointNames();
    auto joint1_iter = std::find(joint_names.begin(), joint_names.end(), "joint1");

    if (joint1_iter == joint_names.end()) {
      RCLCPP_ERROR(LOGGER, "Could not find joint1");
      return;
    }

    auto joint1_index = joint1_iter - joint_names.begin();

    posestamped.header.frame_id = "base_link";
    posestamped.pose.position.x = 0.45;
    posestamped.pose.position.y = 0.0;
    posestamped.pose.position.z = start_height;
    posestamped.pose.orientation = calculate_rotation(posestamped.pose.position);

    irc_ros_msgs::msg::DioCommand msg;
    msg.header = std_msgs::msg::Header();
    msg.header.stamp = move_group_node->get_clock()->now();
    msg.names.push_back("dio_ext/digital_output_1");
    msg.outputs.push_back(false);
    gripper_publisher->publish(msg);

    // Move up and down once
    posestamped.pose.position.x -= 0.10;
    print_pose(posestamped.pose);
    move_group->setPoseTarget(posestamped, "tcp");
    move_group->move();

    posestamped.pose.position.z = start_height - height_offset - num_of_blocks * block_height;
    lin(posestamped);

    posestamped.pose.position.z = start_height;
    lin(posestamped);

    // Start pos
    posestamped.pose.position.x += 0.10;
    print_pose(posestamped.pose);
    move_group->setPoseTarget(posestamped, "tcp");
    move_group->move();

    for (int i = 0; i < num_of_blocks; i++) {
      RCLCPP_INFO(LOGGER, "Picking block %d", i + 1);
      // Lower
      posestamped.pose.position.z = start_height - height_offset - (i + 1) * block_height;
      lin(posestamped);

      // Pick
      msg = irc_ros_msgs::msg::DioCommand();
      msg.header = std_msgs::msg::Header();
      msg.header.stamp = move_group_node->get_clock()->now();
      msg.names.push_back("dio_ext/digital_output_1");
      msg.outputs.push_back(true);
      gripper_publisher->publish(msg);

      std::this_thread::sleep_for(std::chrono::milliseconds(2000));

      // Higher
      posestamped.pose.position.z = start_height;
      posestamped.pose.orientation = calculate_rotation(posestamped.pose.position);
      lin(posestamped);

      // Rotate to place position
      RCLCPP_INFO(LOGGER, "Placing block %d", i + 1);
      auto joint_vals = move_group->getCurrentJointValues();
      joint_vals[joint1_index] -= rotation_angle;
      move_group->setJointValueTarget(joint_vals);
      move_group->move();

      // Update pose var
      posestamped.pose = move_group->getCurrentPose().pose;

      // Lower
      posestamped.pose.position.z =
        start_height - height_offset - (num_of_blocks - i) * block_height + 0.005;

      posestamped.pose.orientation = calculate_rotation(posestamped.pose.position);
      lin(posestamped);

      // Place
      msg = irc_ros_msgs::msg::DioCommand();
      msg.header = std_msgs::msg::Header();
      msg.header.stamp = move_group_node->get_clock()->now();
      msg.names.push_back("dio_ext/digital_output_1");
      msg.outputs.push_back(false);
      gripper_publisher->publish(msg);

      std::this_thread::sleep_for(std::chrono::milliseconds(2000));

      // Higher
      posestamped.pose.position.z = start_height;
      posestamped.pose.orientation = calculate_rotation(posestamped.pose.position);
      lin(posestamped);

      // Rotate to pick rotation (unless it is the last object)
      if (i != num_of_blocks - 1) {
        joint_vals = move_group->getCurrentJointValues();
        joint_vals[joint1_index] += rotation_angle;
        move_group->setJointValueTarget(joint_vals);
        move_group->move();

        // Update pose var
        posestamped.pose = move_group->getCurrentPose().pose;
        posestamped.pose.orientation = calculate_rotation(posestamped.pose.position);
      }
    }

    // Second start pos
    posestamped.pose.position.z = start_height;
    posestamped.pose.orientation = calculate_rotation(posestamped.pose.position);

    print_pose(posestamped.pose);
    lin(posestamped);
    // move_group->setPoseTarget(posestamped, "tcp");
    // move_group->move();

    for (int i = 0; i < num_of_blocks; i++) {
      RCLCPP_INFO(LOGGER, "Picking block %d", i + 1);
      // Lower
      posestamped.pose.position.z = start_height - height_offset - (i + 1) * block_height;
      posestamped.pose.orientation = calculate_rotation(posestamped.pose.position);
      lin(posestamped);

      // Pick
      msg = irc_ros_msgs::msg::DioCommand();
      msg.header = std_msgs::msg::Header();
      msg.header.stamp = move_group_node->get_clock()->now();
      msg.names.push_back("dio_ext/digital_output_1");
      msg.outputs.push_back(true);
      gripper_publisher->publish(msg);

      std::this_thread::sleep_for(std::chrono::milliseconds(2000));

      // Higher
      posestamped.pose.position.z = start_height;
      lin(posestamped);

      // Rotate to place position
      RCLCPP_INFO(LOGGER, "Placing block %d", i + 1);
      auto joint_vals = move_group->getCurrentJointValues();
      joint_vals[joint1_index] += rotation_angle;
      move_group->setJointValueTarget(joint_vals);
      move_group->move();

      // Update pose var
      posestamped.pose = move_group->getCurrentPose().pose;

      // Lower
      posestamped.pose.position.z =
        start_height - height_offset - (num_of_blocks - i) * block_height + 0.005;

      posestamped.pose.orientation = calculate_rotation(posestamped.pose.position);
      lin(posestamped);

      // Place
      msg = irc_ros_msgs::msg::DioCommand();
      msg.header = std_msgs::msg::Header();
      msg.header.stamp = move_group_node->get_clock()->now();
      msg.names.push_back("dio_ext/digital_output_1");
      msg.outputs.push_back(false);
      gripper_publisher->publish(msg);

      std::this_thread::sleep_for(std::chrono::milliseconds(2000));

      // Higher
      posestamped.pose.position.z = start_height;
      posestamped.pose.orientation = calculate_rotation(posestamped.pose.position);
      lin(posestamped);

      // Rotate to pick rotation (unless it is the last object)
      if (i != num_of_blocks - 1) {
        joint_vals = move_group->getCurrentJointValues();
        joint_vals[joint1_index] -= rotation_angle;
        move_group->setJointValueTarget(joint_vals);
        move_group->move();

        // Update pose var
        posestamped.pose = move_group->getCurrentPose().pose;
        posestamped.pose.orientation = calculate_rotation(posestamped.pose.position);
      }
    }
  }

private:
  std::shared_ptr<rclcpp::Publisher<irc_ros_msgs::msg::DioCommand>> gripper_publisher;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<PickAndPlace>());
  rclcpp::shutdown();
  return 0;
}
