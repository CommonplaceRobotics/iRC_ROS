#include <algorithm>
#include <chrono>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "irc_ros_examples/pick_and_place_base.hpp"
#include "irc_ros_msgs/srv/gripper_command.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class PickAndPlaceVacuum : public PickAndPlaceBase
{
public:
  explicit PickAndPlaceVacuum() : PickAndPlaceBase("pick_and_place_vacuum")
  {
    RCLCPP_DEBUG(LOGGER, "Starting the gripper client");

    // ECBPMI Gripper init
    gripper_client =
      create_client<irc_ros_msgs::srv::GripperCommand>("ecbpmi_controller/set_gripper");

    while (!gripper_client->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(LOGGER, "Client interrupted while waiting for service to appear.");
        return;
      }
      RCLCPP_INFO(LOGGER, "Waiting for ECBPMI gripper service to appear...");
    }

    buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);

    RCLCPP_INFO(LOGGER, "Starting the pick and place process");

    // Start the process loop in a new thread
    process_thread = std::thread([this]() {
      // Wait a bit before sending commands so the initialisation can finish
      // TODO: Replace this with waiting for MoveIt/ros2_control to finish their inits
      std::this_thread::sleep_for(std::chrono::seconds(2));

      // Run the process as long as possible
      while (rclcpp::ok()) {
        pick_and_place_vacuum();
      }
    });
  }

  /**
   * @brief Calls the gripper service 
   * @param state true for grip
   * @return true on success
  */
  bool set_gripper(bool state)
  {
    request->grip = state;

    auto result = gripper_client->async_send_request(request);

    // Wait for the result
    std::chrono::time_point<std::chrono::steady_clock> start = std::chrono::steady_clock::now();
    const std::chrono::duration<int64_t, std::milli> timeout = std::chrono::milliseconds(1000);

    while (std::chrono::steady_clock::now() - start < timeout) {
      if (result.future.valid()) {
        return (result.get()->gripped == state);
      }
      std::this_thread::yield();
    }

    RCLCPP_ERROR(LOGGER, "Failed to call gripper service");
    return false;
  }

  /**
   * @brief Moves along the outlines of a tray
   * @param frame_id The base frame of the tray
   * @param objects_x Number of objects in x direction 
   * @param objects_y Number of objects in y direction 
   */
  void outline_tray(std::string frame_id, int objects_x, int objects_y)
  {
    geometry_msgs::msg::PoseStamped posestamped;
    posestamped.header.frame_id = frame_id;
    posestamped.pose.position.x = 0 * slot_offset_;
    posestamped.pose.position.y = 0 * slot_offset_;
    posestamped.pose.position.z = height_offset_;

    // Always point downwards for all movements
    posestamped.pose.orientation.w = 0.707;
    posestamped.pose.orientation.x = 0;
    posestamped.pose.orientation.y = 0.707;
    posestamped.pose.orientation.z = 0;

    geometry_msgs::msg::PoseStamped target_pose;

    target_pose = buffer_->transform(posestamped, planning_frame_);
    lin(target_pose);

    posestamped.pose.position.x = (objects_x - 1) * slot_offset_;
    target_pose = buffer_->transform(posestamped, planning_frame_);
    lin(target_pose);

    posestamped.pose.position.y = (objects_y - 1) * slot_offset_;
    target_pose = buffer_->transform(posestamped, planning_frame_);
    lin(target_pose);

    posestamped.pose.position.x = 0;
    target_pose = buffer_->transform(posestamped, planning_frame_);
    lin(target_pose);

    posestamped.pose.position.y = 0;
    target_pose = buffer_->transform(posestamped, planning_frame_);
    lin(target_pose);
  }

  /**
   * @brief Picks up an object from the given position
   * @param frame_id The base frame of the tray
   * @param x The slot number in x direction 
   * @param y The slot number in y direction 
   */
  void pick(std::string frame_id, int x, int y)
  {
    geometry_msgs::msg::PoseStamped posestamped;
    posestamped.header.frame_id = frame_id;
    posestamped.pose.position.x = x * slot_offset_;
    posestamped.pose.position.y = y * slot_offset_;
    posestamped.pose.position.z = height_offset_;

    // Always point downwards for all movements
    posestamped.pose.orientation.w = 0.707;
    posestamped.pose.orientation.x = 0;
    posestamped.pose.orientation.y = 0.707;
    posestamped.pose.orientation.z = 0;

    geometry_msgs::msg::PoseStamped target_pose;

    // Move over the object
    target_pose = buffer_->transform(posestamped, planning_frame_);
    lin(target_pose);

    // Slowly touch the object
    posestamped.pose.position.z -= height_offset_;
    target_pose = buffer_->transform(posestamped, planning_frame_);
    lin(target_pose, 0.1, 0.1);

    set_gripper(true);

    // Lift the object
    posestamped.pose.position.z += height_offset_;
    target_pose = buffer_->transform(posestamped, planning_frame_);
    lin(target_pose);
  }

  /**
   * @brief Places an object in the given position
   * @param frame_id The base frame of the tray
   * @param x The slot number in x direction 
   * @param y The slot number in y direction 
   */
  void place(std::string frame_id, int x, int y)
  {
    geometry_msgs::msg::PoseStamped posestamped;
    posestamped.header.frame_id = frame_id;
    posestamped.pose.position.x = x * slot_offset_;
    posestamped.pose.position.y = y * slot_offset_;
    posestamped.pose.position.z = height_offset_;

    // Always point downwards for all movements
    posestamped.pose.orientation.w = 0.707;
    posestamped.pose.orientation.x = 0;
    posestamped.pose.orientation.y = 0.707;
    posestamped.pose.orientation.z = 0;

    geometry_msgs::msg::PoseStamped target_pose;

    // Move over the slot
    target_pose = buffer_->transform(posestamped, planning_frame_);
    lin(target_pose);

    // Slowly touch the object
    posestamped.pose.position.z -= height_offset_;
    target_pose = buffer_->transform(posestamped, planning_frame_);
    lin(target_pose, 0.1, 0.1);

    set_gripper(false);
    // While the set_gripper fn blocks until the vacuum is gone or a timeout occurs
    // it takes a little longer than the vacuum gone signal for the object to disconnect
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));

    //Move back up again
    posestamped.pose.position.z += height_offset_;
    target_pose = buffer_->transform(posestamped, planning_frame_);
    lin(target_pose);
  }

  void pick_and_place_vacuum()
  {
    // Number of objects to grip, assumes that both trays are identical
    const int objects_y = 3;
    const int objects_x = 5;

    // Make sure the gripper is empty
    set_gripper(false);

    // Move over the outlines of the trays to make sure they are at the right position
    outline_tray("tray_1", objects_x, objects_y);
    outline_tray("tray_2", objects_x, objects_y);

    // TODO: Reset speed limit

    // Transfer components from first to second tray
    for (int x = 0; x < objects_x; x++) {
      for (int y = 0; y < objects_y; y++) {
        pick("tray_1", x, y);

        place("tray_2", x, y);

        //Wait a bit to make sure the component is really not connected anymore
        std::this_thread::sleep_for(std::chrono::milliseconds(400));
      }
    }

    // Transfer components back to the first tray
    for (int x = 0; x < objects_x; x++) {
      for (int y = 0; y < objects_y; y++) {
        pick("tray_2", x, y);

        place("tray_1", x, y);

        //Wait a bit to make sure the component is really not connected anymore
        std::this_thread::sleep_for(std::chrono::milliseconds(400));
      }
    }
  }

private:
  // ECBPMI gripper service client
  std::shared_ptr<rclcpp::Client<irc_ros_msgs::srv::GripperCommand>> gripper_client;
  std::shared_ptr<irc_ros_msgs::srv::GripperCommand::Request> request =
    std::make_shared<irc_ros_msgs::srv::GripperCommand::Request>();

  // TF2 transform objects
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;

  // Scene parameters
  const double height_offset_ =
    0.03;                             // z offset of how much to lift the objects out of the trays
  const double slot_offset_ = 0.035;  // x and y offset between different slots in the tray
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<PickAndPlaceVacuum>());
  rclcpp::shutdown();
  return 0;
}
