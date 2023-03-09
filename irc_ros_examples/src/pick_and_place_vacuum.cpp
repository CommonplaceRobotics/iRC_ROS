#include <algorithm>
#include <chrono>
#include <thread>
#include <vector>

#include "Eigen/Geometry"
#include "geometry_msgs/msg/pose.hpp"
#include "irc_ros_examples/pick_and_place_base.hpp"
#include "irc_ros_msgs/srv/gripper_command.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"

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
        RCLCPP_ERROR(LOGGER, "client interrupted while waiting for service to appear.");
        return;
      }
      RCLCPP_INFO(LOGGER, "waiting for service to appear...");
    }

    RCLCPP_INFO(LOGGER, "Starting the pick and place process");

    // Run the process as long as possible
    while (rclcpp::ok()) {
      pick_and_place_vacuum();
    }
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

    // Wait for the result.
    if (
      rclcpp::spin_until_future_complete(
        this->get_node_base_interface(), result, std::chrono::seconds(1)) ==
      rclcpp::FutureReturnCode::SUCCESS) {
      return (result.get()->gripped == state);
    }

    RCLCPP_ERROR(LOGGER, "Failed to call gripper service");
    return false;
  }

  void pick_and_place_vacuum()
  {
    const double height_offset = 0.03;  // z offset of how much to lift the objects out of the trays
    const double box_offset = 0.105;    // y offset between the two trays
    const double slot_offset = 0.035;   // x and y offset between different slots in the tray

    // Number of objects to grip
    const int objects_y = 3;
    const int objects_x = 5;

    // Hardcoded start position over the first element to grip
    geometry_msgs::msg::PoseStamped posestamped;
    posestamped.header.frame_id = "base_link";
    posestamped.pose.position.x = 0.209;
    posestamped.pose.position.y = -0.090;
    posestamped.pose.position.z = 0.3180 + height_offset;

    // Always point downwards for all movements
    posestamped.pose.orientation.w = 0;
    posestamped.pose.orientation.x = 0;
    posestamped.pose.orientation.y = 1;
    posestamped.pose.orientation.z = 0;

    // Save startpose to return to lateron without the need to backtrace all movements
    auto start_pose = posestamped;

    // Move to start pos
    move_group->setPoseTarget(posestamped, "hand");
    move_group->move();

    //Make sure the gripper is empty
    set_gripper(false);

    // Move to second edge of first tray
    posestamped.pose.position.x += slot_offset * (objects_x - 1);
    posestamped.pose.position.y += slot_offset * (objects_y - 1);
    lin(posestamped);

    // Move to two edges of second tray
    posestamped.pose.position.y += box_offset;
    lin(posestamped);

    posestamped.pose.position.x -= slot_offset * (objects_x - 1);
    posestamped.pose.position.y -= slot_offset * (objects_y - 1);
    lin(posestamped);

    // Move back to the start position
    posestamped.pose.position.y -= box_offset;
    lin(posestamped);

    // Transfer components from first to second tray
    for (int i = 0; i < objects_x; i++) {
      for (int j = 0; j < objects_y; j++) {
        // Move set position from previous loop iteration
        lin(posestamped);

        posestamped.pose.position.z -= height_offset;
        lin(posestamped);

        set_gripper(true);

        posestamped.pose.position.z += height_offset;
        lin(posestamped);

        // Move to the second tray
        posestamped.pose.position.y += box_offset;
        lin(posestamped);

        posestamped.pose.position.z -= height_offset;
        posestamped.pose.position.z += 0.010;

        lin(posestamped);

        set_gripper(false);

        // TODO: Setting the max velocity does no seem to work
        move_group->setMaxVelocityScalingFactor(0.05);

        //Wait a bit to make sure the component is really not connected anymore
        std::this_thread::sleep_for(std::chrono::milliseconds(400));

        move_group->setMaxVelocityScalingFactor(0.2);

        posestamped.pose.position.z += height_offset;
        posestamped.pose.position.z -= 0.010;
        lin(posestamped);

        // Move to the next spot on the first tray
        posestamped.pose.position.y -= box_offset;
        posestamped.pose.position.y += slot_offset;
      }

      // Go to the first object of the next row
      posestamped.pose.position.y -= slot_offset * (objects_y);
      posestamped.pose.position.x += slot_offset;
    }

    // Move to the first object of the second tray
    posestamped = start_pose;
    posestamped.pose.position.y += box_offset;
    lin(posestamped);

    // Transfer components back to the first tray
    for (int i = 0; i < objects_x; i++) {
      for (int j = 0; j < objects_y; j++) {
        // Move set position from previous loop iteration
        lin(posestamped);

        posestamped.pose.position.z -= height_offset;
        lin(posestamped);

        set_gripper(true);

        posestamped.pose.position.z += height_offset;
        lin(posestamped);

        // Move to the first tray
        posestamped.pose.position.y -= box_offset;
        lin(posestamped);

        posestamped.pose.position.z -= height_offset;
        lin(posestamped);

        set_gripper(false);
        std::this_thread::sleep_for(std::chrono::milliseconds(400));

        posestamped.pose.position.z += height_offset;
        lin(posestamped);

        // Move to the next spot
        posestamped.pose.position.y += box_offset;
        posestamped.pose.position.y += slot_offset;
      }

      // Go to the first object of the next row
      posestamped.pose.position.y -= slot_offset * (objects_y);
      posestamped.pose.position.x += slot_offset;
    }
  }

private:
  // ECBPMI gripper service client
  std::shared_ptr<rclcpp::Client<irc_ros_msgs::srv::GripperCommand>> gripper_client;
  std::shared_ptr<irc_ros_msgs::srv::GripperCommand::Request> request =
    std::make_shared<irc_ros_msgs::srv::GripperCommand::Request>();
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<PickAndPlaceVacuum>());
  rclcpp::shutdown();
  return 0;
}
