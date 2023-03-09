#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "example_interfaces/srv/set_bool.hpp"
#include "irc_ros_msgs/msg/gripper_command.hpp"
#include "irc_ros_msgs/msg/gripper_state.hpp"
#include "irc_ros_msgs/srv/gripper_command.hpp"
#include "std_msgs/msg/bool.hpp"

namespace irc_ros_controllers
{

class EcbpmiController : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_init() override;

private:
  void set_gripper_srv_callback(
    const std::shared_ptr<irc_ros_msgs::srv::GripperCommand::Request> req,
    std::shared_ptr<irc_ros_msgs::srv::GripperCommand::Response> resp);

  void set_gripper_sub_callback(const irc_ros_msgs::msg::GripperCommand & command);

  void publish_state();

  // Service, Subscriber, Publisher for all in-/outputs
  rclcpp::Service<irc_ros_msgs::srv::GripperCommand>::SharedPtr ecbpmi_service;
  rclcpp::Subscription<irc_ros_msgs::msg::GripperCommand>::SharedPtr ecbpmi_subscriber;
  rclcpp::Publisher<irc_ros_msgs::msg::GripperState>::SharedPtr ecbpmi_publisher;

  // Names parsed from the controller.yaml
  std::string output_grip;
  std::string output_release;
  std::string input_grasped;

  // How long to wait for the successfully grasped signal
  const std::chrono::duration<int64_t, std::milli> timeout = std::chrono::milliseconds(1000);
};
}  // namespace irc_ros_controllers
