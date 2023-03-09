// Inspired by UR ROS2 driver
// https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/main/ur_controllers/include/ur_controllers/gpio_controller.hpp

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "irc_ros_msgs/msg/dio_command.hpp"
#include "irc_ros_msgs/msg/dio_state.hpp"
#include "irc_ros_msgs/srv/dio_command.hpp"
#include "std_msgs/msg/bool.hpp"

namespace irc_ros_controllers
{

class DIOController : public controller_interface::ControllerInterface
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
  void set_outputs_service_callback(
    irc_ros_msgs::srv::DioCommand_Request::SharedPtr req,
    irc_ros_msgs::srv::DioCommand_Response::SharedPtr resp);

  void set_outputs_sub_callback(const irc_ros_msgs::msg::DioCommand & command);

  void publish();

  // Service, Subscriber, Publisher for all in-/outputs
  rclcpp::Service<irc_ros_msgs::srv::DioCommand>::SharedPtr outputs_service;
  rclcpp::Subscription<irc_ros_msgs::msg::DioCommand>::SharedPtr outputs_subscriber;
  rclcpp::Publisher<irc_ros_msgs::msg::DioState>::SharedPtr outputs_publisher;
  rclcpp::Publisher<irc_ros_msgs::msg::DioState>::SharedPtr inputs_publisher;

  // Names parsed from the controller.yaml
  std::vector<std::string> digital_inputs;
  std::vector<std::string> digital_outputs;
};
}  // namespace irc_ros_controllers