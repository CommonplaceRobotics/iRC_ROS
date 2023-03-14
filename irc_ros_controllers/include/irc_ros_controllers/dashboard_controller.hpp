#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "irc_ros_controllers/dashboard_semantic_component_interface.hpp"
#include "irc_ros_msgs/msg/can_module_states.hpp"
#include "irc_ros_msgs/srv/can_module_command.hpp"

namespace irc_ros_controllers
{

class DashboardController : public controller_interface::ControllerInterface
{
public:
  CallbackReturn on_init() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void publish();

  void dashboard_command_callback(
    irc_ros_msgs::srv::CanModuleCommand_Request::SharedPtr req,
    irc_ros_msgs::srv::CanModuleCommand_Response::SharedPtr resp);

  // Service, Subscriber, Publisher for all in-/outputs
  rclcpp::Publisher<irc_ros_msgs::msg::CanModuleStates>::SharedPtr dashboard_publisher;
  rclcpp::Service<irc_ros_msgs::srv::CanModuleCommand>::SharedPtr can_module_service;
  std::vector<std::unique_ptr<irc_ros_controllers::DashboardSCI>> module_interfaces;

  std::vector<std::string> gpios;
  std::vector<std::string> joints;

  // TODO: Move which interfaces to subscribe to to controller config yaml?
  std::vector<std::string> module_command_interfaces = {
    "dashboard_command",
  };
  std::vector<std::string> module_state_interfaces = {
    "can_id",      "temperature_board", "temperature_motor", "hardware_ident",
    "error_state", "motor_state",       "reset_state",       "supply_voltage"};

  // How long to wait for the acknowledgement of commands by the hardware interface
  const std::chrono::duration<int64_t, std::milli> timeout = std::chrono::milliseconds(1000);
};
}  // namespace irc_ros_controllers