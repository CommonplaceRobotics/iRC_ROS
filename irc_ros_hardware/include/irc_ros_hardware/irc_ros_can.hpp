#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "CAN/can_socket.hpp"
#include "CAN/module.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

namespace irc_hardware
{
class IrcRosCan : public hardware_interface::SystemInterface
{
public:
  IrcRosCan();
  ~IrcRosCan();

  // ROS2 Control functions
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & /*start_interfaces*/,
    const std::vector<std::string> & /*stop_interfaces*/) override;
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & /*start_interfaces*/,
    const std::vector<std::string> & /*stop_interfaces*/) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  std::shared_ptr<CAN::CanInterfaceSocketCAN> can_interface_;

  std::unordered_map<std::string, Module::Ptr> modules_;

  // Name of the can socket
  std::string can_socket_;

  // Default values if nothing is specified in the .ros2_control.urdf
  const std::string default_can_socket_ = "can0";

  // This corrosponds to the usual used offset and
  // two steps on the CAN address selection wheel on DIN rail modules
  const int default_can_id_step_ = 0x10;

  const double default_gear_scale_ = 10.0;

  // Timeouts for the startup process
  const std::chrono::duration<int64_t> reset_timeout_ = std::chrono::seconds(3);
  const std::chrono::duration<int64_t> motor_enable_timeout_ = std::chrono::seconds(3);
};
}  // namespace irc_hardware
