// Used the ROS2 driver for the iiwa by ICube as a template for the implementation
// https://github.com/ICube-Robotics/iiwa_ros2/tree/main/iiwa_controllers/external_torque_sensor_broadcaster

#pragma once

#include <array>
#include <string>
#include <unordered_map>

#include "irc_ros_hardware/CAN/modulestates.hpp"
#include "irc_ros_msgs/msg/can_module_states.hpp"
#include "semantic_components/semantic_component_interface.hpp"

// TODO: Use template or how to integrate different irc_ros_msgs types

namespace irc_ros_controllers
{

class DashboardSCI
: public semantic_components::SemanticComponentInterface<irc_ros_msgs::msg::CanModuleState>
{
public:
  explicit DashboardSCI(const std::string & name, const std::string module_type)
  : SemanticComponentInterface(name, state_interface_suffixes.size())
  {
    module_type_ = module_type;

    for (auto sis : state_interface_suffixes) {
      interface_names_.emplace_back(name_ + "/" + sis);
    }
  }

  virtual ~DashboardSCI() = default;

  DashboardSCI(const DashboardSCI &) = delete;
  DashboardSCI & operator=(const DashboardSCI &) = delete;

  std::string module_type_;

  // TODO: enums to string (Add map in modulestates.hpp)
  bool get_values_as_message(irc_ros_msgs::msg::CanModuleState & message)
  {
    message.name = name_;
    for (auto && si : state_interfaces_) {
      std::string name = si.get().get_name();

      if (name == (name_ + "/" + "temperature_board")) {
        message.temperature_board = si.get().get_value();
      } else if (name == (name_ + "/" + "temperature_motor")) {
        message.temperature_motor = si.get().get_value();
      } else if (name == (name_ + "/" + "can_id")) {
        std::stringstream ss;
        ss << "0x" << std::hex << static_cast<int>(si.get().get_value());
        message.can_id = ss.str().c_str();
      } else if (name == (name_ + "/" + "hardware_ident")) {
        message.hardware_ident = static_cast<int>(si.get().get_value());
      } else if (name == (name_ + "/" + "error_state")) {
        message.error_state = static_cast<int>(si.get().get_value());
      } else if (name == (name_ + "/" + "motor_state")) {
        message.motor_state = static_cast<int>(si.get().get_value());
      } else if (name == (name_ + "/" + "reset_state")) {
        message.reset_state = static_cast<int>(si.get().get_value());
      } else if (name == (name_ + "/" + "supply_voltage")) {
        message.supply_voltage = si.get().get_value();
      }
      // Dont react for errors yet, as not all interfaces are implemented
      //  else {
      //     message.name=si.get().get_value();
      //     message.can_id="not found";
      // }
    }

    return true;
  }

private:
  // TODO: Should this be passed from the dashboard_controller via the constructor?
  const std::array<std::string, 8> state_interface_suffixes = {
    "can_id",      "temperature_board", "temperature_motor", "hardware_ident",
    "error_state", "motor_state",       "reset_state",       "supply_voltage"};
};
}  // namespace irc_ros_controllers