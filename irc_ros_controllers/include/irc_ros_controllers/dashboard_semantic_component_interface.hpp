// Used the ROS2 driver for the iiwa by ICube as a template for the implementation
// https://github.com/ICube-Robotics/iiwa_ros2/tree/main/iiwa_controllers/external_torque_sensor_broadcaster

#pragma once

#include <array>
#include <string>
#include <unordered_map>

#include "irc_ros_hardware/CAN/modulestates.hpp"
#include "irc_ros_msgs/msg/can_module_states.hpp"
#include "semantic_components/semantic_component_interface.hpp"

namespace irc_ros_controllers
{

class DashboardSCI
: public semantic_components::SemanticComponentInterface<irc_ros_msgs::msg::CanModuleState>
{
public:
  explicit DashboardSCI(
    const std::string & name, const std::string module_type,
    std::vector<std::string> state_interfaces)
  : SemanticComponentInterface(name, state_interfaces.size()), module_type_(module_type)
  {
    for (auto si : state_interfaces) {
      interface_names_.emplace_back(name_ + "/" + si);
    }
  }

  virtual ~DashboardSCI() = default;
  DashboardSCI(const DashboardSCI &) = delete;
  DashboardSCI & operator=(const DashboardSCI &) = delete;

  // TODO: Cast the state enums to strings? (Add map in modulestates.hpp)
  // Easier to just print the states/show them inside a GUI, but then parsing the state
  // depends on the exact string assigned to it instead of an enum comparison, which on the other
  // hand would require to include the fitting .hpp file.
  bool get_values_as_message(irc_ros_msgs::msg::CanModuleState & message)
  {
    message.name = name_;
    for (auto && si : state_interfaces_) {
      std::string name = si.get().get_name();

      // Here all possible StateInterfaces need to be handled
      if (name == (name_ + "/" + "can_id")) {
        std::stringstream ss;
        ss << "0x" << std::hex << static_cast<int>(si.get().get_value());
        message.can_id = ss.str().c_str();
      } else if (name == (name_ + "/" + "hardware_ident")) {
        message.hardware_ident = static_cast<int>(si.get().get_value());
      } else if (name == (name_ + "/" + "version_major")) {
        message.version_major = static_cast<int>(si.get().get_value());
      } else if (name == (name_ + "/" + "version_minor")) {
        message.version_minor = static_cast<int>(si.get().get_value());
      } else if (name == (name_ + "/" + "temperature_board")) {
        message.temperature_board = si.get().get_value();
      } else if (name == (name_ + "/" + "temperature_motor")) {
        message.temperature_motor = si.get().get_value();
      } else if (name == (name_ + "/" + "supply_voltage")) {
        message.supply_voltage = si.get().get_value();
      } else if (name == (name_ + "/" + "motor_current")) {
        message.motor_current = si.get().get_value();
      } else if (name == (name_ + "/" + "error_state")) {
        message.error_state = static_cast<int>(si.get().get_value());
      } else if (name == (name_ + "/" + "reset_state")) {
        message.reset_state = static_cast<int>(si.get().get_value());
      } else if (name == (name_ + "/" + "motor_state")) {
        message.motor_state = static_cast<int>(si.get().get_value());
      } else if (name == (name_ + "/" + "command_mode")) {
        message.command_mode = static_cast<int>(si.get().get_value());
      } else {
        // Unexpected StateInterface
        // TODO: Once only the fitting interfaces are assigned in the dashboard controller return
        // false and handle the error. (Issue #75)
        // return false;
      }
    }

    return true;
  }

private:
  std::string module_type_;
};
}  // namespace irc_ros_controllers