// Inspired by UR ROS2 driver
// https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/main/ur_controllers/include/ur_controllers/gpio_controller.hpp

#include "irc_ros_controllers/dio_controller.hpp"

#include <algorithm>
#include <functional>
#include <map>

#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

namespace irc_ros_controllers
{

controller_interface::CallbackReturn DIOController::on_init()
{
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration DIOController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;

  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (auto name : digital_outputs) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("iRC_ROS::DIOController"), "Adding command interface %s", name.c_str());
    config.names.emplace_back(name);
  }

  return config;
}

controller_interface::InterfaceConfiguration DIOController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;

  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (auto name : digital_inputs) {
    RCLCPP_INFO(
      rclcpp::get_logger("iRC_ROS::DIOController"), "Adding state interface %s", name.c_str());
    config.names.emplace_back(name);
  }

  return config;
}

controller_interface::return_type DIOController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  publish();

  return controller_interface::return_type::OK;
}

void DIOController::set_outputs_service_callback(
  irc_ros_msgs::srv::DioCommand_Request::SharedPtr req,
  irc_ros_msgs::srv::DioCommand_Response::SharedPtr resp)
{
  if (req->names.size() == req->outputs.size()) {
    resp->success = true;

    // TODO: Find an elegant solution to iterate over cmd instead of command_interfaces_
    for (auto && ci : command_interfaces_) {
      auto it = std::find(req->names.begin(), req->names.end(), ci.get_name());

      if (it != req->names.end()) {
        auto value = static_cast<int>(req->outputs[it - req->names.begin()]);

        ci.set_value(value);

        RCLCPP_INFO(
          rclcpp::get_logger("iRC_ROS::DIOController"), "Output %s set to %i",
          ci.get_name().c_str(), value);
      }
    }
  } else {
    RCLCPP_ERROR(
      rclcpp::get_logger("iRC_ROS::DIOController"), "Number of outputs and names are not equal");
    resp->success = false;
  }
}

void DIOController::set_outputs_sub_callback(const irc_ros_msgs::msg::DioCommand & cmd)
{
  if (cmd.names.size() == cmd.outputs.size()) {
    // TODO: Find an elegant solution to iterate over cmd instead of command_interfaces_
    for (auto && ci : command_interfaces_) {
      auto it = std::find(cmd.names.begin(), cmd.names.end(), ci.get_name());

      if (it != cmd.names.end()) {
        auto value = cmd.outputs[it - cmd.names.begin()];

        ci.set_value(value);

        RCLCPP_INFO(
          rclcpp::get_logger("iRC_ROS::DIOController"), "Output %s set to %i",
          ci.get_name().c_str(), value);
      }
    }
  } else {
    RCLCPP_ERROR(
      rclcpp::get_logger("iRC_ROS::DIOController"), "Number of outputs and names are not equal");
  }
}

void DIOController::publish()
{
  std::map<std::string, bool> digital_inputs;

  for (auto && si : state_interfaces_) {
    std::string name = si.get_name();
    bool value = static_cast<bool>(si.get_value());

    // Add to map
    digital_inputs[name] = value;
  }

  irc_ros_msgs::msg::DioState msg;
  for (auto const & [name, value] : digital_inputs) {
    msg.names.push_back(name);
    msg.inputs.push_back(value);
  }

  msg.header = std_msgs::msg::Header();
  msg.header.stamp = get_node()->get_clock()->now();

  inputs_publisher->publish(msg);

  std::map<std::string, bool> digital_outputs;

  for (auto && ci : command_interfaces_) {
    std::string name = ci.get_name();
    bool value = static_cast<bool>(ci.get_value());

    // Add to map
    digital_outputs[name] = value;
  }

  irc_ros_msgs::msg::DioState out_msg;
  for (auto const & [name, value] : digital_outputs) {
    out_msg.names.push_back(name);
    out_msg.outputs.push_back(value);
  }

  out_msg.header = std_msgs::msg::Header();
  out_msg.header.stamp = get_node()->get_clock()->now();

  outputs_publisher->publish(out_msg);

  RCLCPP_DEBUG(rclcpp::get_logger("iRC_ROS::DIOController"), "Published");
}

controller_interface::CallbackReturn DIOController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::string name = get_node()->get_name();
  get_node()->get_parameter("digital_inputs", digital_inputs);
  get_node()->get_parameter("digital_outputs", digital_outputs);

  RCLCPP_DEBUG(
    rclcpp::get_logger("iRC_ROS::DIOController"),
    "Got %li inputs and %li outputs from the configuration file", digital_inputs.size(),
    digital_outputs.size());

  // Output service and subscriber
  outputs_service = get_node()->create_service<irc_ros_msgs::srv::DioCommand>(
    (name + "/set_outputs").c_str(), std::bind(
                                       &DIOController::set_outputs_service_callback, this,
                                       std::placeholders::_1, std::placeholders::_2));

  outputs_subscriber = get_node()->create_subscription<irc_ros_msgs::msg::DioCommand>(
    (name + "/set_outputs").c_str(), 10,
    std::bind(&DIOController::set_outputs_sub_callback, this, std::placeholders::_1));

  outputs_publisher = get_node()->create_publisher<irc_ros_msgs::msg::DioState>(
    (name + "/get_outputs").c_str(), rclcpp::SystemDefaultsQoS());

  inputs_publisher = get_node()->create_publisher<irc_ros_msgs::msg::DioState>(
    (name + "/get_inputs").c_str(), rclcpp::SystemDefaultsQoS());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DIOController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DIOController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace irc_ros_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  irc_ros_controllers::DIOController, controller_interface::ControllerInterface)