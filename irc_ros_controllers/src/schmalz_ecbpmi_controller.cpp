#include "irc_ros_controllers/schmalz_ecbpmi_controller.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <map>
#include <thread>

#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

namespace irc_ros_controllers
{

controller_interface::CallbackReturn EcbpmiController::on_init()
{
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration EcbpmiController::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration config;

  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  config.names.emplace_back(output_grip);
  config.names.emplace_back(output_release);

  return config;
}

controller_interface::InterfaceConfiguration EcbpmiController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;

  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  config.names.emplace_back(input_grasped);

  return config;
}

controller_interface::return_type EcbpmiController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  RCLCPP_DEBUG(rclcpp::get_logger("iRC_ROS::EcbpmiController"), "update");
  publish_state();

  return controller_interface::return_type::OK;
}

void EcbpmiController::set_gripper_srv_callback(
  const std::shared_ptr<irc_ros_msgs::srv::GripperCommand::Request> req,
  std::shared_ptr<irc_ros_msgs::srv::GripperCommand::Response> resp)
{
  // Set state to grip/release
  command_interfaces_[0].set_value(req->grip);
  command_interfaces_[1].set_value(!req->grip);

  std::chrono::time_point<std::chrono::steady_clock> start = std::chrono::steady_clock::now();
  bool gripped;

  while (std::chrono::steady_clock::now() - start < timeout) {
    gripped = static_cast<int>(state_interfaces_[0].get_value());
    if (req->grip == gripped) {
      break;
    }

    std::this_thread::yield();
  }

  resp->gripped = gripped;
}

void EcbpmiController::set_gripper_sub_callback(const irc_ros_msgs::msg::GripperCommand & cmd)
{
  command_interfaces_[0].set_value(cmd.grip);
  command_interfaces_[1].set_value(!cmd.grip);
}

void EcbpmiController::publish_state()
{
  bool value = static_cast<bool>(state_interfaces_[0].get_value());

  irc_ros_msgs::msg::GripperState msg;

  msg.header = std_msgs::msg::Header();
  msg.header.stamp = get_node()->get_clock()->now();
  msg.grasped = value;

  ecbpmi_publisher->publish(msg);

  RCLCPP_DEBUG(rclcpp::get_logger("iRC_ROS::EcbpmiController"), "Published inputs");
}

controller_interface::CallbackReturn EcbpmiController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::string name = get_node()->get_name();
  get_node()->get_parameter("output_grip", output_grip);
  get_node()->get_parameter("output_release", output_release);
  get_node()->get_parameter("input_grasped", input_grasped);

  ecbpmi_service = get_node()->create_service<irc_ros_msgs::srv::GripperCommand>(
    name + "/set_gripper", std::bind(
                             &EcbpmiController::set_gripper_srv_callback, this,
                             std::placeholders::_1, std::placeholders::_2));

  ecbpmi_subscriber = get_node()->create_subscription<irc_ros_msgs::msg::GripperCommand>(
    (name + "/set_gripper").c_str(), 10,
    std::bind(&EcbpmiController::set_gripper_sub_callback, this, std::placeholders::_1));

  ecbpmi_publisher = get_node()->create_publisher<irc_ros_msgs::msg::GripperState>(
    (name + "/gripper_state").c_str(), rclcpp::SystemDefaultsQoS());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn EcbpmiController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn EcbpmiController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace irc_ros_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  irc_ros_controllers::EcbpmiController, controller_interface::ControllerInterface)