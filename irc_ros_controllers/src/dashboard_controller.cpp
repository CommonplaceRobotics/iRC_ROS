#include "irc_ros_controllers/dashboard_controller.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <map>
#include <ranges>

#include "irc_ros_controllers/dashboard_semantic_component_interface.hpp"
#include "irc_ros_msgs/srv/can_module_command.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

namespace irc_ros_controllers
{

controller_interface::CallbackReturn DashboardController::on_init()
{
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration DashboardController::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration config;

  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (auto name : joints) {
    config.names.emplace_back(name + "/dashboard_command");
  }
  for (auto name : gpios) {
    config.names.emplace_back(name + "/dashboard_command");
  }

  return config;
}

controller_interface::InterfaceConfiguration DashboardController::state_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration config;

  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Configure the state interfaces required for DashboardSCI
  for (auto && module_interface : module_interfaces) {
    auto module_names = module_interface->get_state_interface_names();
    config.names.insert(std::end(config.names), std::begin(module_names), std::end(module_names));
  }

  return config;
}

controller_interface::return_type DashboardController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  publish();

  return controller_interface::return_type::OK;
}

void DashboardController::publish()
{
  auto module_states = irc_ros_msgs::msg::CanModuleStates();

  for (auto && module_interface : module_interfaces) {
    auto module_state = irc_ros_msgs::msg::CanModuleState();

    module_interface->get_values_as_message(module_state);

    module_states.module_states.push_back(module_state);
  }

  dashboard_publisher->publish(module_states);

  // state_interfaces_->get_name();
}

void DashboardController::dashboard_command_callback(
  irc_ros_msgs::srv::CanModuleCommand_Request::SharedPtr req,
  irc_ros_msgs::srv::CanModuleCommand_Response::SharedPtr resp)
{
  std::string name = req->name;

  // TODO: Find position of req in joints/gpios
  int index;
  auto iter = std::find(joints.begin(), joints.end(), name);
  if (iter != joints.end()) {
    index = iter - joints.begin();
  } else {
    iter = std::find(gpios.begin(), gpios.end(), name);
    index = iter - gpios.begin();
    if (iter != gpios.end()) {
      index = joints.size() + iter - gpios.begin();
    } else {
      // ERR: Not found
      resp->success = false;
      return;
    }
  }

  command_interfaces_[index].set_value(req->type);

  // Wait timeout milliseconds for ack, else respond with error
  std::chrono::time_point<std::chrono::steady_clock> start = std::chrono::steady_clock::now();
  int reply_value;
  while (std::chrono::steady_clock::now() - start < timeout) {
    reply_value = static_cast<int>(command_interfaces_[index].get_value());
    if (reply_value == 0) {
      // Success
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  resp->success = reply_value == 0;
};

controller_interface::CallbackReturn DashboardController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::string name = get_node()->get_name();
  get_node()->get_parameter("gpios", gpios);
  get_node()->get_parameter("joints", joints);

  dashboard_publisher = get_node()->create_publisher<irc_ros_msgs::msg::CanModuleStates>(
    (name + "/states").c_str(), rclcpp::SystemDefaultsQoS());

  can_module_service = get_node()->create_service<irc_ros_msgs::srv::CanModuleCommand>(
    name + "/dashboard_command", std::bind(
                                   &DashboardController::dashboard_command_callback, this,
                                   std::placeholders::_1, std::placeholders::_2));

  for (auto name : joints) {
    module_interfaces.push_back(
      std::move(std::make_unique<irc_ros_controllers::DashboardSCI>(name, "joint")));
  }

  for (auto name : gpios) {
    module_interfaces.push_back(
      std::move(std::make_unique<irc_ros_controllers::DashboardSCI>(name, "dio")));
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DashboardController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto && module_interface : module_interfaces) {
    module_interface->release_interfaces();
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DashboardController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto && module_interface : module_interfaces) {
    // TODO: Currently assigns all state interfaces even if they dont belong to that module
    module_interface->assign_loaned_state_interfaces(state_interfaces_);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace irc_ros_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  irc_ros_controllers::DashboardController, controller_interface::ControllerInterface)