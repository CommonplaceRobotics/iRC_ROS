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

/**
 * @brief Reads the configuration, starts publishers and services,
 * creates semantic component interface.
*/
controller_interface::CallbackReturn DashboardController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::string name = get_node()->get_name();
  get_node()->get_parameter("joints", joints_);
  get_node()->get_parameter("gpios", gpios_);

  dashboard_publisher_ = get_node()->create_publisher<irc_ros_msgs::msg::CanModuleStates>(
    (name + "/states").c_str(), rclcpp::SystemDefaultsQoS());

  can_module_service_ = get_node()->create_service<irc_ros_msgs::srv::CanModuleCommand>(
    name + "/dashboard_command", std::bind(
                                   &DashboardController::dashboard_command_callback, this,
                                   std::placeholders::_1, std::placeholders::_2));

  for (auto name : joints_) {
    module_interfaces_.push_back(std::move(std::make_unique<irc_ros_controllers::DashboardSCI>(
      name, "joint", module_state_interfaces_)));
  }

  for (auto name : gpios_) {
    module_interfaces_.push_back(std::move(
      std::make_unique<irc_ros_controllers::DashboardSCI>(name, "dio", module_state_interfaces_)));
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief Claim the dashboard command interfaces.
 */
controller_interface::InterfaceConfiguration DashboardController::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration config;

  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (auto name : joints_) {
    config.names.emplace_back(name + "/dashboard_command");
  }
  for (auto name : gpios_) {
    config.names.emplace_back(name + "/dashboard_command");
  }

  return config;
}

/**
 * @brief Sets the dashboard command interfaces.
 * 
 * While the SCI takes care of reading the values and creating the messages, it cannot configure
 * the state interfaces itself.
 */
controller_interface::InterfaceConfiguration DashboardController::state_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration config;

  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Configure the state interfaces required for DashboardSCI
  for (auto && module_interface : module_interfaces_) {
    auto module_names = module_interface->get_state_interface_names();
    config.names.insert(std::end(config.names), std::begin(module_names), std::end(module_names));
  }

  return config;
}
/**
 * @brief Assign state interfaces to SCIs.
 */
controller_interface::CallbackReturn DashboardController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto && module_interface : module_interfaces_) {
    // TODO: Currently assigns all state interfaces even if they dont belong to that module (Issue #75)
    module_interface->assign_loaned_state_interfaces(state_interfaces_);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief Release the state interfaces from the SCIs.
 */
controller_interface::CallbackReturn DashboardController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto && module_interface : module_interfaces_) {
    module_interface->release_interfaces();
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief Only calls the publish method.
 */
controller_interface::return_type DashboardController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  publish();

  return controller_interface::return_type::OK;
}

/**
 * @brief Send the messages created by the SCIs for each module
 */
void DashboardController::publish()
{
  auto module_states = irc_ros_msgs::msg::CanModuleStates();

  for (auto && module_interface : module_interfaces_) {
    auto module_state = irc_ros_msgs::msg::CanModuleState();

    module_interface->get_values_as_message(module_state);

    module_states.module_states.push_back(module_state);
  }

  dashboard_publisher_->publish(module_states);
}

/**
 * @brief Handle the dashboard command reception.
 */
void DashboardController::dashboard_command_callback(
  irc_ros_msgs::srv::CanModuleCommand_Request::SharedPtr req,
  irc_ros_msgs::srv::CanModuleCommand_Response::SharedPtr resp)
{
  std::string name = req->name;

  // Find position of req in command interfaces by the index in joints_/gpios_
  // Since the command interfaces are filled first with joints and then gpios we get this messy
  // search function
  int index;

  // Search through joints first
  auto iter = std::find(joints_.begin(), joints_.end(), name);
  if (iter != joints_.end()) {
    // Found, target is a joint at command_interfaces[index]
    index = iter - joints_.begin();
  } else {
    // Not found, search through the gpios next
    iter = std::find(gpios_.begin(), gpios_.end(), name);
    index = iter - gpios_.begin();
    if (iter != gpios_.end()) {
      // Found, target is a gpio at command_interfaces[index]

      // Since the command interfaces are stored as
      // [joint_1, ..., joint_n, gpio_1, ..., gpio_m]
      // we need to add the number of joints as an offset
      index = joints_.size() + iter - gpios_.begin();
    } else {
      // Name not found
      resp->success = false;
      return;
    }
  }

  command_interfaces_[index].set_value(req->type);

  // Wait timeout_ milliseconds for ack, else respond with error
  std::chrono::time_point<std::chrono::steady_clock> start = std::chrono::steady_clock::now();
  int reply_value;
  while (std::chrono::steady_clock::now() - start < timeout_) {
    reply_value = static_cast<int>(command_interfaces_[index].get_value());
    if (reply_value == 0) {
      // Success, the hardware interface acknowledged our message.
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  resp->success = reply_value == 0;
};

}  // namespace irc_ros_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  irc_ros_controllers::DashboardController, controller_interface::ControllerInterface)