// Inspired by DIO Controller
// 
// Not a controller, only forwards the twist command published on /cmd_vel to the system_interface

#pragma once

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>
#include <unordered_map>

#include "irc_ros_msgs/msg/dio_command.hpp"
#include "irc_ros_msgs/msg/dio_int_command.hpp"

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_msgs/msg/int16.h"
// #include "tf2_msgs/msg/tf_message.hpp"

namespace irc_ros_controllers
{

class PlatformController : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_init() override;

private:

bool is_halted = false;

bool subscriber_is_active_ = false;
rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_command_subscriber_ = nullptr;
rclcpp::Subscription<irc_ros_msgs::msg::DioCommand>::SharedPtr reset_enable_subscriber = nullptr;
rclcpp::Subscription<irc_ros_msgs::msg::DioIntCommand>::SharedPtr change_motiontype_subscriber = nullptr;

void receive_velocity_callback_sub_(const geometry_msgs::msg::Twist & cmd);
void set_outputs_sub_callback(const irc_ros_msgs::msg::DioCommand & command);

void set_motiontype_sub_callback(const irc_ros_msgs::msg::DioIntCommand & command);

std::queue<geometry_msgs::msg::Twist> previous_commands_; 

std::vector<std::string> commandedTwist;
std::vector<std::string> currentTwist;

std::vector<std::string> digital_inputs;
std::vector<std::string> digital_outputs;

void halt();

//   // Service, Subscriber, Publisher for all in-/outputs
//   rclcpp::Service<irc_ros_msgs::srv::DioCommand>::SharedPtr outputs_service;
//   rclcpp::Subscription<irc_ros_msgs::msg::DioCommand>::SharedPtr outputs_subscriber;
//   rclcpp::Publisher<irc_ros_msgs::msg::DioState>::SharedPtr outputs_publisher;
//   rclcpp::Publisher<irc_ros_msgs::msg::DioState>::SharedPtr inputs_publisher;

//   // Names parsed from the controller.yaml
//   std::vector<std::string> digital_inputs;
//   std::vector<std::string> digital_outputs;
};
}  // namespace irc_ros_controllers