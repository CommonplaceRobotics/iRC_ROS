#include "irc_ros_hardware/CAN/module.hpp"

#include <array>
#include <bitset>

#include "irc_ros_msgs/srv/can_module_command.hpp"

namespace irc_hardware
{

Module::Module(std::string name, std::shared_ptr<CAN::CanInterface> can_interface, int can_id)
: name_(name), can_id_(can_id), can_interface_(can_interface)
{
}

/**
 * @brief Resets all errors. Afterwards only MNE might be present as it is supposed to be set
 * automatically.
 * 
 * Since reset_error also sets MNE it can be used to stop the motors.
 */
void Module::reset_error()
{
  // Only sent reset if not already send recently
  if (resetState != ResetState::reset && resetState != ResetState::resetting) {
    RCLCPP_INFO(rclcpp::get_logger("iRC_ROS"), "Module 0x%02x: Resetting", can_id_);

    CAN::CanMessage message(can_id_, cprcan::reset_error);
    can_interface_->write_message(message);

    resetState = ResetState::resetting;

    reset_time_point_ = std::chrono::steady_clock::now();
  } else if (
    resetState == ResetState::resetting &&
    std::chrono::steady_clock::now() - reset_time_point_ > reset_timeout_) {
    resetState = ResetState::not_reset;
    RCLCPP_WARN(rclcpp::get_logger("iRC_ROS"), "Module 0x%02x: Resetting timed out", can_id_);

    reset_error();
  }
}

void Module::reset_error_callback(cprcan::bytevec response)
{
  RCLCPP_INFO(rclcpp::get_logger("iRC_ROS"), "Module 0x%02x: Error reset acknowledged", can_id_);

  // Dont set it to finished quite yet, wait for the error bit to be 0;
  // resetState = ResetState::reset;
}

/**
 * @brief Sends the enable message to the controller if the motor is not already
 * enabled. The state is then set to enabling, and will be set to enabled once
 * read_can() does not find a MNE fault in the motion response message.
 */
void Module::enable_motor()
{
  if (motorState != MotorState::enabled && motorState != MotorState::enabling) {
    RCLCPP_INFO(rclcpp::get_logger("iRC_ROS"), "Module 0x%02x: Enabling motor", can_id_);

    CAN::CanMessage message(can_id_, cprcan::enable_motor);
    can_interface_->write_message(message);

    motorState = MotorState::enabling;

    motor_state_time_point_ = std::chrono::steady_clock::now();
  } else if (
    motorState == MotorState::enabling &&
    std::chrono::steady_clock::now() - motor_state_time_point_ > motor_state_timeout_) {
    motorState = MotorState::disabled;
    RCLCPP_WARN(rclcpp::get_logger("iRC_ROS"), "Module 0x%02x: Enabling motor timed out", can_id_);
  }
}

void Module::enable_motor_callback(cprcan::bytevec response)
{
  if (motorState != MotorState::enabled) {
    RCLCPP_INFO(rclcpp::get_logger("iRC_ROS"), "Module 0x%02x: Motor enabled", can_id_);
    motorState = MotorState::enabled;
  }
}

/**
 * @brief Sends the disable message to the controller if the motor is not already
 * disabled. The state is then set to disabling, and will be set to disabled once
 * read_can() does finds a MNE fault in the motion response message.
 */
void Module::disable_motor()
{
  if (motorState != MotorState::disabled && motorState != MotorState::disabling) {
    RCLCPP_INFO(rclcpp::get_logger("iRC_ROS"), "Module 0x%02x: Disabling motor", can_id_);

    CAN::CanMessage message(can_id_, cprcan::disable_motor);
    can_interface_->write_message(message);

    motorState = MotorState::disabling;

    motor_state_time_point_ = std::chrono::steady_clock::now();
  } else if (
    motorState == MotorState::disabling &&
    std::chrono::steady_clock::now() - motor_state_time_point_ > motor_state_timeout_) {
    motorState = MotorState::enabled;
    RCLCPP_WARN(rclcpp::get_logger("iRC_ROS"), "Module 0x%02x: Disabling motor timed out", can_id_);
  }
}

void Module::disable_motor_callback(cprcan::bytevec response)
{
  if (motorState != MotorState::disabled) {
    RCLCPP_INFO(rclcpp::get_logger("iRC_ROS"), "Module 0x%02x: Motor disabled", can_id_);
    motorState = MotorState::disabled;
  }
}
/**
 * @brief Sets the motor to a ready state. This means resetting any
 * errors besides MNE and enabling the motors
 *
 * Contrary to its name it is also used to prepare the DIO modules, as it
 * contains the reset error calls.
 * 
 * TODO: Should this be renamed and split up between dio and joint classes?
 */
void Module::prepare_movement()
{
  // If we are in position mode save the current position as set_pos_,
  // as after reset enable it might have changed a bit.
  if (commandMode == CommandMode::position) {
    set_pos_ = pos_;
  }

  if (errorState.any()) {
    RCLCPP_INFO(
      rclcpp::get_logger("iRC_ROS"), "Module 0x%02x: Errors%s detected, resetting", can_id_,
      errorState.str().c_str());

    reset_error();
  }

  if (!errorState.any_except_mne() && motorState != MotorState::enabled) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("iRC_ROS"), "Module 0x%02x: Motor not enabled, enabling before movement",
      can_id_);
    enable_motor();
  }
}

/**
 * @brief Sends a ping message to the module. The reply should be a startup message.
 */
void Module::ping()
{
  CAN::CanMessage message(can_id_, cprcan::ping);
  can_interface_->write_message(message);
}

/**
 * @brief Updates the double copies used for the ros2_control controller interface.
 * Must be called periodically, which happens in irc_ros_can.cpp.
*/
void Module::update_double_copies()
{
  can_id_double_ = static_cast<double>(can_id_);
  hardware_ident_double_ = static_cast<double>(hardwareIdent);
  version_major_double_ = static_cast<double>(version_[0]);
  version_minor_double_ = static_cast<double>(version_[1]);
  controller_type_double_ = static_cast<double>(controllerType);
  // TODO: find an acceptable way to transmit the different error states.
  error_state_double_ = static_cast<double>(errorState.to_uint8());
  motor_state_double_ = static_cast<double>(motorState);
  reset_state_double_ = static_cast<double>(resetState);
  command_mode_double_ = static_cast<double>(commandMode);

  // Convert mV to V
  supply_voltage_double_ = static_cast<double>(supply_voltage_) / 1000.0;
  // Convert mA to A
  motor_current_double_ = static_cast<double>(motor_current_) / 1000.0;
}

/**
 * @brief Handles received commands from an external program, e.g. the dashboard.
 * The command is executed and the reception, not the successful execution, is
 * acknowledged by setting the cmd to 0.
 */
void Module::dashboard_command()
{
  int command = static_cast<int>(dashboard_cmd_double_);
  switch (command) {
    case irc_ros_msgs::srv::CanModuleCommand_Request_<std::allocator<void>>::TYPE_NONE:
      // This case is to discern no command and unrecognised commands
      dashboard_cmd_double_ = 0;
      break;
    case irc_ros_msgs::srv::CanModuleCommand_Request_<std::allocator<void>>::TYPE_PING:
      ping();
      dashboard_cmd_double_ = 0;
      break;
    case irc_ros_msgs::srv::CanModuleCommand_Request_<std::allocator<void>>::TYPE_ERROR_RESET:
      reset_error();
      dashboard_cmd_double_ = 0;
      break;
    case irc_ros_msgs::srv::CanModuleCommand_Request_<std::allocator<void>>::TYPE_ENABLE_MOTOR:
      enable_motor();
      dashboard_cmd_double_ = 0;
      break;
    case irc_ros_msgs::srv::CanModuleCommand_Request_<std::allocator<void>>::TYPE_DISABLE_MOTOR:
      disable_motor();
      dashboard_cmd_double_ = 0;
      break;
    case irc_ros_msgs::srv::CanModuleCommand_Request_<std::allocator<void>>::TYPE_REFERENCE:
      referencing();
      dashboard_cmd_double_ = 0;
      break;
    case irc_ros_msgs::srv::CanModuleCommand_Request_<std::allocator<void>>::TYPE_SET_POS_TO_ZERO:
      set_position_to_zero();
      dashboard_cmd_double_ = 0;
      break;
    default:
      RCLCPP_ERROR(
        rclcpp::get_logger("iRC_ROS"),
        "Module 0x%02x: Unrecognised command received with enum value: %i", can_id_, command);
      dashboard_cmd_double_ = -1;
      break;
  }
}

}  // namespace irc_hardware
