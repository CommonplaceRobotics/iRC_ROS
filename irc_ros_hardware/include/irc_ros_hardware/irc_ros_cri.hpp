#pragma once

#include <array>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "CRI/cri_messages.hpp"
#include "CRI/cri_socket.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

namespace irc_hardware
{
class IrcRosCri : public hardware_interface::SystemInterface
{
private:
  CriSocket crisocket;

  // This is the maximum number of joints that can be set with CRI
  // 6 Robot joints and 3 external axis
  // Also 3 values for gripper and 4 for mobile platform joints
  static constexpr uint8_t CRI_MAX_N_JOINTS = 9;

  cri_messages::Status currentStatus;

  // Current jogs
  // 0-5 are used for internal axis
  // 6-8 are used for external axis
  std::vector<double> jog_array;

  bool continueAlive;
  bool continueMessage;
  std::thread aliveThread;
  std::thread messageThread;
  int aliveWaitMs;

  int cmd_counter;
  std::mutex counterLock;
  std::mutex aliveLock;

  // Read joint position values
  std::vector<double> pos_;  // [rad]
  std::vector<double> vel_;  // Not yet implemented! [rad/s]

  // Values given from the hardware_interface, all have length of N_JOINTS; predefine?
  std::vector<double> set_pos_;  // [rad]
  std::vector<double> set_pos_last_;
  std::vector<double> set_vel_;  // percentage of the maximum speed

  // Used to counteract the offsets in EmbeddedCtrl, read from .ros2_control.xacro files
  std::vector<double> pos_offset_;  // [rad]

  // Not implemented!
  std::vector<double> digital_in_double_;
  std::vector<double> digital_out_double_;

  cri_messages::Kinstate lastKinstate;
  std::array<int, 16> lastErrorJoints;
  std::string kinstateMessage;

  // Thread functions
  void AliveThreadFunction();
  void MessageThreadFunction();

  // Other functions
  int get_cmd_counter();
  void Command(const std::string &);
  void GetConfig(const std::string &);

  void GetReferencingInfo();

  // Function to react to specific status values, to display warnings, error messages, etc.
  void ProcessStatus(const cri_messages::Status &);

  bool to_move = false;
  double move_velocity = 50.0f;
  void CmdMove();
  void CmdDout();

public:
  IrcRosCri();
  ~IrcRosCri();

  // ROS2 Control functions
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;
};
}  // namespace irc_hardware
