/**
 * @brief CPR CAN v2 protocol implementation
 * 
 * This file provides most of the functionality of the current version of
 * the CPR CAN v2 protocol. This is aimed at the closed-loops controllers found
 * in the igus ReBeL robots, but most functionality should be compatible with
 * the open loop controllers found in other setups.
 *  
 * The protocol documentation can be found here:
 * https://cpr-robots.com/download/CAN/CPR_CAN_Protocol_V2_UserGuide_en.pdf
 *
 * Not implemented are the EEPROM commands and the extended error messages
 * If you have to update EEPROM values and know what you are doing please use
 * the ModuleControl tool 
 * 
 * This can be found here:
 * https://wiki.cpr-robots.com/index.php/Config_Software_ModuleCtrl
 */

#pragma once

#include <algorithm>
#include <bitset>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "irc_ros_hardware/CAN/can_socket.hpp"
#include "irc_ros_hardware/CAN/cprcan_v2.hpp"
#include "irc_ros_hardware/CAN/modulestates.hpp"
#include "irc_ros_hardware/common/errorstate.hpp"

namespace irc_hardware
{

class Module
{
public:
  using Ptr = std::shared_ptr<Module>;

  Module(std::string name, std::shared_ptr<CAN::CanInterface> can_interface, int can_id);

  const std::string get_name() { return name_; };

  virtual void prepare_movement();
  virtual bool is_ready_to_move() = 0;

  virtual void position_cmd() = 0;
  virtual void velocity_cmd() = 0;
  virtual void torque_cmd() = 0;

  virtual void reset_error(bool force = false);
  virtual void set_position_to_zero() = 0;
  virtual void enable_motor();
  virtual void disable_motor();
  virtual void referencing() = 0;
  virtual void rotor_alignment() = 0;

  void ping();

  // Not implemented
  // void eepromWriteEnable();
  // void saveParameter();
  // void readParameter();

  void update_double_copies();

  virtual void read_can() = 0;
  virtual void write_can() = 0;

  void dashboard_command();

  // Public member variables which are set during the startup procedure
  const std::string name_;
  const int can_id_;
  double temperature_scale_;
  PositioningReadyState positioningReadyState;  //set first in the Rebel_CAN::on_init() function
  MotorState motorState = MotorState::disabled;
  ReferenceState referenceState = ReferenceState::not_required;
  int reference_priority_ = std::numeric_limits<int>::max();

  // Public member variables which are accessed by controllers
  // DIO Controller variables
  std::vector<double> digital_in_double_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> digital_out_double_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // Dashboard Controller variables
  double temperature_board_;
  double temperature_motor_;

  // Dashboard Controller variables
  // double type copies for compatibility, refreshed via update_double_copies()
  double can_id_double_;
  double hardware_ident_double_;
  double version_major_double_;
  double version_minor_double_;
  double error_state_double_;
  double motor_state_double_;
  double reset_state_double_;
  double supply_voltage_double_;
  double command_mode_double_;

  double dashboard_cmd_double_ = 0.0;

  // Standard ROS2 Control controller variables
  double pos_ = std::numeric_limits<double>::quiet_NaN();
  double set_pos_ = std::numeric_limits<double>::quiet_NaN();
  double vel = std::numeric_limits<double>::quiet_NaN();
  double set_vel_ = std::numeric_limits<double>::quiet_NaN();
  double set_torque_ = std::numeric_limits<double>::quiet_NaN();

  CommandMode commandMode = CommandMode::none;

protected:
  std::bitset<8> digital_in_;
  std::bitset<8> digital_out_;

  // Read via startup message
  HardwareIdent hardwareIdent;
  std::array<uint8_t, 2> version_ = {0, 0};  // major, minor version

  // Internal states
  ErrorState errorState;
  ErrorState lastErrorState;
  SetToZeroState setToZeroState = SetToZeroState::not_zeroed;
  RotorAlignmentState rotorAlignmentState = RotorAlignmentState::unaligned;
  ResetState resetState = ResetState::not_reset;

  // TimePoints for timeouting failed commands
  std::chrono::time_point<std::chrono::steady_clock> motor_state_time_point_;
  std::chrono::time_point<std::chrono::steady_clock> reset_time_point_;

  // Timeout times
  const std::chrono::duration<int64_t> motor_state_timeout_ = std::chrono::seconds(1);
  const std::chrono::duration<int64_t> reset_timeout_ = std::chrono::seconds(1);

  u_int8_t msg_counter_ = 0;
  std::shared_ptr<CAN::CanInterface> can_interface_;

  u_int16_t supply_voltage_ = 0;
};

}  // namespace irc_hardware