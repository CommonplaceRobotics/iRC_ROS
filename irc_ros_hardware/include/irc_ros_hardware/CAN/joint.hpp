#pragma once

#include <bitset>
#include <deque>
#include <memory>
#include <string>

#include "irc_ros_hardware/CAN/can_interface.hpp"
#include "irc_ros_hardware/CAN/can_message.hpp"
#include "irc_ros_hardware/CAN/module.hpp"
#include "irc_ros_hardware/CAN/modulestates.hpp"

namespace irc_hardware
{
class Joint : public Module
{
public:
  using Ptr = std::shared_ptr<Joint>;

  Joint(std::string name, std::shared_ptr<CAN::CanInterface> can_interface, int can_id);
  ~Joint();

  int32_t encoder_pos_;
  double tics_over_degree_ = 1.0;  // [tics]/[deg]

  void position_cmd() override;
  void velocity_cmd() override;
  void torque_cmd() override;

  void set_position_to_zero() override;
  void referencing() override;
  void rotor_alignment() override;

  void read_can() override;
  void write_can() override;

private:
  void set_position_to_zero_callback(cprcan::bytevec response);
  void referencing_callback(cprcan::bytevec response);
  void rotor_alignment_callback(cprcan::bytevec response);
  void standard_response_callback(CAN::TimedCanMessage message);
  void encoder_message_callback(cprcan::bytevec data);
  void startup_message_callback(cprcan::bytevec data);
  void environmental_message_callback(cprcan::bytevec data);

  // Will be used for the zero-torque mode
  // bool zero_torque;

  //  While the documentation specifies these DIOs, the current motor
  //  boards dont have any outputs and only internally used inputs.
  //  Therefore don't use these two bitsets unless you know what you are
  //  doing!
  // std::bitset<4> digital_in;
  // std::bitset<8> digital_out;

  //  Saves the timestamp from the last position message
  //  used to estimate the velocity
  CAN::t_timestamp last_stamp_;

  // Used to filter the velocities
  std::deque<double> velocity_buffer_;
  static constexpr size_t velocity_buffer_size_ = 10;
  std::array<double, velocity_buffer_size_> velocity_buffer_weights_;
};

}  // namespace irc_hardware
