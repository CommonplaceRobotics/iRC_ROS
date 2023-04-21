#include "irc_ros_hardware/CAN/digital_io.hpp"

namespace irc_hardware
{

DIO::DIO(std::string name, std::shared_ptr<CAN::CanInterface> can_interface, int can_id)
: Module(name, can_interface, can_id)
{
  // TODO: Once these get dynamically set from the irc_ros_can.cpp/on_init() function set the
  // log level to warning and set the sizes dynamically.
  if (digital_out_double_.size() != digital_out_.size()) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("iRC_ROS"),
      "Module 0x%02x: digital_out bitset and double vectors have different sizes of %li and %li. "
      "Therefore only %li outputs are available.",
      can_id, digital_out_.size(), digital_out_double_.size(),
      std::min(digital_out_.size(), digital_out_double_.size()));
  }
};

/**
 * @brief Reads out the last received message for the can ids belonging to this module.
 * Similar to the Joint::read_can() method but much shorter.
 */
void DIO::read_can()
{
  CAN::TimedCanMessage message;

  // standard response
  if (can_interface_->get_next_message(can_id_ + 1, message)) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("iRC_ROS"), "Module 0x%02x: Standard response received", can_id_);

    // Error
    std::bitset<8> error_code = message.data[0];
    errorState.parse(error_code);

    if (!(lastErrorState == errorState)) {
      RCLCPP_INFO(
        rclcpp::get_logger("iRC_ROS"), "Module 0x%02x: Errorcode: %s ", can_id_,
        error_code.to_string().c_str());
    }

    lastErrorState = errorState;

    // Resetting and no Errors? -> Reset is done
    // NOTE: DIOs dont have MNE errors so this differs to the Joint code
    if (!errorState.any()) {
      resetState = ResetState::reset;

      // DIOs dont report an Enabled acknowledgement, therefore no errors->enabled
      motorState = MotorState::enabled;
    }

    // Flags + DIN
    std::bitset<8> flags_din = message.data[7];

    // The highest bit is not valid
    digital_in_[7] = 0;  // flags_din[7]

    digital_in_[6] = flags_din[6];
    digital_in_[5] = flags_din[5];
    digital_in_[4] = flags_din[4];
    digital_in_[3] = flags_din[3];
    digital_in_[2] = flags_din[2];
    digital_in_[1] = flags_din[1];
    digital_in_[0] = flags_din[0];

    RCLCPP_DEBUG(
      rclcpp::get_logger("iRC_ROS"), "Module 0x%02x: digital_in: %s", can_id_,
      digital_in_.to_string().c_str());

    // Workaround for ros2_control limiting the interface data to doubles.
    for (int i = 0; i < std::min(digital_in_.size(), digital_in_double_.size()); i++) {
      digital_in_double_[i] = static_cast<double>(digital_in_[i]);
    }
  }
}

/**
 * @brief The output function. Writes the digital_out values to the digital outputs. While DIO
 * modules do not have motors attached, the communication is still done via the position command.
 */
void DIO::position_cmd()
{
  prepare_movement();

  // Even though DIO modules dont have a motor attached we still need to send
  // some data.
  int32_t set_pos_tics = 0;

  // TODO: Remove this conversion once a better way of using bitsets as in/out of
  // StateInterface/CommandInterface is found (Issues #9 #76)
  // digital_out_double_ -> digital_out_
  for (int i = 0; i < std::min(digital_out_double_.size(), digital_out_.size()); i++) {
    digital_out_[i] = digital_out_double_[i] > 0.5;
  }

  RCLCPP_DEBUG(
    rclcpp::get_logger("iRC_ROS"), "Module 0x%02x: digital_out: %s", can_id_,
    digital_out_.to_string().c_str());

  CAN::CanMessage message(can_id_, cprcan::position_msg(set_pos_tics, msg_counter_, digital_out_));

  can_interface_->write_message(message);

  msg_counter_ = (++msg_counter_) % 256;
}

/**
 * @brief Only calls the output function.
 */
void DIO::write_can()
{
  RCLCPP_DEBUG(rclcpp::get_logger("iRC_ROS"), "Module 0x%02x: Creating can message", can_id_);

  position_cmd();
}

}  // namespace irc_hardware