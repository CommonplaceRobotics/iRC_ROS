#include "irc_ros_hardware/CAN/joint.hpp"

#include <bitset>

namespace irc_hardware
{
Joint::Joint(std::string name, std::shared_ptr<CAN::CanInterface> can_interface, int can_id)
: Module::Module(name, can_interface, can_id)
{
  // Calculate the weighted moving average weights
  // https://en.wikiversity.org/wiki/Moving_Average/Weighted
  double divider = 0.0;
  for (int i = 0; i < velocity_buffer_size_; i++) {
    divider += i + 1;
  }

  for (int i = 0; i < velocity_buffer_size_; i++) {
    velocity_buffer_weights_[i] = (i + 1) / divider;

    // Also fill the deque until it reaches its max size. Otherwise the queue size would need to
    // be handled dynamically. 0.0 is a reasonable assumption for the velocity as the motor should
    // be at standstill when initialising.
    velocity_buffer_.push_front(0.0);
  }
}

Joint::~Joint()
{
  // Both reset_errors() and disable_motor() can be used for stopping the motion. The advantage of
  // disable_motors it that it preserves the errors in case further investigation, e.g. via module
  // control, is desired.
  disable_motor();
};

/**
 * @brief Moves the motor to the position given by set_pos_ and sets the 
 * digital Outputs at the same time. Must be called regularly, otherwise the
 * controllers watchdog resets after 50ms.
 *
 * This should be the main way of controlling your robot arm, e.g. igus ReBeL
 * as it is the most precise.
 *  
 * From the CPRCAN v2 user guide:
 * The Position CMD puts the axis into the positioning mode. This mode is
 * comparable with the CANopen operating mode IPO or CSP. The specification
 * of the position is done in Encodertics. The motor controller tries to reach
 * the target position as fast as possible, no path with target speed and
 * acceleration ramps is calculated. If the target position is too far away
 * from the current position, this can lead to unexpected behavior or a
 * shutdown.
 */
void Joint::position_cmd()
{
  // Converts the position in radians to motor tics
  int32_t set_pos_tics = set_pos_ * 180.0 / (M_PI)*tics_over_degree_;

  CAN::CanMessage message(can_id_, cprcan::position_msg(set_pos_tics, msg_counter_, digital_out_));
  can_interface_->write_message(message);

  msg_counter_ = (++msg_counter_) % 256;
}

/**
 * @brief Sends the velocity of set_vel_. Must be called regularly, otherwise the
 * controllers watchdog resets after 50ms.
 * 
 * You should use this for mobile platforms.
 */
void Joint::velocity_cmd()
{
  // TODO: Check that the units are correct
  int16_t intVel = set_vel_ * tics_over_degree_;

  CAN::CanMessage message(can_id_, cprcan::velocity_msg(intVel, msg_counter_));
  can_interface_->write_message(message);

  msg_counter_ = (++msg_counter_) % 256;
}
/**
 * @brief Sends the torque of set_torque_. Must be called regularly, otherwise the
 * controllers watchdog resets after 50ms.
 * 
 * TODO: This method is untested, so use with precaution.
 */
void Joint::torque_cmd()
{
  // Torque in message is in fraction of the maximal achievable torque.
  int16_t int_set_torque = set_torque_;

  CAN::CanMessage message(can_id_, cprcan::torque_msg(int_set_torque, msg_counter_));
  can_interface_->write_message(message);

  msg_counter_ = (++msg_counter_) % 256;
}

/**
 * @brief Sets the current position as the new 0 offset. In most cases this should not be used.
 * Can be undone by the referencing function.
 * 
 * This gets called by the user while the rest of the process is done in the callback
 * 
 * TODO: This method is untested, so use with precaution.
 */
void Joint::set_position_to_zero()
{
  RCLCPP_WARN(
    rclcpp::get_logger("iRC_ROS"),
    "Module 0x%02x: setPositionToZero may cause the robot to lose correct joint limits and crash. "
    "Use with caution!",
    can_id_);

  if (setToZeroState == SetToZeroState::not_zeroed) {
    CAN::CanMessage message(can_id_, cprcan::set_pos_to_zero);
    can_interface_->write_message(message);

    setToZeroState = SetToZeroState::zeroing_step1;
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("iRC_ROS"),
      "Module 0x%02x: setPositionToZero: Called while process is already running", can_id_);
  }
}

/**
 * @brief Gets called by the read_can function once a response message arrives.
 * @param response The message that triggered the function call
 */
void Joint::set_position_to_zero_callback(cprcan::bytevec response)
{
  if (
    setToZeroState == SetToZeroState::zeroing_step1 &&
    response == cprcan::set_pos_to_zero_response_1) {
    setToZeroState = SetToZeroState::zeroing_step2;
  } else if (
    setToZeroState == SetToZeroState::zeroing_step2 &&
    response == cprcan::set_pos_to_zero_response_2) {
    CAN::CanMessage message(can_id_, cprcan::set_pos_to_zero);
    can_interface_->write_message(message);

    setToZeroState = SetToZeroState::zeroing_step3;
  } else if (
    setToZeroState == SetToZeroState::zeroing_step3 &&
    response == cprcan::set_pos_to_zero_response_3) {
    setToZeroState = SetToZeroState::zeroing_step4;
  } else if (
    setToZeroState == SetToZeroState::zeroing_step4 &&
    response == cprcan::set_pos_to_zero_response_4) {
    setToZeroState = SetToZeroState::zeroed;
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("iRC_ROS"), "Module 0x%02x: setPositionToZero: Unexpected response",
      can_id_);
  }
}

/**
 * @brief References the position of the axis. The igus rebel joints are equipped with
 * absolute encoders and it is not necessary as it is done on startup already. For DIN-RAIL
 * modules it is required to be done on startup.
 * 
 * This gets called by the user while the rest of the process is done in the callback
 */
void Joint::referencing()
{
  if (
    referenceState == ReferenceState::unreferenced ||
    referenceState == ReferenceState::not_required) {
    RCLCPP_INFO(
      rclcpp::get_logger("iRC_ROS"), "Module 0x%02x: Referencing: Sending first ref message",
      can_id_);

    CAN::CanMessage message(can_id_, cprcan::referencing);
    can_interface_->write_message(message);

    referenceState = ReferenceState::referencing_step1;
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("iRC_ROS"),
      "Module 0x%02x: Referencing: Called while process is already running", can_id_);
  }
}

/**
 * @brief Gets called by the read_can function once a response message arrives.
 * @param response The message that triggered the function call
 */
void Joint::referencing_callback(cprcan::bytevec response)
{
  if (
    referenceState == ReferenceState::referencing_step1 &&
    response == cprcan::referencing_response_1) {
    RCLCPP_INFO(
      rclcpp::get_logger("iRC_ROS"), "Module 0x%02x: Referencing: ACK 1 received", can_id_);

    CAN::CanMessage message(can_id_, cprcan::referencing);
    can_interface_->write_message(message);

    referenceState = ReferenceState::referencing_step2;
  } else if (
    referenceState == ReferenceState::referencing_step2 &&
    response == cprcan::referencing_response_2) {
    RCLCPP_INFO(rclcpp::get_logger("iRC_ROS"), "Module 0x%02x: Referencing done", can_id_);

    referenceState = ReferenceState::referenced;
  } else if (
    referenceState == ReferenceState::referencing_step1 &&
    response == cprcan::referencing_response_1) {
    RCLCPP_WARN(
      rclcpp::get_logger("iRC_ROS"), "Module 0x%02x: Referencing: ACK 2 received too early",
      can_id_);
    // Legacy fix, some FW+module combinations send the second ACK to early and we still need to
    // send the second referencing command.
    referencing();
  } else if (response == cprcan::referencing_response_error) {
    RCLCPP_ERROR(
      rclcpp::get_logger("iRC_ROS"),
      "Module 0x%02x: Referencing: Referencing has already been started", can_id_);
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("iRC_ROS"), "Module 0x%02x: Referencing: Unexpected response received",
      can_id_);
  }
}

/**
 * @brief For closed loop. Aligns the motor position and references the joints via absolute encoders.
 * This is done during powering up automatically and is not needed to be called manually under normal
 * circumstances.
 * 
 * This gets called by the user while the rest of the process is done in the callback
 */
void Joint::rotor_alignment()
{
  if (rotorAlignmentState == RotorAlignmentState::unaligned) {
    CAN::CanMessage message(can_id_, cprcan::rotor_alignment);
    can_interface_->write_message(message);

    rotorAlignmentState = RotorAlignmentState::aligning_step1;
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("iRC_ROS"),
      "Module 0x%02x: Aligning: Called while process is already running", can_id_);
  }
}

/**
 * @brief Gets called by the read_can function once a response message arrives.
 * @param response The message that triggered the function call
 */
void Joint::rotor_alignment_callback(cprcan::bytevec response)
{
  if (
    rotorAlignmentState == RotorAlignmentState::aligning_step1 &&
    response == cprcan::rotor_alignment_response_1) {
    CAN::CanMessage message(can_id_, cprcan::rotor_alignment);
    can_interface_->write_message(message);

    rotorAlignmentState = RotorAlignmentState::aligning_step2;
  } else if (
    rotorAlignmentState == RotorAlignmentState::aligning_step2 &&
    response == cprcan::rotor_alignment_response_2) {
    rotorAlignmentState = RotorAlignmentState::aligned;
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("iRC_ROS"), "Module 0x%02x: Aligning: Unexpected response received",
      can_id_);
  }
}

/**
 * @brief Parses the standard response. May influence most of the internal states.
 */
void Joint::standard_response_callback(CAN::TimedCanMessage message)
{
  // Error
  std::bitset<8> error_code = message.data[0];
  errorState.parse(error_code);

  if (!(lastErrorState == errorState)) {
    RCLCPP_INFO(
      rclcpp::get_logger("iRC_ROS"), "Module 0x%02x: Errorcode: %s ", can_id_,
      error_code.to_string().c_str());
  }

  lastErrorState = errorState;

  // motorState = enabled but error -> motorState is outdated, set to disabled
  // NOTE: COM error should not occur anymore since us reading the message means the keep
  // alive signal is already being sent.
  if (motorState == MotorState::enabled && errorState.any()) {
    RCLCPP_INFO(
      rclcpp::get_logger("iRC_ROS"), "Module 0x%02x: Errors%s detected, motorstate set to disabled",
      can_id_, errorState.str().c_str());

    // As errors occurred the motor controller will have disabled the motors automcatically
    // As such no disable_motor() call is required, but we need to update our internal state.
    motorState = MotorState::disabled;
  }

  // Resetting and only MNE error? -> Reset is done
  // Note: Contrary to the protocol MNE errors don't occur heres so !any() instead of !any_except_mne() is used
  if (!errorState.any() && resetState != ResetState::reset) {
    resetState = ResetState::reset;
    RCLCPP_INFO(
      rclcpp::get_logger("iRC_ROS"),
      "Module 0x%02x: Reset appears to have been successful, no errors remaining", can_id_);
  } else if (errorState.any() && resetState == ResetState::reset) {
    resetState = ResetState::not_reset;
    RCLCPP_INFO(rclcpp::get_logger("iRC_ROS"), "Module 0x%02x: Reset required", can_id_);
  }

  // Position
  int32_t position =
    (message.data[1] << 24) + (message.data[2] << 16) + (message.data[3] << 8) + message.data[4];

  // Converts motor tics to radians
  double new_pos = (static_cast<double>(position) / (tics_over_degree_)) * (M_PI / 180.0);

  // Calculate the velocity
  std::chrono::duration<double> diff = message.timestamp - last_stamp_;
  double delay = diff.count();

  // Ignore wraparounds and div by 0
  if (delay > 0.0) {
    double new_vel = (new_pos - pos_) / delay;

    // TODO: Velocity is of by factor 10, why?
    new_vel *= 10.0;

    velocity_buffer_.pop_back();
    velocity_buffer_.push_front(new_vel);

    double sum = 0.0;
    for (int i = 0; i < velocity_buffer_size_; i++) {
      sum += velocity_buffer_[i] * velocity_buffer_weights_[i];
    }

    vel_ = sum;
  }

  // Write the new position and save the last timestamp
  pos_ = new_pos;
  last_stamp_ = message.timestamp;

  // Current RMS in mA, only available for closed_loop modules
  if (controllerType == ControllerType::closed_loop) {
    motor_current_ = (message.data[5] << 8) + message.data[6];
  }

  // Flags + DIN
  std::bitset<8> flags_din = message.data[7];

  bool referenced = flags_din[7];
  bool aligened = flags_din[6];
  // Bit 5 is unused

  // NOTE: Bit 6 having this functionality on closed loop controllers is not mentioned in the docs
  bool posRdyState = flags_din[4];

  // NOTE: The inputs on the motor modules are not usable for the end-user
  // We only read them to follow the protocol specifications
  digital_in_[0] = flags_din[3];
  digital_in_[1] = flags_din[2];
  digital_in_[2] = flags_din[1];
  digital_in_[3] = flags_din[0];

  if (positioningReadyState != PositioningReadyState::not_implemented) {
    positioningReadyState =
      posRdyState ? PositioningReadyState::ready : PositioningReadyState::not_ready;
  }

  if (referenced) {
    referenceState = ReferenceState::referenced;
  } else if (referenceState == ReferenceState::referenced && !referenced) {
    // If we were already referenced or not currently in the process of
    // referencing a 0 in this status bit means we lost the reference.
    // TODO: Can this happen in an actual scenario?
    RCLCPP_WARN(
      rclcpp::get_logger("iRC_ROS"),
      "Module 0x%02x: Referencing: Set to unreferenced by referenced bit", can_id_);
    // Temporary removed until made sure that this bit is set on all modules.
    // referenceState = ReferenceState::unreferenced;
  }

  if (aligened) {
    rotorAlignmentState = RotorAlignmentState::aligned;
  } else if (rotorAlignmentState == RotorAlignmentState::aligned && !aligened) {
    // TODO: Same status as the referenced bit
    // referenceState = ReferenceState::unreferenced;
  }
}

/**
 * @brief Parses the output encoder position
 */
void Joint::encoder_message_callback(cprcan::bytevec data)
{
  encoder_pos_ = 100 * ((data[4] << 24) + (data[5] << 16) + (data[6] << 8) + data[7]);
}

/**
 * @brief Parses the startup message, which can also be triggered by a ping message.
 */
void Joint::startup_message_callback(cprcan::bytevec data)
{
  RCLCPP_INFO(rclcpp::get_logger("iRC_ROS"), "Module 0x%02x: Startup message received", can_id_);

  uint8_t hwid = data[5];

  if (hardware_id_map.count(hwid) == 0) {
    hardwareIdent = HardwareIdent::unknown;
    RCLCPP_INFO(
      rclcpp::get_logger("iRC_ROS"), "Module 0x%02x: Unknown Hardware Ident 0x%02x", can_id_, hwid);
  } else {
    hardwareIdent = hardware_id_map.at(hwid);
    RCLCPP_INFO(
      rclcpp::get_logger("iRC_ROS"), "Module 0x%02x: Hardware Ident 0x%02x", can_id_, hwid);
  }

  version_[0] = data[6];
  version_[1] = data[7];
  if (version_[0] < 3) {
    RCLCPP_WARN(
      rclcpp::get_logger("iRC_ROS"),
      "Module 0x%02x: Firmware version is %02x.%02x. This version is very outdated, please "
      "update via ModuleControl",
      can_id_, version_[0], version_[1]);

    // Old versions do not support the ready to move bit
    positioningReadyState = PositioningReadyState::not_implemented;

  } else {
    RCLCPP_INFO(
      rclcpp::get_logger("iRC_ROS"), "Module 0x%02x: Firmware version is %02x.%02x", can_id_,
      version_[0], version_[1]);
  }
}

/**
 * @brief Parses the environmental message.
 */
void Joint::environmental_message_callback(cprcan::bytevec data)
{
  // Supply voltage in mV
  supply_voltage_ = (data[2] << 8) + data[3];

  // The motor temperature sensor is not installed by default, which results in a reading of
  // several hundred degrees celsius.
  temperature_motor_ = temperature_scale_ * ((data[4] << 8) + data[5]);
  temperature_board_ = temperature_scale_ * ((data[6] << 8) + data[7]);

  RCLCPP_DEBUG(
    rclcpp::get_logger("iRC_ROS"),
    "Module 0x%02x: Environmental parameters: %i mV, Motor %.1lf °C, Board %.1lf °C", can_id_,
    supply_voltage_, temperature_motor_, temperature_board_);

  // Ensure the hardware id is requested again
  // TODO: Does not really fit in the callback, but it is called about once a second if the can bus
  // is working. Still, we should find a better place for the fn.
  if (hardwareIdent == HardwareIdent::undefined) {
    RCLCPP_INFO(
      rclcpp::get_logger("iRC_ROS"),
      "Module 0x%02x: Hardware ID not set yet, sending ping to request id.", can_id_);
    ping();
  }
}

/**
 * @brief Reads the last can message for each of the can ids belonging to this module.
 */
void Joint::read_can()
{
  RCLCPP_DEBUG(rclcpp::get_logger("iRC_ROS"), "Module 0x%02x: read_can()", can_id_);

  CAN::TimedCanMessage message;

  // Standard response
  while (can_interface_->get_next_message(can_id_ + 1, message)) {
    // Requires the timestamp, thus we TimedCanMessage instead of just the payload
    standard_response_callback(message);
  }

  //control cmd received
  while (can_interface_->get_next_message(can_id_ + 2, message)) {
    if (message.data == cprcan::reset_error_response) {
      reset_error_callback(message.data);
    } else if (message.data == cprcan::set_pos_to_zero_response_1) {
      set_position_to_zero_callback(message.data);
    } else if (message.data == cprcan::set_pos_to_zero_response_2) {
      set_position_to_zero_callback(message.data);
    } else if (message.data == cprcan::set_pos_to_zero_response_3) {
      set_position_to_zero_callback(message.data);
    } else if (message.data == cprcan::set_pos_to_zero_response_4) {
      set_position_to_zero_callback(message.data);
    } else if (message.data == cprcan::enable_motor_response) {
      enable_motor_callback(message.data);
    } else if (message.data == cprcan::disable_motor_response) {
      disable_motor_callback(message.data);
    } else if (message.data == cprcan::referencing_response_1) {
      referencing_callback(message.data);
    } else if (message.data == cprcan::referencing_response_2) {
      referencing_callback(message.data);
    } else if (message.data == cprcan::referencing_response_error) {
      referencing_callback(message.data);
    } else if (message.data == cprcan::rotor_alignment_response_1) {
      rotor_alignment_callback(message.data);
    } else if (message.data == cprcan::rotor_alignment_response_2) {
      rotor_alignment_callback(message.data);
    } else if (cprcan::data_has_header(message.data, cprcan::encoder_msg_header)) {
      encoder_message_callback(message.data);
    } else if (cprcan::data_has_header(message.data, cprcan::startup_msg_header)) {
      startup_message_callback(message.data);
    } else if (cprcan::data_has_header(message.data, cprcan::extended_error_msg_header)) {
      // Extended error message
      // This message does not get handled since the contents are not
      // written in the protocol document. Please use ModuleControl
      // to evaluate any unexplainable errors.
    }
  }
  while (can_interface_->get_next_message(can_id_ + 3, message)) {
    if (cprcan::data_has_header(message.data, cprcan::environmental_msg_header)) {
      environmental_message_callback(message.data);
    }
  }
}

/**
 * @brief The output function. Sends movements to the modules over CAN
 * Until it gets input to the module from ros2_control it sends a heartbeat
 * signal with the motors disabled.
 */
void Joint::write_can()
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("iRC_ROS"),
    "Module 0x%02x: Creating can message, mode %d set_pos_: %lf, set_vel_: %lf, set_torque_: %lf",
    can_id_, static_cast<int>(commandMode), set_pos_, set_vel_, set_torque_);

  if (commandMode == CommandMode::none) {
    // Start a heartbeat position command

    if (std::isnan(pos_)) {
      set_pos_ = 0.0;
      position_cmd();

      // Reset set_pos_ so this is not taken as an input from the hardware
      set_pos_ = std::numeric_limits<double>::quiet_NaN();
    } else {
      set_pos_ = pos_;
      position_cmd();
    }
  } else {
    // Command mode chosen, try to move
    if (
      !errorState.any() && motorState == MotorState::enabled &&
      positioningReadyState != PositioningReadyState::not_ready) {
      // Ready to move

      // Thus no more automatic resets are allowed unless a command mode switch occurs
      may_reset_ = false;

      if (commandMode == CommandMode::position && !std::isnan(set_pos_)) {
        position_cmd();
      } else if (commandMode == CommandMode::velocity && !std::isnan(set_vel_)) {
        velocity_cmd();
      } else if (commandMode == CommandMode::torque && !std::isnan(set_torque_)) {
        torque_cmd();
      } else {
        RCLCPP_DEBUG(
          rclcpp::get_logger("iRC_ROS"),
          "Module 0x%02x: Command mode set but no joint goal provided.", can_id_);
      }
    } else {
      // This should not normally happen, as the activate function should have prepared the
      // movement. If it didn't succeed the movement will lag behind a bit. Too much lag may
      // cause the motor to jerk and go into a LAG error.

      // Try to reset errors and activate the motors. Only right after starting the loop
      if (may_reset_) {
        prepare_movement();
      } else {
        RCLCPP_WARN(
          rclcpp::get_logger("iRC_ROS"),
          "Module 0x%02x: Module is in error state and may not reset automatically", can_id_);
      }

      if (commandMode == CommandMode::position) {
        // If the motor is already enabled or getting enabled, sending a position of 0 might cause
        // unwanted movement. Instead the motors set_position is set as its current position.
        set_pos_ = pos_;
        position_cmd();
      } else if (commandMode == CommandMode::velocity) {
        set_vel_ = 0.0;
        velocity_cmd();
      } else if (commandMode == CommandMode::torque) {
        set_torque_ = 0.0;
        torque_cmd();
      }
    }
  }
}
}  // namespace irc_hardware