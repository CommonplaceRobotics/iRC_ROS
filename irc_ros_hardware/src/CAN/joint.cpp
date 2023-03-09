#include "irc_ros_hardware/CAN/joint.hpp"

#include <bitset>

namespace irc_hardware
{
Joint::Joint(std::string name, std::shared_ptr<CAN::CanInterface> can_interface, int can_id)
: Module::Module(name, can_interface, can_id)
{
}

Joint::~Joint()
{
  // TODO: reset_error could also disable the joint, is there an advantage of using that?
  disable_motor();
};

/**
 * @brief If the module supports a positioning ready bit then wait for it to be set.
 * TODO: Currently unused, remove once functionality is implemented elsewhere.
 */
bool Joint::is_ready_to_move()
{
  // TODO: Maybe this should go someplace else
  if (hardwareIdent == HardwareIdent::undefined) {
    RCLCPP_INFO(
      rclcpp::get_logger("iRC_ROS"),
      "Module 0x%02x: Hardware ID not set yet, sending ping. No movement until then", can_id_);
    ping();
    return false;
  }

  if (
    positioningReadyState == PositioningReadyState::ready ||
    positioningReadyState == PositioningReadyState::not_implemented) {
    return true;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("iRC_ROS"), "Module 0x%02x: Module reports that it is not ready to move yet",
    can_id_);

  return false;
}

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
  //TODO: Check that the units are correct
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
 * TODO: Unit conversion to SI unit possible?
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
 * This gets called the first time by the user.
 * After that upon receiving a response it gets called again
 * from the message callback.
 *
 * TODO: This is untested
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
  } else if (setToZeroState == SetToZeroState::zeroing_step3) {
    CAN::CanMessage message(can_id_, cprcan::set_pos_to_zero);
    can_interface_->write_message(message);

    setToZeroState = SetToZeroState::zeroing_step4;
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("iRC_ROS"),
      "Module 0x%02x: setPositionToZero called while process is already running", can_id_);
  }
}

/**
 * @brief References the position of the axis. The igus rebel joints are equipped with
 * absolute encoders and it is not necessary as it is done on startup already. For DIN-RAIL
 * modules it is required to be done on startup.
 * 
 * This gets called the first time by the user.
 * After that upon receiving a response it gets called again
 * from the message callback.
 */
void Joint::referencing()
{
  if (referenceState == ReferenceState::unreferenced) {
    CAN::CanMessage message(can_id_, cprcan::referencing);
    can_interface_->write_message(message);

    referenceState = ReferenceState::referencing_step1;
  } else if (referenceState == ReferenceState::referencing_step2) {
    CAN::CanMessage message(can_id_, cprcan::referencing);
    can_interface_->write_message(message);

    referenceState = ReferenceState::referencing_step3;
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("iRC_ROS"),
      "Module 0x%02x: referencing called while process is already running", can_id_);
  }
}

/**
 * @brief For closed loop. Aligns the motor position and references the joints via absolute encoders.
 * This is done during powering up automatically and is not needed to be called manually under normal
 * circumstances.
 * 
 * This gets called the first time by the user.
 * After that upon receiving a response it gets called again
 * from the message callback.
 *
 * TODO: This is untested.
*/
void Joint::rotor_alignment()
{
  if (rotorAlignmentState == RotorAlignmentState::unaligned) {
    CAN::CanMessage message(can_id_, cprcan::rotor_alignment);
    can_interface_->write_message(message);

    rotorAlignmentState = RotorAlignmentState::aligning_step1;
  } else if (rotorAlignmentState == RotorAlignmentState::aligning_step2) {
    rotorAlignmentState = RotorAlignmentState::aligning_step3;
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("iRC_ROS"),
      "Module 0x%02x: rotorAlignment called while process is already running", can_id_);
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

    // motorState = enabled but error -> motorState is outdated, set to disabled
    // NOTE: COM error should not occur anymore since us reading the message means the keep
    // alive signal is already being sent.
    if (motorState == MotorState::enabled && errorState.any()) {
      RCLCPP_INFO(
        rclcpp::get_logger("iRC_ROS"),
        "Module 0x%02x: Errors%s detected, motorstate set to disabled", can_id_,
        errorState.str().c_str());

      // As errors occurred the motor controller will have disabled the motors automcatically
      // As such no disable_motor() call is required.
      motorState = MotorState::disabled;
    }

    // Resetting and only MNE error? -> Reset is done
    // TODO: Seems that MNE errors don't occur here, so any() instead of any_except_mne() is used
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
    // TODO: Ringbuffer or similar structure for a moving average filter?
    std::chrono::duration<double> diff = message.timestamp - last_stamp_;
    double delay = diff.count();

    // Ignore wraparounds and div by 0
    if (delay > 0.0) {
      vel = (new_pos - pos_) / delay;

      // TODO: Velocity is of by factor 10, why?
      vel *= 10.0;
    }

    // Write the new position and save the last timestamp
    pos_ = new_pos;
    last_stamp_ = message.timestamp;

    // Current
    // TODO: Current is only available on some modules/protocol versions
    // TODO: Add current to hw_interface
    int16_t current = (message.data[5] << 8) + message.data[6];

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
      // referencing a 0 in this status bit means we lost the reference
      referenceState = ReferenceState::unreferenced;
    }

    if (aligened) {
      rotorAlignmentState = RotorAlignmentState::aligned;
    } else if (rotorAlignmentState == RotorAlignmentState::aligned && !aligened) {
      // Same principle as for referenced above
      referenceState = ReferenceState::unreferenced;
    }

    // TODO: Add SetToZero handling here
  }

  //control cmd received
  if (can_interface_->get_next_message(can_id_ + 2, message)) {
    RCLCPP_DEBUG(rclcpp::get_logger("iRC_ROS"), "Module 0x%02x: Control cmd received", can_id_);

    if (message.data == cprcan::reset_error_response) {
      RCLCPP_INFO(
        rclcpp::get_logger("iRC_ROS"), "Module 0x%02x: Error reset acknowledged", can_id_);

      // Dont set it to finished quite yet, wait for the error bit to be 0;
      // resetState = ResetState::reset;
    } else if (message.data == cprcan::enable_motor_response) {
      if (motorState != MotorState::enabled) {
        RCLCPP_INFO(rclcpp::get_logger("iRC_ROS"), "Module 0x%02x: Motor enabled", can_id_);
        motorState = MotorState::enabled;
      }
    } else if (message.data == cprcan::disable_motor_response) {
      if (motorState != MotorState::disabled) {
        RCLCPP_INFO(rclcpp::get_logger("iRC_ROS"), "Module 0x%02x: Motor disabled", can_id_);
        motorState = MotorState::disabled;
      }
    } else if (cprcan::data_has_header(message.data, cprcan::encoder_msg_header)) {
      // Output enc pos
      encoder_pos_ = (message.data[4] << 24) + (message.data[5] << 16) + (message.data[6] << 8) +
                     message.data[7];
      encoder_pos_ *= 100;
    } else if (cprcan::data_has_header(message.data, cprcan::startup_msg_header)) {
      // Startup message, can also be triggered by a ping message
      RCLCPP_INFO(
        rclcpp::get_logger("iRC_ROS"), "Module 0x%02x: Startup message received", can_id_);

      uint8_t hwid = message.data[5];

      if (hardware_id_map.count(hwid) == 0) {
        hardwareIdent = HardwareIdent::unknown;
        RCLCPP_INFO(
          rclcpp::get_logger("iRC_ROS"), "Module 0x%02x: Unknown Hardware Ident 0x%02x", can_id_,
          hwid);
      } else {
        hardwareIdent = hardware_id_map.at(hwid);
        RCLCPP_INFO(
          rclcpp::get_logger("iRC_ROS"), "Module 0x%02x: Hardware Ident 0x%02x", can_id_, hwid);
      }

      version_[0] = message.data[6];
      version_[1] = message.data[7];
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

    } else if (cprcan::data_has_header(message.data, cprcan::extended_error_msg_header)) {
      // Extended error message
      // This message does not get handled since the contents are not
      // written in the protocol document. Please use ModuleControl
      // to evaluate any unexplainable errors.
    }
  }
  if (can_interface_->get_next_message(can_id_ + 3, message)) {
    if (cprcan::data_has_header(message.data, cprcan::environmental_msg_header)) {
      // Environmental parameters
      supply_voltage_ = (message.data[2] << 8) + message.data[3];

      // The motor temperature sensor is not installed by default, which results in a reading of
      // several hundred degrees celsius.
      temperature_motor_ = temperature_scale_ * ((message.data[4] << 8) + message.data[5]);
      temperature_board_ = temperature_scale_ * ((message.data[6] << 8) + message.data[7]);

      RCLCPP_DEBUG(
        rclcpp::get_logger("iRC_ROS"),
        "Module 0x%02x: Environmental parameters: %i mV, Motor %.1lf °C, Board %.1lf °C", can_id_,
        supply_voltage_, temperature_motor_, temperature_board_);
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
    if (motorState != MotorState::enabled) {
      // Since the motor won't move yet and pos might not been read yet, send 0.0 as the goal.
      set_pos_ = 0.0;
      position_cmd();
    }

    // Reset set_pos_ so this is not taken as an input from the hardware
    set_pos_ = std::numeric_limits<double>::quiet_NaN();
  } else {
    // Command mode chosen, try move
    if (
      motorState == MotorState::enabled &&
      positioningReadyState != PositioningReadyState::not_ready) {
      // Ready to move

      // Temporary workaround for not being able reference without the periodic signal
      // TODO: Solve this better
      if (referenceState == ReferenceState::unreferenced) {
        referencing();
      }

      if (commandMode == CommandMode::position && !std::isnan(set_pos_)) {
        position_cmd();
      } else if (commandMode == CommandMode::velocity && !std::isnan(set_vel_)) {
        velocity_cmd();
      } else if (commandMode == CommandMode::torque && !std::isnan(set_torque_)) {
        torque_cmd();
      } else {
        RCLCPP_ERROR(
          rclcpp::get_logger("iRC_ROS"),
          "Module 0x%02x: command mode set but no joint goal provided.", can_id_);
      }
    } else {
      // This should not normally happen, as the activate function should have prepared the
      // movement. If it didn't succeed the movement will lag behind a bit. Too much lag may
      // cause the motor to jerk and go into a LAG error.

      // Try to reset errors and activate the motors.
      prepare_movement();

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