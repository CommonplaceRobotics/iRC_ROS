// Based on the TruPhysics ROS project with permission to use
// https://bitbucket.org/truphysics/igus_rebel/src/master/

#include "irc_ros_hardware/irc_ros_cri.hpp"

#include <algorithm>
#include <bitset>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "irc_ros_hardware/CRI/cri_keywords.hpp"
#include "irc_ros_hardware/common/errorstate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace irc_hardware
{
IrcRosCri::IrcRosCri()
: crisocket("192.168.3.11", 3920, 200),
  cmd_counter(1),
  continueAlive(false),
  continueMessage(false),
  aliveWaitMs(50),
  currentStatus(),
  lastKinstate(cri_messages::Kinstate::NO_ERROR),
  kinstateMessage("")
{
}

IrcRosCri::~IrcRosCri() {}

/**
 * @brief This sends the keep alive heartbeat to the CRI controller. The message always
 * contains a jog, under normal position command operation it should be left at 0. 
 */
void IrcRosCri::AliveThreadFunction()
{
  RCLCPP_DEBUG(rclcpp::get_logger("iRC_ROS"), "Starting to send ALIVEJOG");

  while (continueAlive) {
    std::ostringstream msg;
    msg << std::showpoint;
    msg << std::fixed;
    msg << std::setprecision(8);
    msg << "CRISTART " << get_cmd_counter() << " ";
    msg << "ALIVEJOG ";

    for (auto j : jog_array) {
      msg << j << " ";
    }

    // We have to write 9 values in every case so fill the message in
    // case less joints are specified
    for (int i = jog_array.size(); i < 9; i++) {
      msg << 0.0f << " ";
    }

    msg << "CRIEND" << std::endl;

    // This seems unnecessary, if any weird communication behaviour occurs
    // then it might have been there for a reason. Else remove this in a
    // future release
    //{
    // std::lock_guard<std::mutex> lockGuard(aliveLock);

    crisocket.SendMessage(msg.str());
    //}

    std::this_thread::sleep_for(std::chrono::milliseconds(aliveWaitMs));
  }

  RCLCPP_WARN(rclcpp::get_logger("iRC_ROS"), "Stopped to send ALIVEJOG");
}

void IrcRosCri::MessageThreadFunction()
{
  RCLCPP_DEBUG(rclcpp::get_logger("iRC_ROS"), "Starting to process robot messages");

  while (continueMessage) {
    if (crisocket.HasMessage()) {
      std::string msg = crisocket.GetMessage();

      //RCLCPP_INFO(rclcpp::get_logger("iRC_ROS"), "%s", msg.c_str());
      cri_messages::MessageType type = cri_messages::CriMessage::GetMessageType(msg);

      //RCLCPP_INFO(rclcpp::get_logger("iRC_ROS"), "Type: %d", type);

      switch (type) {
        case cri_messages::MessageType::STATUS: {
          cri_messages::Status status = cri_messages::Status(msg);
          // RCLCPP_INFO(rclcpp::get_logger("iRC_ROS"), "SET: %f", status.posJointSetPoint.front());
          // RCLCPP_INFO(rclcpp::get_logger("iRC_ROS"), "IS:  %f", status.posJointCurrent.front());

          currentStatus = status;
          ProcessStatus(currentStatus);
          break;
        }

        case cri_messages::MessageType::MESSAGE: {
          cri_messages::Message message = cri_messages::Message(msg);
          // Not sure if the ROS node should display these?
          //RCLCPP_INFO(rclcpp::get_logger("iRC_ROS"), "MESSAGE: %s", message.message.c_str());
          break;
        }

        case cri_messages::MessageType::CMD: {
          cri_messages::Command command = cri_messages::Command(msg);

          // Not sure if the ROS node should display these?
          RCLCPP_INFO(rclcpp::get_logger("iRC_ROS"), "CMD: %s", command.command.c_str());
          break;
        }

        case cri_messages::MessageType::CONFIG: {
          cri_messages::ConfigType configType = cri_messages::Config::GetConfigType(msg);

          switch (configType) {
            case cri_messages::ConfigType::KINEMATICLIMITS: {
              cri_messages::KinematicLimits kinematicLimits = cri_messages::KinematicLimits(msg);
              kinematicLimits.Print();
              break;
            }
          }

          break;
        }
        case cri_messages::MessageType::INFO: {
          cri_messages::Info info = cri_messages::Info(msg);
          RCLCPP_INFO(rclcpp::get_logger("iRC_ROS"), "INFO: %s", info.info.c_str());
          break;
        }

        case cri_messages::MessageType::EXECACK: {
          RCLCPP_INFO(rclcpp::get_logger("iRC_ROS"), "EXECACK received");
          break;
        }
        case cri_messages::MessageType::CMDERROR: 
        {   
            RCLCPP_ERROR(rclcpp::get_logger("iRC_ROS"), "CMDERROR: %s", msg.c_str());
            break;
        }
      }
    }
  }

  RCLCPP_WARN(rclcpp::get_logger("iRC_ROS"), "Stopped to process robot messages");
}

/**
 * @brief The current 4-digits command counter in [1:9999]
 * 
 * From the CRI reference:
 * All messages from the server do have an incrementing sCnt as first parameter,
 * the messages from the client an independent cCnt. Both are incremented with each
 * message from 1 to 9999, then reset to 1
 */
int IrcRosCri::get_cmd_counter()
{
  std::lock_guard<std::mutex> lockGuard(counterLock);
  int current = cmd_counter;
  cmd_counter = (cmd_counter % 9999) + 1;
  return current;
}

/**
 * @brief Adds the header and footer to a command message and sends it to the controller.
 */
void IrcRosCri::Command(const std::string & command)
{
  std::ostringstream msg;
  msg << cri_keywords::START << " " << get_cmd_counter() << " ";
  msg << cri_keywords::TYPE_CMD << " ";
  msg << command << " ";
  msg << cri_keywords::END << std::endl;

  crisocket.SendMessage(msg.str());
}

/**
 * @brief Adds the header and footer to a config request message and sends it to the controller.
 */
void IrcRosCri::GetConfig(const std::string & config)
{
  std::ostringstream msg;
  msg << cri_keywords::START << " " << get_cmd_counter() << " ";
  msg << cri_keywords::TYPE_CONFIG << " ";
  msg << config << " ";
  msg << cri_keywords::END << std::endl;

  crisocket.SendMessage(msg.str());
}

/**
 * @brief This is the main read method. It parses the information sent from the controller
 * and adjusts the internal state as such.
 */
void IrcRosCri::ProcessStatus(const cri_messages::Status & status)
{
  cri_messages::Kinstate currentKinstate = status.kinstate;
  std::array<int, 16> currentErrorJoints = status.errorJoints;

  // TODO: Cleanup
  if (lastKinstate != currentKinstate) {
    if (lastKinstate != cri_messages::Kinstate::NO_ERROR) {
      RCLCPP_INFO(
        rclcpp::get_logger("iRC_ROS"), "Kinematics error resolved [%s]", kinstateMessage.c_str());
    }

    if (currentKinstate != cri_messages::Kinstate::NO_ERROR) {
      static const std::map<cri_messages::Kinstate, std::string> kinstate_msg_map = {
        {cri_messages::Kinstate::JOINT_LIMIT_MIN, "joint at minimum limit"},
        {cri_messages::Kinstate::JOINT_LIMIT_MAX, "joint at maximum limit"},
        {cri_messages::Kinstate::CARTESIAN_SINGULARITY_CENTER, "cartesian singularity (center)"},
        {cri_messages::Kinstate::CARTESIAN_SINGULARITY_REACH, "cartesian singularity (reach)"},
        {cri_messages::Kinstate::CARTESIAN_SINGULARITY_WRIST, "cartesian singularity (wrist)"},
        {cri_messages::Kinstate::TOOL_AT_VIRTUAL_BOX_LIMIT_1, "tool at virtual box limit 1"},
        {cri_messages::Kinstate::TOOL_AT_VIRTUAL_BOX_LIMIT_2, "tool at virtual box limit 2"},
        {cri_messages::Kinstate::TOOL_AT_VIRTUAL_BOX_LIMIT_3, "tool at virtual box limit 3"},
        {cri_messages::Kinstate::TOOL_AT_VIRTUAL_BOX_LIMIT_4, "tool at virtual box limit 4"},
        {cri_messages::Kinstate::TOOL_AT_VIRTUAL_BOX_LIMIT_5, "tool at virtual box limit 5"},
        {cri_messages::Kinstate::TOOL_AT_VIRTUAL_BOX_LIMIT_6, "tool at virtual box limit 6"},
        {cri_messages::Kinstate::MOTION_NOT_ALLOWED, "motion not allowed"},
        {cri_messages::Kinstate::UNKNOWN, "unknown error"},
      };

      auto it = kinstate_msg_map.find(currentKinstate);

      if (it != kinstate_msg_map.end()) {
        kinstateMessage = kinstate_msg_map.at(currentKinstate);
      }

      RCLCPP_ERROR(rclcpp::get_logger("iRC_ROS"), "Kinematics error [%s]", kinstateMessage.c_str());
    }
  }

  if (currentErrorJoints != lastErrorJoints) {
    // loop through the 6 joint errors
    ErrorState errorState;

    for (unsigned int i = 0; i < 6; i++) {
      int errorJoint = currentErrorJoints.at(i);
      if (errorJoint != lastErrorJoints.at(i)) {
        errorState.parse(errorJoint);

        if (errorState.any()) {
          RCLCPP_ERROR(
            rclcpp::get_logger("iRC_ROS"), "Joint %i Error: [%s]", i,
            errorState.str_verbose().c_str());
        } else {
          RCLCPP_INFO(rclcpp::get_logger("iRC_ROS"), "Joint %i Error: Cleaned", i);
        }
      }
    }
  }

  lastKinstate = currentKinstate;
  lastErrorJoints = currentErrorJoints;
}

/**
 * @brief Currently unused method that requests info about the state of
 * the referencing.
 */
void IrcRosCri::GetReferencingInfo() { Command(std::string("GetReferencingInfo")); }

/**
  *  @brief Sends the positions from the set_pos vector to the CRI controller.
  */
void IrcRosCri::CmdMove()
{
  std::ostringstream msg;
  msg << "Move"
      << " "
      << "Joint"
      << " ";

  // Limit the precision to one digit behind the decimal point
  msg << std::fixed << std::setprecision(1);

  // Add the joint goals as degrees
  for (int i = 0; i < set_pos_.size(); i++) {
    msg << (set_pos_[i] * 180 / M_PI) + pos_offset_[i] << " ";
  }

  // We always have to send 9 values so we need to fill the message if we use less
  for (int i = set_pos_.size(); i < 9; i++) {
    msg << 0.0f << " ";
  }

  msg << move_velocity;

  Command(msg.str());
}

/**
 ******************************
 * ROS2 Control functionality *
 ******************************
 */

hardware_interface::CallbackReturn IrcRosCri::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Overwrite the default IP address in the urdf
  if (info_.hardware_parameters.count("ip") > 0) {
    std::string ip = info_.hardware_parameters.at("ip");
    crisocket.SetIp(ip);
  }

  for (const hardware_interface::ComponentInfo & joint : info.joints) {
    // TODO: Checks

    jog_array.push_back(0.0f);
    pos_.push_back(0.0f);
    vel_.push_back(0.0f);
    set_pos_.push_back(0.0f);
    set_pos_last_.push_back(0.0f);
    set_vel_.push_back(0.0f);

    // Read the joint offset from the ros2_control configuration
    double cri_joint_offset = 0.0;
    if (joint.parameters.count("cri_joint_offset") > 0) {
      cri_joint_offset = stod(joint.parameters.at("cri_joint_offset"));
    } else {
      RCLCPP_WARN(
        rclcpp::get_logger("iRC_ROS"),
        "No cri_joint_offset specified for joint %s, using default value of %lf", joint.name,
        cri_joint_offset);
    }
    pos_offset_.push_back(cri_joint_offset);
  }

  for (const hardware_interface::ComponentInfo & gpio : info.gpios) {
    digital_in_double_.push_back(0.0f);
    digital_out_double_.push_back(0.0f);
  }
  if (jog_array.size() > 9) {
    RCLCPP_ERROR(
      rclcpp::get_logger("iRC_ROS"),
      "Detected %lu joints in the urdf file, but only a maximum of %i is supported",
      jog_array.size(), CRI_MAX_N_JOINTS);
    return ::hardware_interface::CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("iRC_ROS"), "Detected %lu joints in the urdf file", jog_array.size());
  return hardware_interface::CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn IrcRosCri::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  continueMessage = true;
  messageThread = std::thread(&IrcRosCri::MessageThreadFunction, this);

  crisocket.Start();

  // The following delay does not appear to be necessary, re-add if problems occur
  // std::this_thread::sleep_for(std::chrono::milliseconds(500));

  Command(cri_keywords::COMMAND_CONNECT);
  Command(cri_keywords::COMMAND_RESET);
  Command(cri_keywords::COMMAND_ENABLE);

  continueAlive = true;
  aliveThread = std::thread(&IrcRosCri::AliveThreadFunction, this);

  GetConfig(cri_keywords::CONFIG_GETKINEMATICLIMITS);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IrcRosCri::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  std::fill(jog_array.begin(), jog_array.end(), 0.0f);

  std::this_thread::sleep_for(std::chrono::milliseconds(aliveWaitMs + 10));

  Command(cri_keywords::COMMAND_DISABLE);
  Command(cri_keywords::COMMAND_DISCONNECT);

  continueAlive = false;

  if (aliveThread.joinable()) {
    aliveThread.join();
  }

  crisocket.Stop();

  continueMessage = false;

  if (messageThread.joinable()) {
    messageThread.join();
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> IrcRosCri::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  //Add the position state
  for (int i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pos_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_[i]));
  }
  // TODO: DIO specific state_interfaces
  int counter = 0;
  for (auto && gpio : info_.gpios) {
    for (auto && si : gpio.state_interfaces) {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(gpio.name, si.name, &digital_in_double_[counter]));
      counter++;
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> IrcRosCri::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (int i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &set_pos_[i]));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &set_vel_[i]));
  }

  // DIO specific state_interfaces
  int counter = 0;
  for (auto && gpio : info_.gpios) {
    //   // Support multiple command interfaces here, but the modules dont support that yet
    for (auto && ci : gpio.command_interfaces) {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(gpio.name, ci.name, &digital_out_double_[counter]));
      counter++;
    }
  }

  return command_interfaces;
}

/**
 * @brief Reads the current joint positions and converts them to radians for ROS2
 */
hardware_interface::return_type IrcRosCri::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  std::vector<double> temp_pos = pos_;

  std::copy(
    currentStatus.posJointCurrent.begin(), currentStatus.posJointCurrent.begin() + pos_.size(),
    temp_pos.begin());

  // degrees to radians and apply offset
  for (size_t i = 0; i < pos_.size(); i++) {
    temp_pos[i] = temp_pos[i] + pos_offset_[i];
    pos_[i] = temp_pos[i] * M_PI / 180.0;
  }

  return hardware_interface::return_type::OK;
}

/**
 * @brief
 * If we did not set a velocity we want the jog set to 0.0f and instead send a position command
 * If neither velocity nor position have been send yet do no movement and only send jog messages
 * with the velocity set to 0.0f
 */
hardware_interface::return_type IrcRosCri::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Make it possible to use different movement commands after another.
  // If more than one set_... var is not NaN reset them all and wait for ros2 control to send a new goal.
  if (
    std::none_of(set_pos_.begin(), set_pos_.end(), [](double d) { return std::isnan(d); }) &&
    std::none_of(set_vel_.begin(), set_vel_.end(), [](double d) { return std::isnan(d); })) {
    std::fill(set_pos_.begin(), set_pos_.end(), std::numeric_limits<double>::quiet_NaN());
    std::fill(set_vel_.begin(), set_vel_.end(), std::numeric_limits<double>::quiet_NaN());
  }

  // Position command
  if (std::none_of(set_pos_.begin(), set_pos_.end(), [](double d) { return std::isnan(d); })) {
    // Only send a message if the command changed. Since the CRI controller can't handle new
    // position goals while still moving we only want to send the goal position without any
    // interpolation.
    //
    // TODO: Find ros2_controller which only sends the final position once or wait for a protocol
    // update for CRI (Issue #72)
    if (set_pos_ != set_pos_last_) {
      CmdMove();
      set_pos_last_ = set_pos_;
    }
  }
  // Velocity command
  else if (std::none_of(set_vel_.begin(), set_vel_.end(), [](double d) { return std::isnan(d); })) {
    // Simply use the velocities in the jog message
    std::copy(set_vel_.begin(), set_vel_.end(), jog_array.begin());
  }
  // No movement
  else {
    std::fill(jog_array.begin(), jog_array.end(), 0.0f);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace irc_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(irc_hardware::IrcRosCri, hardware_interface::SystemInterface)
