// Based on the TruPhysics ROS project with permission to use
// https://bitbucket.org/truphysics/igus_rebel/src/master/

#include "irc_ros_hardware/CRI/cri_messages.hpp"

#include <map>

#include "irc_ros_hardware/CRI/cri_keywords.hpp"
#include "rclcpp/rclcpp.hpp"

#define FLOAT_PRINT_PRECISION 3

namespace irc_hardware
{
namespace cri_messages
{
MessageType CriMessage::GetMessageType(const std::string & msg)
{
  std::string::size_type typeStart = msg.find(" ") + 1;
  std::string::size_type typeEnd = msg.find(" ", typeStart);

  std::string typeString = msg.substr(typeStart, typeEnd - typeStart);

  static const std::map<std::string, MessageType> message_type_map = {
    {cri_keywords::TYPE_STATUS, MessageType::STATUS},
    {cri_keywords::TYPE_OPINFO, MessageType::OPINFO},
    {cri_keywords::TYPE_GSIG, MessageType::GSIG},
    {cri_keywords::TYPE_GRIPPERSTATE, MessageType::GRIPPERSTATE},
    {cri_keywords::TYPE_RUNSTATE, MessageType::RUNSTATE},
    {cri_keywords::TYPE_MESSAGE, MessageType::MESSAGE},
    {cri_keywords::TYPE_CMD, MessageType::CMD},
    {cri_keywords::TYPE_CONFIG, MessageType::CONFIG},
    {cri_keywords::TYPE_INFO, MessageType::INFO},
    {cri_keywords::TYPE_CMDACK, MessageType::CMDACK},
    {cri_keywords::TYPE_EXECACK, MessageType::EXECACK},
    {cri_keywords::TYPE_EXECPAUSE, MessageType::EXECPAUSE},
    {cri_keywords::TYPE_EXECEND, MessageType::EXECEND},
    {cri_keywords::TYPE_EXECPAUSE, MessageType::EXECPAUSE},
    {cri_keywords::TYPE_CMDERROR, MessageType::CMDERROR}
  };

  auto it = message_type_map.find(typeString);

  if (it != message_type_map.end()) {
    return message_type_map.at(typeString);
  }

  RCLCPP_ERROR(
    rclcpp::get_logger("iRC_ROS::CRI"), "Unknown message type: \"%s\"", typeString.c_str());
  return MessageType::UNKNOWN;
}

std::string CriMessage::ParseMessageString(
  const std::string & statusString, const std::string::size_type & begin,
  const std::string::size_type & end, const std::string::size_type & keywordLength)
{
  int start = begin + keywordLength + 1;
  int length = end - start - 1;

  return statusString.substr(start, length);
}

template <class T, std::size_t N>
void CriMessage::FillArray(std::array<T, N> & array, const std::string & spaceSeparatedValues)
{
  typename std::array<T, N>::size_type idx = 0;
  std::string::size_type begin = 0;
  std::string::size_type end = spaceSeparatedValues.find(" ", begin + 1);

  while (end != std::string::npos) {
    if (idx >= N) {
      RCLCPP_ERROR(rclcpp::get_logger("iRC_ROS::CRI"), "Parsing error");
      return;
    }

    std::string value = spaceSeparatedValues.substr(begin, end - begin);

    if (std::is_same<float, T>::value) {
      array.at(idx) = std::stof(value);
    }

    if (std::is_same<int, T>::value) {
      array.at(idx) = std::stoi(value);
    }

    begin = spaceSeparatedValues.find(" ", end);
    end = spaceSeparatedValues.find(" ", begin + 1);
    idx++;
  }

  std::string value = spaceSeparatedValues.substr(begin);

  if (std::is_same<float, T>::value) {
    array.at(idx) = std::stof(value);
  }

  if (std::is_same<int, T>::value) {
    array.at(idx) = std::stoi(value);
  }
}

template <class T>
void CriMessage::FillVector(std::vector<T> & vector, const std::string & spaceSeparatedValues)
{
  std::string::size_type begin = 0;
  std::string::size_type end = spaceSeparatedValues.find(" ", begin + 1);

  while (end != std::string::npos) {
    std::string value = spaceSeparatedValues.substr(begin, end - begin);

    if (std::is_same<float, T>::value) {
      vector.push_back(std::stof(value));
    }

    if (std::is_same<int, T>::value) {
      vector.push_back(std::stoi(value));
    }

    begin = spaceSeparatedValues.find(" ", end);
    end = spaceSeparatedValues.find(" ", begin + 1);
  }

  std::string value = spaceSeparatedValues.substr(begin);

  if (std::is_same<float, T>::value) {
    vector.push_back(std::stof(value));
  }

  if (std::is_same<int, T>::value) {
    vector.push_back(std::stoi(value));
  }
}

template <class T, std::size_t N>
std::string CriMessage::ArrayToString(std::array<T, N> & array)
{
  std::ostringstream msg;
  msg << std::showpoint;
  msg << std::fixed;
  msg << std::setprecision(FLOAT_PRINT_PRECISION);

  for (T val : array) {
    msg << val << " ";
  }

  return msg.str();
}

template <class T>
std::string CriMessage::VectorToString(std::vector<T> & vector)
{
  std::ostringstream msg;
  msg << std::showpoint;
  msg << std::fixed;
  msg << std::setprecision(FLOAT_PRINT_PRECISION);

  for (T val : vector) {
    msg << val << " ";
  }

  return msg.str();
}

Status::Status(const std::string & messageString) : CriMessage(MessageType::STATUS)
{
  // RCLCPP_INFO(rclcpp::get_logger("iRC_ROS::CRI"), "%s", messageString.c_str());

  std::string::size_type modeStart = messageString.find(cri_keywords::STATUS_MODE);
  std::string::size_type posJointSetPointStart =
    messageString.find(cri_keywords::STATUS_POSJOINTSETPOINT);
  std::string::size_type posJointCurrentStart =
    messageString.find(cri_keywords::STATUS_POSJOINTCURRENT);
  std::string::size_type posCartRobotStart = messageString.find(cri_keywords::STATUS_POSCARTROBOT);
  std::string::size_type posCartPlattformStart =
    messageString.find(cri_keywords::STATUS_POSCARTPLATTFORM);
  std::string::size_type overrideValueStart = messageString.find(cri_keywords::STATUS_OVERRIDE);
  std::string::size_type dinStart = messageString.find(cri_keywords::STATUS_DIN);
  std::string::size_type doutStart = messageString.find(cri_keywords::STATUS_DOUT);
  std::string::size_type eStopStart = messageString.find(cri_keywords::STATUS_ESTOP);
  std::string::size_type supplyStart = messageString.find(cri_keywords::STATUS_SUPPLY);
  std::string::size_type currentallStart = messageString.find(cri_keywords::STATUS_CURRENTALL);
  std::string::size_type currentjointsStart =
    messageString.find(cri_keywords::STATUS_CURRENTJOINTS);
  std::string::size_type errorStart = messageString.find(cri_keywords::STATUS_ERROR);
  std::string::size_type kinstateStart = messageString.find(cri_keywords::STATUS_KINSTATE);

  if (!((modeStart < posJointSetPointStart) && (posJointSetPointStart < posJointCurrentStart) &&
        (posJointCurrentStart < posCartRobotStart) && (posCartRobotStart < posCartPlattformStart) &&
        (posCartPlattformStart < overrideValueStart) && (overrideValueStart < dinStart) &&
        (dinStart < doutStart) && (doutStart < eStopStart) && (eStopStart < supplyStart) &&
        (supplyStart < currentallStart) && (currentallStart < currentjointsStart) &&
        (currentjointsStart < errorStart) && (errorStart < kinstateStart))) {
    RCLCPP_ERROR(
      rclcpp::get_logger("iRC_ROS::CRI"), "Bad parsing error for message \"%s\"",
      messageString.c_str());
    return;
  }

  std::string modeString = ParseMessageString(
    messageString, modeStart, posJointSetPointStart, cri_keywords::STATUS_MODE.size());
  std::string posJointSetPointString = ParseMessageString(
    messageString, posJointSetPointStart, posJointCurrentStart,
    cri_keywords::STATUS_POSJOINTSETPOINT.size());
  std::string posJointCurrentString = ParseMessageString(
    messageString, posJointCurrentStart, posCartRobotStart,
    cri_keywords::STATUS_POSJOINTCURRENT.size());
  std::string posCartRobotString = ParseMessageString(
    messageString, posCartRobotStart, posCartPlattformStart,
    cri_keywords::STATUS_POSCARTROBOT.size());
  std::string posCartPlattformString = ParseMessageString(
    messageString, posCartPlattformStart, overrideValueStart,
    cri_keywords::STATUS_POSCARTPLATTFORM.size());
  std::string overrideValueString = ParseMessageString(
    messageString, overrideValueStart, dinStart, cri_keywords::STATUS_OVERRIDE.size());
  std::string dinString =
    ParseMessageString(messageString, dinStart, doutStart, cri_keywords::STATUS_DIN.size());
  std::string doutString =
    ParseMessageString(messageString, doutStart, eStopStart, cri_keywords::STATUS_DOUT.size());
  std::string eStopString =
    ParseMessageString(messageString, eStopStart, supplyStart, cri_keywords::STATUS_ESTOP.size());
  std::string supplyString = ParseMessageString(
    messageString, supplyStart, currentallStart, cri_keywords::STATUS_SUPPLY.size());
  std::string currentallString = ParseMessageString(
    messageString, currentallStart, currentjointsStart, cri_keywords::STATUS_CURRENTALL.size());
  std::string currentjointsString = ParseMessageString(
    messageString, currentjointsStart, errorStart, cri_keywords::STATUS_CURRENTJOINTS.size());
  std::string errorString =
    ParseMessageString(messageString, errorStart, kinstateStart, cri_keywords::STATUS_ERROR.size());
  std::string kinstateString = ParseMessageString(
    messageString, kinstateStart, messageString.size() + 1, cri_keywords::STATUS_KINSTATE.size());

  std::string::size_type errorSummaryEnd = errorString.find(" ");
  errorSummary = errorString.substr(0, errorSummaryEnd);
  std::string errorJointsString = errorString.substr(errorSummaryEnd + 1);

  mode = GetMode(modeString);
  FillArray(posJointSetPoint, posJointSetPointString);
  FillArray(posJointCurrent, posJointCurrentString);
  FillArray(posCartRobot, posCartRobotString);
  FillArray(posCartPlattform, posCartPlattformString);
  overrideValue = std::stof(overrideValueString);
  digital_in = std::stoi(dinString);    // TODO: Process further to actual meaning
  digital_out = std::stoi(doutString);  // TODO: Process further to actual meaning
  eStop = std::stoi(eStopString);       // TODO: Process further to actual meaning
  supply = std::stoi(supplyString);
  currentall = std::stoi(currentallString);
  FillArray(currentjoints, currentjointsString);
  // errorSummary already set above.
  FillArray(errorJoints, errorJointsString);  // TODO: Process further to actual meaning
  kinstate = GetKinstate(kinstateString);
}

Status::Status() : CriMessage(MessageType::STATUS)
{
  mode = Mode::UNKNOWN;
  posJointSetPoint.fill(0.0f);
  posJointCurrent.fill(0.0f);
  posCartRobot.fill(0.0f);
  posCartPlattform.fill(0.0f);
  overrideValue = 0.0f;
  digital_in = 0;
  digital_out = 0;
  eStop = 0;
  supply = 0;
  currentall = 0;
  currentjoints.fill(0);
  errorSummary = "NotInitialized";
  errorJoints.fill(0);
  kinstate = Kinstate::UNKNOWN;
}

void Status::Print()
{
  std::ostringstream msg;
  msg << std::showpoint << std::fixed << std::setprecision(FLOAT_PRINT_PRECISION)
      << "            mode: " << static_cast<int>(mode) << std::endl
      << "posJointSetPoint: " << ArrayToString(posJointSetPoint) << std::endl
      << " posJointCurrent: " << ArrayToString(posJointCurrent) << std::endl
      << "    posCartRobot: " << ArrayToString(posCartRobot) << std::endl
      << "posCartPlattform: " << ArrayToString(posCartPlattform) << std::endl
      << "   overrideValue: " << overrideValue << std::endl
      << "      digital_in: " << digital_in << std::endl
      << "     digital_out: " << digital_out << std::endl
      << "           eStop: " << eStop << std::endl
      << "          supply: " << supply << std::endl
      << "      currentall: " << currentall << std::endl
      << "   currentjoints: " << ArrayToString(currentjoints) << std::endl
      << "    errorSummary: " << errorSummary << std::endl
      << "     errorJoints: " << ArrayToString(errorJoints) << std::endl
      << "        kinstate: " << static_cast<int>(kinstate) << std::endl
      << std::endl;

  std::cout << msg.str();
}

/** 
 * @brief Returns the current mode. As of now only joint is used
 * TODO: Throw warning if its not the correct mode
 * and switch it?
 */
Mode Status::GetMode(const std::string & modeString)
{
  RCLCPP_DEBUG(rclcpp::get_logger("iRC_ROS::CRI"), "Converting Mode %s", modeString.c_str());

  static const std::map<std::string, Mode> mode_map = {
    {"joint", Mode::JOINT},       {"cartbase", Mode::CARTBASE}, {"carttool", Mode::CARTTOOL},
    {"platform", Mode::PLATFORM}, {"fsm", Mode::FSM},
  };

  auto it = mode_map.find(modeString);

  if (it != mode_map.end()) {
    return mode_map.at(modeString);
  }

  RCLCPP_ERROR(
    rclcpp::get_logger("iRC_ROS::CRI"), "Unknown robot mode during parsing: %s",
    modeString.c_str());
  return Mode::UNKNOWN;
}

Kinstate Status::GetKinstate(const std::string & kinstateString)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("iRC_ROS::CRI"), "Converting Kinstate %s", kinstateString.c_str());
  int kinstateInt = std::stoi(kinstateString);

  // TODO: How to handle unknown states?
  auto kinstate = static_cast<Kinstate>(kinstateInt);

  return kinstate;
}

Message::Message(const std::string & messageString) : CriMessage(MessageType::MESSAGE)
{
  std::string::size_type messageStart =
    messageString.find(cri_keywords::TYPE_MESSAGE) + cri_keywords::TYPE_MESSAGE.size() + 1;
  message = messageString.substr(messageStart);
}

Command::Command(const std::string & messageString) : CriMessage(MessageType::CMD)
{
  std::string::size_type commandStart =
    messageString.find(cri_keywords::TYPE_CMD) + cri_keywords::TYPE_CMD.size() + 1;
  command = messageString.substr(commandStart);
}

Info::Info(const std::string & messageString) : CriMessage(MessageType::INFO)
{
  std::string::size_type infoStart =
    messageString.find(cri_keywords::TYPE_INFO) + cri_keywords::TYPE_INFO.size() + 1;
  info = messageString.substr(infoStart);
}

ConfigType Config::GetConfigType(const std::string & msg)
{
  std::string::size_type typeStart = msg.find(" ") + 1;
  std::string::size_type typeEnd = msg.find(" ", typeStart);

  std::string typeString = msg.substr(typeStart, typeEnd - typeStart);

  if (typeString != cri_keywords::TYPE_CONFIG) {
    RCLCPP_ERROR(
      rclcpp::get_logger("iRC_ROS::CRI"),
      "Got message to parse for config data that was not a config message.");
    return ConfigType::UNKNOWN;
  }

  std::string::size_type configTypeStart = typeEnd + 1;
  std::string::size_type configTypeEnd = msg.find(" ", configTypeStart + 1);

  std::string configTypeString = msg.substr(configTypeStart, configTypeEnd - configTypeStart);

  if (configTypeString == cri_keywords::CONFIG_GETKINEMATICLIMITS_ANSWER) {
    return ConfigType::KINEMATICLIMITS;
  }

  RCLCPP_ERROR(
    rclcpp::get_logger("iRC_ROS::CRI"), "Unknown config type: \"%s\"", configTypeString.c_str());
  return ConfigType::UNKNOWN;
}

KinematicLimits::KinematicLimits(const std::string & messageString)
: Config(ConfigType::KINEMATICLIMITS)
{
  std::string::size_type answerStart =
    messageString.find(cri_keywords::CONFIG_GETKINEMATICLIMITS_ANSWER);
  answerStart += cri_keywords::CONFIG_GETKINEMATICLIMITS_ANSWER.size() + 1;

  std::vector<float> minMax;
  FillVector(minMax, messageString.substr(answerStart));

  if (minMax.size() % 2 != 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("iRC_ROS::CRI"), "Error parsing config message of type %d",
      (int)configType);
    return;
  }

  for (int i = 0; i < minMax.size(); i += 2) {
    minMaxPairs.push_back(std::pair<float, float>(minMax.at(i), minMax.at(i + 1)));
  }
}

std::string KinematicLimits::ToString()
{
  std::ostringstream msg;

  for (int i = 0; i < minMaxPairs.size() - 1; i++) {
    msg << "(" << minMaxPairs.at(i).first << ", " << minMaxPairs.at(i).second << ") ";
  }

  msg << "(" << minMaxPairs.at(minMaxPairs.size() - 1).first << ", "
      << minMaxPairs.at(minMaxPairs.size() - 1).second << ")";

  return msg.str();
}

void KinematicLimits::Print()
{
  RCLCPP_INFO(rclcpp::get_logger("iRC_ROS"), "Kinematic limits: %s", ToString().c_str());
}
}  // namespace cri_messages
}  // namespace irc_hardware
