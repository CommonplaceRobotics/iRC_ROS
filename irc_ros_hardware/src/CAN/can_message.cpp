#include "irc_ros_hardware/CAN/can_message.hpp"

// #include <algorithm>
// #include <iomanip>
#include <array>
#include <sstream>
#include <vector>

namespace irc_hardware
{
namespace CAN
{

/**
 * @brief Constructor, copies data from data to its internal vector
 * @param id CAN message ID
 * @param _data message data vector, maximum size of 8 bytes
 */
CanMessage::CanMessage(CanMessageID id, std::vector<uint8_t> data) : id(id), data(data)
{
  if (data.size() > 8) {
    throw std::invalid_argument("Invalid CAN message length");
  }
}

/**
 * @brief Constructor, copies values from the C style data array. This
 * constructor should not be used by the user, only called in can_socket.cpp
 * @param id CAN message ID
 * @param length Message length in bytes
 * @param _data message data array, must at least be length bytes
 */
CanMessage::CanMessage(CanMessageID id, size_t length, uint8_t * _data) : id(id)
{
  if (length > 8) {
    throw std::invalid_argument("Invalid CAN message length");
  }

  for (size_t i = 0; i < length; i++) {
    data.push_back(_data[i]);
  }
}

/**
 * @brief DEPRECATED! Constructor, copies the parameters into the message. Values are 0 if omitted.
 * @param id CAN message ID
 * @param length Message length in bytes
 * @param data0 message byte 0
 * @param data1 message byte 1
 * @param data2 message byte 2
 * @param data3 message byte 3
 * @param data4 message byte 4
 * @param data5 message byte 5
 * @param data6 message byte 6
 * @param data7 message byte 7
 */
CanMessage::CanMessage(
  CanMessageID id, uint8_t length, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3,
  uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7)
: id(id)
{
  if (length > 8) {
    throw std::invalid_argument("Invalid CAN message length");
  }

  std::array<uint8_t, 8> _data = {data0, data1, data2, data3, data4, data5, data6, data7};

  for (uint8_t i : _data) {
    data.push_back(i);
  }
}

/**
 * @brief Converts the CAN message to a human readable string. .data values are given in hex format. The format is based on the candump output.
 * @return
 */
std::string CanMessage::to_string() const
{
  std::stringstream ss;
  ss << std::setfill('0') << std::setw(2) << "0x" << std::hex << id << std::dec << "  ["
     << std::setw(1) << data.size() << "] " << std::setw(2) << std::hex;
  // Implicitily cast the uint8_t to unsigned int, else it will be interpreted as an (ASCII) char
  for (unsigned int i : data) {
    ss << " " << i;
  }
  return ss.str();
}

}  // namespace CAN
}  // namespace irc_hardware