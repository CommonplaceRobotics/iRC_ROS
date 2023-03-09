#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace irc_hardware
{
namespace CAN
{

using CanMessageID = uint32_t;
using t_timestamp = std::chrono::time_point<std::chrono::steady_clock>;

class CanMessage
{
public:
  std::vector<uint8_t> data;
  CanMessageID id = 0;

  /**
     * @brief Default constructor, initializes all values to 0
     */
  CanMessage() {}

  /**
     * @brief Vector constructor
     */
  CanMessage(CanMessageID id, std::vector<uint8_t> data);

  /**
     * @brief Constructor, uses std::vector for initialisation
     * @param id CAN message ID
     * @param data message data, 8 bytes max
     */
  // template<std::size_t SIZE>
  // CanMessage(CanMessageID id, std::array<uint8_t, SIZE> _data);

  /**
     * @brief Constructor, copies length values from the data array
     * @param id CAN message ID
     * @param length Message length in bytes
     * @param data message data array, must at least be length bytes
     */
  // CanMessage(CanMessageID id, uint8_t length, uint8_t* data);
  CanMessage(CanMessageID id, size_t length, uint8_t * _data);

  /**
     * @brief Constructor, copies the parameters into the message. Values are 0 if omitted.
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
  CanMessage(
    CanMessageID id, uint8_t length, uint8_t data0 = 0, uint8_t data1 = 0, uint8_t data2 = 0,
    uint8_t data3 = 0, uint8_t data4 = 0, uint8_t data5 = 0, uint8_t data6 = 0, uint8_t data7 = 0);

  /**
     * @brief Converts the CAN message to a human readable string. Data values are given in hex format.
     * @return
     */
  std::string to_string() const;
};

class TimedCanMessage : public CanMessage
{
public:
  t_timestamp timestamp;

  TimedCanMessage() {}

  TimedCanMessage(CanMessageID id, uint8_t length, uint8_t * data, t_timestamp timestamp)
  : CanMessage(id, length, data), timestamp(timestamp)
  {
  }

  TimedCanMessage(const CanMessage & message, t_timestamp timestamp)
  : CanMessage(message), timestamp(timestamp)
  {
  }
};

}  // namespace CAN
}  // namespace irc_hardware
