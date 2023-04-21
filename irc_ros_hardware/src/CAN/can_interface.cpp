#include "irc_ros_hardware/CAN/can_interface.hpp"

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <unordered_map>

namespace irc_hardware
{
namespace CAN
{

/**
 * @brief Retrieves the CAN message received with the given ID
 * @param id CAN ID to retrieve
 * @param message CAN message data
 * @return true if a message was found
 */
bool CanInterface::get_next_message(CanMessageID id, TimedCanMessage & message)  // const
{
  const std::lock_guard<std::mutex> lock(map_mutex_);

  auto it = message_buffer_.find(id);
  if (it == message_buffer_.end()) {
    return false;

  } else {
    if (message_buffer_[id].size() > 0) {
      message = message_buffer_[id].front();
      message_buffer_[id].pop();
      return true;
    }

    return false;
  }
}

/**
 * @brief Write the CAN dump to the logger
 */
void CanInterface::write_can_dump()
{
  if (!can_dump_.empty() && can_dump_filled_ > 0) {
    // Copy the data
    std::unique_lock<std::mutex> lock(can_dump_mutex_);
    std::vector<CanDumpEntry> temp_dump = can_dump_;
    size_t current_index = can_dump_index_;
    int filled = can_dump_filled_;
    can_dump_filled_ = 0;
    lock.unlock();

    // Dont write duplicate entries
    int skipCnt = temp_dump.size() - filled;

    // Write messages
    for (size_t i = skipCnt; i < temp_dump.size(); i++) {
      // sort ringbuffer
      size_t idx = current_index + i;
      if (idx >= temp_dump.size()) idx -= temp_dump.size();

      std::stringstream ss;

      // Direction
      if (temp_dump[idx].is_incoming)
        ss << "I --> ";
      else
        ss << "O <-- ";

      // Entry nummer
      ss << std::setw(3) << (i - skipCnt) << " ";

      // Time
      auto ts = temp_dump[idx].message.timestamp;
      double ts_float =
        std::chrono::duration_cast<std::chrono::duration<double>>(
          std::chrono::time_point_cast<std::chrono::milliseconds>(ts).time_since_epoch())
          .count();
      ss << std::setfill('0') << std::setw(9) << std::fixed << std::setprecision(6) << ts_float;

      // CAN message
      ss << " ID: 0x" << std::hex << std::setw(2) << temp_dump[idx].message.id
         << " LEN: " << std::dec << std::setw(1)
         << static_cast<int>(temp_dump[idx].message.data.size()) << " MSG:" << std::hex
         << std::setfill('0');
      for (int j = 0; j < 8 && j < temp_dump[idx].message.data.size(); j++) {
        ss << " " << std::setw(2) << static_cast<int>(temp_dump[idx].message.data[j]);
      }

      if (temp_dump[idx].error_code)
        ss << " ERROR: " << translate_can_dump_error(temp_dump[idx].error_code);

      RCLCPP_INFO(rclcpp::get_logger("iRC_ROS::CAN"), ss.str().c_str());
    }
  }
}

/**
 * @brief Translates the interface dependent error code to a human readable string
 * @param error_code interface dependent error code
 * @return human readable string or raw error code
 */
std::string CanInterface::translate_can_dump_error(int error_code) const
{
  return std::to_string(error_code);
}

/**
 * @brief Clears the message dump
 */
void CanInterface::clear_message_dump()
{
  std::lock_guard<std::mutex> lock(can_dump_mutex_);
  can_dump_index_ = can_dump_filled_ = 0;
}

/**
 * @brief Clears the message buffer
 */
void CanInterface::clear_message_buffer_()
{
  if (!is_connected()) {
    message_buffer_.clear();
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("iRC_ROS::CAN"), "Can not clear CAN message buffer while connected");
  }
}

/**
 * @brief Sets the maximum size of the CAN dump
 * @param size Maximum number of messages in the CAN dump. Set to 0 to disable the CAN dump.
 */
void CanInterface::set_can_dump_size(size_t size)
{
  if (can_dump_.size() != size) {
    RCLCPP_INFO(rclcpp::get_logger("iRC_ROS::CAN"), "Setting CAN dump size to %lu messages", size);
    std::lock_guard<std::mutex> lock(can_dump_mutex_);
    can_dump_.resize(size);
    can_dump_index_ = can_dump_filled_ = 0;
  }
}

/**
 * @brief Adds a message to the CAN dump
 * @param message CAN message
 * @param is_incoming true if the message was incoming, false if it was outgoing
 * @param error_code Interface dependent error code
 */
void CanInterface::add_can_dump_entry(
  const TimedCanMessage & message, bool is_incoming, int error_code)
{
  if (!can_dump_.empty()) {
    CanDumpEntry entry;
    entry.message = message;
    entry.is_incoming = is_incoming;
    entry.error_code = error_code;

    // Get an index, no conflict may occur else the log is worthless
    std::unique_lock<std::mutex> lock(can_dump_mutex_);
    size_t index = can_dump_index_++;
    size_t size = can_dump_.size();
    if (can_dump_index_ >= size) can_dump_index_ = 0;
    if (can_dump_filled_ < size) can_dump_filled_++;
    lock.unlock();

    can_dump_[index] = entry;
  }
}

/**
 * @brief Adds a received message to the message buffer
 * @param message received message
 */
void CanInterface::add_message_to_buffer(const TimedCanMessage & message)
{
  const std::lock_guard<std::mutex> lock(map_mutex_);
  message_buffer_[message.id].push(message);
  if (message_buffer_[message.id].size() > max_message_queue_size_) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("iRC_ROS::CAN"),
      "CAN ids 0x%02x message buffer has grown over max allowed size, dropping oldest message",
      message.id);
    message_buffer_[message.id].pop();
  } else if (message_buffer_[message.id].size() > warn_message_queue_size_) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("iRC_ROS::CAN"),
      "CAN ids 0x%02x message buffer is growing more than expected", message.id);
  }
}

/**
 * @brief Puts a received CAN message into the buffer, the dump and publishes it via callback.
 * Derived classes should not do these things on their own but call this method.
 * @param message
 */
void CanInterface::consume_message(const TimedCanMessage & message)
{
  add_message_to_buffer(message);
  add_can_dump_entry(message, false);
  if (can_bridge_callback) {
    if (can_bridge_callback(message)) return;
  }
  if (message_received_callback) {
    message_received_callback(message);
  }
}

/**
 * @brief Sets the function to be called when a message is received
 * @param ctx context pointer
 * @param func function to be called
 */
void CanInterface::set_message_received_callback(std::function<void(const TimedCanMessage &)> func)
{
  message_received_callback = func;
}

/**
 * @brief Sets the CAN bridge function to be called when a message is received.
 * This function should return true to prevent further processing
 * @param ctx context pointer
 * @param func function to be called
 */
void CanInterface::set_can_bridge_callback(std::function<bool(const TimedCanMessage &)> func)
{
  can_bridge_callback = func;
}

}  // namespace CAN
}  // namespace irc_hardware
