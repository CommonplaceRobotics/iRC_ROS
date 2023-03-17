/**
 * @brief Abstract CanInterface class. This handles all system-independent tasks of the interface
 * including message distribution (message buffer, callback, message dump).
 */

#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

#include "can_message.hpp"

namespace irc_hardware
{
namespace CAN
{

/**
 * @brief Abstract CAN interface class with all hardware independent methods
 */
class CanInterface
{
public:
  using Ptr = std::shared_ptr<CanInterface>;
  /**
    * @brief Connect to CAN bus
    * @return true on success
    */
  virtual bool connect(std::string can_port) = 0;
  /**
    * @brief Disconnects from CAN bus
    * @return true on success
    */
  virtual bool disconnect() = 0;

  /**
    * @brief Returns the connection state
    * @return true if connected
    */
  virtual bool is_connected() const = 0;

  /**
    * @brief Sends a CAN message if connected
    * @param message CAN message data
    * @return true if sent successfully
    */
  virtual bool write_message(const CanMessage & message) = 0;
  /**
    * @brief Retrieves the CAN message received with the given ID
    * @param id CAN ID to retrieve
    * @param message CAN message data
    * @return true if a message was found
    */
  bool get_next_message(CanMessageID id, TimedCanMessage & message);  // const;

  /**
    * @brief Write the CAN dump to the logger
    */
  void write_can_dump();
  /**
    * @brief Sets the maximum size of the CAN dump
    * @param size Maximum number of messages in the CAN dump. Set to 0 to disable the CAN dump.
    */
  void set_can_dump_size(size_t size);

  /**
    * @brief Sets the function to be called when a message is received
    * @param ctx context pointer
    * @param func function to be called
    */
  void set_message_received_callback(std::function<void(const TimedCanMessage &)> func);

  /**
    * @brief Sets the CAN bridge function to be called when a message is received.
    * This function should return true to prevent further processing
    * @param ctx context pointer
    * @param func function to be called
    */
  void set_can_bridge_callback(std::function<bool(const TimedCanMessage &)> func);

protected:
  /**
    * @brief Puts a received CAN message into the buffer, the dump and publishes it via callback.
    * Derived classes should not do these things on their own but call this method.
    * @param message
    */
  void consume_message(const TimedCanMessage & message);
  /**
    * @brief Adds a message to the CAN dump
    * @param message CAN message
    * @param is_incoming true if the message was incoming, false if it was outgoing
    * @param error_code Interface dependent error code
    */
  void add_can_dump_entry(const TimedCanMessage & message, bool is_incoming, int error_code = 0);
  /**
    * @brief Clears the message dump
    */
  void clear_message_dump();
  /**
    * @brief Clears the message buffer
    */
  void clear_message_buffer_();

  /**
    * @brief Translates the interface dependent error code to a human readable string
    * @param error_code interface dependent error code
    * @return human readable string or raw error code
    */
  virtual std::string translate_can_dump_error(int error_code) const;

private:
  std::mutex map_mutex_;
  size_t warn_message_queue_size_ = 5;
  size_t max_message_queue_size_ = 7;

  /**
    * @brief Buffer of the last incoming messages of each CAN ID
    */
  std::unordered_map<CanMessageID, std::queue<TimedCanMessage>> message_buffer_;

  /**
    * @brief Adds a received message to the message buffer
    * @param message received message
    */
  void add_message_to_buffer(const TimedCanMessage & message);

  /**
    * @brief CAN dump entry data
    */
  struct CanDumpEntry
  {
    TimedCanMessage message;
    int error_code = 0;
    bool is_incoming = false;
  };

  /**
    * @brief CAN dump ring buffer of incoming and outgoing messages. This is intended for logging messages if an error occurred.
    */
  std::vector<CanDumpEntry> can_dump_;
  /**
    * @brief Current position of the ring buffer
    */
  size_t can_dump_index_ = 0;
  /**
    * @brief Number of messages in the ring buffer that have not been written to the log yet
    */
  size_t can_dump_filled_ = 0;
  /**
    * @brief Protective mutex for the CAN dump
    */
  std::mutex can_dump_mutex_;

  /**
    * @brief Function to be called when a message is received
    */
  std::function<void(const TimedCanMessage &)> message_received_callback;
  /**
    * @brief Function to be called when a message is received, if this returns true further callbacks are not called
    */
  std::function<bool(const TimedCanMessage &)> can_bridge_callback;
};

}  // namespace CAN
}  // namespace irc_hardware
