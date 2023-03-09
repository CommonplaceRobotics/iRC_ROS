/**
 * @brief CAN interface for SocketCAN (Linux)
 */

#pragma once

#include <linux/can.h>

#include <string>

#include "can_interface.hpp"

namespace irc_hardware
{
namespace CAN
{

/**
 * @brief CAN access via SocketCAN (Linux)
 */
class CanInterfaceSocketCAN : public CanInterface
{
public:
  ~CanInterfaceSocketCAN();

  /**
   * @brief Connect to CAN bus
   * @return true on success
   */
  bool connect(std::string can_port = "can0");

  /**
   * @brief Disconnects from CAN bus
   * @return true on success
   */
  bool disconnect();

  /**
   * @brief Returns the connection state
   * @return true if connected
   */
  bool is_connected() const { return is_socket_connected_ && socket_ >= 0; }

  /**
   * @brief Sends a CAN message if connected
   * @param message CAN message data
   * @return true if sent successfully
   */
  bool write_message(const CanMessage & message);

  /**
   * @brief Translates the interface dependent error code to a human readable string
   * @param error_code interface dependent error code
   * @return human readable string or raw error code
   */
  virtual std::string translate_can_dump_error(int error_code) const;

private:
  /**
   * @brief SocketCAN socket number
   */
  int socket_ = -1;

  /**
   * @brief true if connected, set by read_thread()
   */
  bool is_socket_connected_ = false;

  /**
   * @brief set to true to stop the read thread and disconnect
   */
  bool is_disconnect_requested_ = false;

  /**
   * @brief Last error code from sending a message
   */
  int last_send_error_ = 0;

  /**
   * @brief Count of last error code
   */
  int last_send_error_count_ = 0;

  void parse_errorframe(can_frame & frame);

  /**
     * @brief Reads CAN messages, stops when is_disconnect_requested_ is set to true
     */
  void read_thread();

  /**
     * @brief Static method to call read_thread via std::thread
     * @param context pointer to this
     */
  static void read_thread_static(void * context);
};

}  // namespace CAN
}  // namespace irc_hardware
