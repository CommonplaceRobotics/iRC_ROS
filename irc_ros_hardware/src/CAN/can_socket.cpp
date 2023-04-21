/**
 * @brief CAN interface for SocketCAN (Linux)
 */

#include "irc_ros_hardware/CAN/can_socket.hpp"

#include <errno.h>
#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <cstring>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

namespace irc_hardware
{
namespace CAN
{

CanInterfaceSocketCAN::~CanInterfaceSocketCAN() { disconnect(); }

/**
 * @brief Connect to CAN bus
 * @param can_port The port where the CAN interface is found, defaults to "can0"
 * @return true on success
 */
bool CanInterfaceSocketCAN::connect(std::string can_port)
{
  is_socket_connected_ = false;
  is_disconnect_requested_ = false;

  RCLCPP_INFO(rclcpp::get_logger("iRC_ROS::CAN"), "Connecting");

  try {
    socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    // TODO: Socket and rc further below should return -1 on error, but it appears to be just < 0, why?
    if (socket_ < 0)
      throw std::runtime_error("Could not create socket: " + std::string(strerror(errno)));

    // Register can socket number
    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, can_port.c_str(), IFNAMSIZ);
    ioctl(socket_, SIOCGIFINDEX, &ifr);

    // Register for error frames
    can_err_mask_t err_mask = CAN_ERR_MASK;
    setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask));

    // Bind can socket
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    int rc = bind(socket_, (struct sockaddr *)&addr, sizeof(addr));
    if (rc < 0)
      throw std::runtime_error("Could not bind CAN socket: " + std::string(strerror(errno)));

    // Constructs the new thread and runs it. Does not block execution.
    std::thread t1([this]() { read_thread_static(this); });
    t1.detach();

    RCLCPP_INFO(rclcpp::get_logger("iRC_ROS::CAN"), "Connected to CAN module via SocketCAN...");

    // Needs to be set before testing the write fn, as it will otherwise fail
    is_socket_connected_ = true;

    // It takes a short time for the socket to be usable
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Bus test message, fails if socket is available but not usable
    std::vector<uint8_t> data = {0x42, 0x43, 0, 0, 0, 0, 0, 0};
    if (!write_message(CanMessage(0x10, data))) {
      throw std::runtime_error("Could not test CAN socket");
    }

  } catch (std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("iRC_ROS::CAN"), "Could not connect to CAN module via SocketCAN: %s",
      e.what());
    is_socket_connected_ = false;
  }

  // TODO: Is there an advantage over return is_socket_connected_ ?
  return is_connected();
}

/**
 * @brief Disconnects from CAN bus
 * @return true on success
 */
bool CanInterfaceSocketCAN::disconnect()
{
  RCLCPP_DEBUG(rclcpp::get_logger("iRC_ROS::CAN"), "SocketCAN disconnect requested");
  is_disconnect_requested_ = true;
  if (socket_ >= 0) {
    std::this_thread::sleep_for(std::chrono::microseconds(500));
    close(socket_);
    socket_ = -1;
    RCLCPP_INFO(rclcpp::get_logger("iRC_ROS::CAN"), "CAN interface SocketCAN disconnected.");
  }

  return !is_socket_connected_;
}

void CanInterfaceSocketCAN::parse_errorframe(can_frame & frame)
{
  union {
    uint16_t can_id;
    struct
    {
      uint16_t CAN_TX_TIMEOUT : 1; /* TX timeout (by netdevice driver) */
      uint16_t CAN_LOSTARB : 1;    /* lost arbitration    / data[0]    */
      uint16_t CAN_CRTL : 1;       /* controller problems / data[1]    */
      uint16_t CAN_PROT : 1;       /* protocol violations / data[2..3] */
      uint16_t CAN_TRX : 1;        /* transceiver status  / data[4]    */
      uint16_t CAN_ACK : 1;        /* received no ACK on transmission */
      uint16_t CAN_BUSOFF : 1;     /* bus off */
      uint16_t CAN_BUSERROR : 1;   /* bus error (may flood!) */
      uint16_t CAN_RESTARTED : 1;  /* bus error (may flood!) */
    };
  } error_code;

  uint32_t error_can_id = frame.can_id & CAN_ERR_MASK;
  error_code.can_id = frame.can_id;
  if (error_code.CAN_TX_TIMEOUT) {
    RCLCPP_ERROR(rclcpp::get_logger("iRC_ROS::CAN"), "SocketCAN ERROR: CAN_TX_TIMEOUT");
  }
  if (error_code.CAN_LOSTARB) {
    RCLCPP_ERROR(rclcpp::get_logger("iRC_ROS::CAN"), "SocketCAN ERROR: CAN_LOSTARB");
  }
  if (error_code.CAN_CRTL) {
    if (frame.data[1] & CAN_ERR_CRTL_UNSPEC) /* unspecified */
      RCLCPP_ERROR(
        rclcpp::get_logger("iRC_ROS::CAN"),
        "SocketCAN ERROR: controller problems, unspecified error "
        "(rx errors %d, tx errors %d)",
        frame.data[7], frame.data[6]);
    if (frame.data[1] & CAN_ERR_CRTL_RX_OVERFLOW) /* RX buffer overflow */
      RCLCPP_ERROR(
        rclcpp::get_logger("iRC_ROS::CAN"),
        "SocketCAN ERROR: controller problems, RX buffer overflow "
        "(rx errors %d, tx errors %d)",
        frame.data[7], frame.data[6]);
    if (frame.data[1] & CAN_ERR_CRTL_TX_OVERFLOW) /* TX buffer overflow */
      RCLCPP_ERROR(
        rclcpp::get_logger("iRC_ROS::CAN"),
        "SocketCAN ERROR: controller problems, TX buffer overflow "
        "(rx errors %d, tx errors %d)",
        frame.data[7], frame.data[6]);
    if (frame.data[1] & CAN_ERR_CRTL_RX_WARNING) /* reached warning level for RX errors */
      RCLCPP_ERROR(
        rclcpp::get_logger("iRC_ROS::CAN"),
        "SocketCAN ERROR: controller problems, reached warning "
        "level for RX errors (rx errors %d, tx errors %d)",
        frame.data[7], frame.data[6]);
    if (frame.data[1] & CAN_ERR_CRTL_TX_WARNING) /* reached warning level for TX errors */
      RCLCPP_ERROR(
        rclcpp::get_logger("iRC_ROS::CAN"),
        "SocketCAN ERROR: controller problems, reached warning "
        "level for TX errors (rx errors %d, tx errors %d)",
        frame.data[7], frame.data[6]);
    if (frame.data[1] & CAN_ERR_CRTL_RX_PASSIVE) /* reached error passive status RX */
      RCLCPP_ERROR(
        rclcpp::get_logger("iRC_ROS::CAN"),
        "SocketCAN ERROR: controller problems,  reached error "
        "passive status RX (rx errors %d, tx errors %d)",
        frame.data[7], frame.data[6]);
    if (frame.data[1] & CAN_ERR_CRTL_TX_PASSIVE) /* reached error passive status TX */
      RCLCPP_ERROR(
        rclcpp::get_logger("iRC_ROS::CAN"),
        "SocketCAN ERROR: controller problems,  reached error "
        "passive status TX  (rx errors %d, tx errors %d)",
        frame.data[7], frame.data[6]);
  }
  if (error_code.CAN_PROT) {
    RCLCPP_ERROR(rclcpp::get_logger("iRC_ROS::CAN"), "SocketCAN ERROR: CAN_PROT");
  }
  if (error_code.CAN_TRX) {
    RCLCPP_ERROR(rclcpp::get_logger("iRC_ROS::CAN"), "SocketCAN ERROR: CAN_TRX");
  }
  if (error_code.CAN_ACK) {
    RCLCPP_ERROR(rclcpp::get_logger("iRC_ROS::CAN"), "SocketCAN ERROR: CAN_ACK");
  }
  if (error_code.CAN_BUSOFF) {
    RCLCPP_ERROR(rclcpp::get_logger("iRC_ROS::CAN"), "SocketCAN ERROR: CAN_BUSOFF");
  }
  if (error_code.CAN_BUSERROR) {
    RCLCPP_ERROR(rclcpp::get_logger("iRC_ROS::CAN"), "SocketCAN ERROR: CAN_BUSERROR");
  }
  if (error_code.CAN_RESTARTED) {
    RCLCPP_ERROR(rclcpp::get_logger("iRC_ROS::CAN"), "SocketCAN ERROR: CAN_RESTARTED");
  }
}
/**
 * @brief Reads CAN messages, stops when IsThreadRunning is set to false
 */
void CanInterfaceSocketCAN::read_thread()
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("iRC_ROS::CAN"), "SocketCAN thread started(Socket==%d). %s", socket_,
    is_disconnect_requested_ ? "Disconnect is already requested" : "");

  // struct for poll()
  struct pollfd fds[200];
  memset(fds, 0, sizeof(fds));
  fds[0].fd = socket_;
  fds[0].events = POLLIN;

  while (!is_disconnect_requested_ && socket_ >= 0) {
    // poll instead of select:
    // https://www.ibm.com/docs/en/i/7.3?topic=designs-using-poll-instead-select
    int rc = poll(fds, 1, 0);
    if (rc == 1) {
      struct can_frame frame;
      memset(&frame, 0, sizeof(frame));
      frame.can_id = 0;
      frame.len = 0;

      ssize_t bytes_read = read(socket_, &frame, sizeof(frame));

      if (frame.can_id & (1 << CAN_EFF_ID_BITS)) {
        // Error frame received
        parse_errorframe(frame);
        // TODO: Any error handling?
      } else if (bytes_read > 0) {
        if (frame.len > 0) {
          std::ostringstream msg;
          msg << std::hex << std::setfill('0') << std::setw(2) << "0x"
              << static_cast<int>(frame.can_id) << " [" << std::setw(1)
              << static_cast<int>(frame.len) << "] ";

          for (int i = 0; i < frame.len; i++) {
            msg << std::setw(2) << static_cast<int>(frame.data[i]) << " ";
          }

          RCLCPP_DEBUG(rclcpp::get_logger("iRC_ROS::CAN"), "Received: %s", msg.str().c_str());

          auto timestamp = std::chrono::steady_clock::now();

          consume_message(TimedCanMessage(frame.can_id, frame.len, frame.data, timestamp));
        }
      } else if (bytes_read == 0) {
        RCLCPP_DEBUG(rclcpp::get_logger("iRC_ROS::CAN"), "SocketCAN read: EOF");
      } else {
        int err = errno;
        RCLCPP_ERROR(
          rclcpp::get_logger("iRC_ROS::CAN"), "SocketCAN read: failed to read: %s", strerror(err));

        auto timestamp = std::chrono::steady_clock::now();
        add_can_dump_entry(
          TimedCanMessage(frame.can_id, frame.len, frame.data, timestamp), true, err);
      }
    }
  }
  RCLCPP_DEBUG(
    rclcpp::get_logger("iRC_ROS::CAN"), "SocketCAN thread exiting %s (Socket==%d)",
    is_disconnect_requested_ ? "because disconnect is requested" : "", socket_);
  if (socket_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("iRC_ROS::CAN"), "SocketCAN thread exiting");
  }
  is_socket_connected_ = false;
}

/**
 * @brief Static method to call read_thread via std::thread
 * @param context pointer to this
 */
void CanInterfaceSocketCAN::read_thread_static(void * context)
{
  static_cast<CanInterfaceSocketCAN *>(context)->read_thread();
}

/**
 * @brief Sends a CAN message if connected
 * @param message CAN message data
 */
bool CanInterfaceSocketCAN::write_message(const CanMessage & message)
{
  if (!is_socket_connected_ || socket_ < 0) {
    RCLCPP_WARN(
      rclcpp::get_logger("iRC_ROS::CAN"), "Sending CAN message failed, CAN not connected yet?");

    return false;
  }

  struct can_frame frame;
  memset(&frame, 0, sizeof(frame));
  std::copy(message.data.begin(), message.data.end(), frame.data);
  frame.can_id = (canid_t)message.id;
  frame.len = message.data.size();

  RCLCPP_DEBUG(rclcpp::get_logger("iRC_ROS::CAN"), "Sending CAN message");

  std::ostringstream msg;
  msg << std::hex << std::setfill('0') << std::setw(2) << "0x" << static_cast<int>(frame.can_id)
      << " [" << std::setw(1) << static_cast<int>(frame.len) << "] ";

  for (int i = 0; i < frame.len; i++) {
    msg << std::setw(2) << static_cast<int>(frame.data[i]) << " ";
  }

  RCLCPP_DEBUG(rclcpp::get_logger("iRC_ROS::CAN"), "Sending: %s", msg.str().c_str());

  int bytes_sent = write(socket_, &frame, sizeof(frame));
  if (bytes_sent < 0) {  // Transmission error
    // Dont spam the log with repeated errors
    int err = errno;
    if (err != last_send_error_) {
      last_send_error_count_ = 0;
      last_send_error_ = err;
    }

    if (last_send_error_count_ == 0) {
      RCLCPP_ERROR(
        rclcpp::get_logger("iRC_ROS::CAN"), "Failed to write CAN message '%s', reason %s",
        message.to_string().c_str(), strerror(err));
      return false;
    } else if (
      last_send_error_count_ == 10 || last_send_error_count_ == 25 ||
      last_send_error_count_ % 50 == 0) {
      RCLCPP_ERROR(
        rclcpp::get_logger("iRC_ROS::CAN"),
        "Failed to write CAN message '%s', reason %s, repeated %d times",
        message.to_string().c_str(), strerror(err), last_send_error_count_);

      disconnect();
    }

    last_send_error_count_++;

    auto timestamp = std::chrono::steady_clock::now();
    add_can_dump_entry(TimedCanMessage(message, timestamp), false, err);

    return false;
  } else {  // sent successfully
    RCLCPP_DEBUG(rclcpp::get_logger("iRC_ROS::CAN"), "CAN message sent successfully");
    last_send_error_ = 0;

    auto timestamp = std::chrono::steady_clock::now();
    add_can_dump_entry(TimedCanMessage(message, timestamp), false);
  }

  return true;
}

/**
 * @brief Translates the interface dependent error code to a human readable string
 * @param error_code interface dependent error code
 * @return human readable string or raw error code
 */
std::string CanInterfaceSocketCAN::translate_can_dump_error(int error_code) const
{
  return strerror(error_code);
}

}  // namespace CAN
}  // namespace irc_hardware