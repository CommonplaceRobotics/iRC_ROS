/**
 * @brief Contains all the messages defined in the CPR CAN v2 guide
 */

#pragma once

#include <algorithm>
#include <bitset>
#include <cstdint>
#include <vector>

#include "irc_ros_hardware/CAN/can_message.hpp"

namespace cprcan
{

using bytevec = const std::vector<uint8_t>;

bytevec ping = {0x01, 0xCC};

bytevec reset_error = {0x01, 0x06};
bytevec reset_error_response = {0x06, 0x00, 0x01, 0x06, 0x00, 0x01, 0x00, 0x00};

bytevec set_pos_to_zero = {0x01, 0x08, 0x00, 0x00};
bytevec set_pos_to_zero_response_1 = {0x06, 0x00, 0x01, 0x08, 0x00, 0x01, 0x00, 0x00};
bytevec set_pos_to_zero_response_2 = {0x06, 0x00, 0x02, 0x08, 0x00, 0x01, 0x00, 0x00};
bytevec set_pos_to_zero_response_3 = set_pos_to_zero_response_1;
bytevec set_pos_to_zero_response_4 = {0x06, 0x00, 0x02, 0x08, 0x00, 0x02, 0x00, 0x00};

bytevec enable_motor = {0x01, 0x09};
bytevec enable_motor_response = {0x06, 0x00, 0x01, 0x09, 0x00, 0x01, 0x00, 0x00};

bytevec disable_motor = {0x01, 0x0A};
bytevec disable_motor_response = {0x06, 0x00, 0x01, 0x0A, 0x00, 0x01, 0x00, 0x00};

bytevec referencing = {0x01, 0x0B};
bytevec referencing_response_1 = {0x06, 0x00, 0x02, 0x0B, 0x00, 0x01, 0x00, 0x00};
bytevec referencing_response_2 = {0x06, 0x00, 0x02, 0x0B, 0x00, 0x02, 0x00, 0x00};
bytevec referencing_response_error = {0x07, 0x00, 0x02, 0x0B, 0x00, 0x98, 0x00, 0x00};

bytevec rotor_alignment = {0x01, 0x0C};
bytevec rotor_alignment_response_1 = {0x06, 0x00, 0x02, 0x0C, 0x00, 0x01, 0x00, 0x00};
bytevec rotor_alignment_response_2 = {0x06, 0x00, 0x02, 0x0C, 0x00, 0x02, 0x00, 0x00};

// Message headers, only compare the start of the message
bytevec encoder_msg_header = cprcan::bytevec{0xEF, 0x00, 0x00, 0x7E};

// While to CPR_CAN_Protocol_V2_UserGuide specifies a startup message as
// {0x01, 0x02, 0x03, 0x04, 0x00}, it is not the only possible one
// {0x01, 0x02, 0x03, 0x04, 0x03} and for CAN DIO Modules {0x01, 0x02, 0x06, 0x00, 0x00}
// are sent. Thus we only compare the first two bytes.
bytevec startup_msg_header = {0x01, 0x02};

// This message is not parsed, as the extended error codes are not found inside the
// public protocol specifications.
bytevec extended_error_msg_header = {0xEF, 0x00, 0x00, 0x7E};

bytevec environmental_msg_header = {0x12, 0x00};

static bool data_has_header(std::vector<uint8_t> data, bytevec header)
{
  return std::equal(header.begin(), header.end(), data.begin());
}

static bytevec position_msg(int32_t pos, uint8_t message_counter, std::bitset<8> digital_out)
{
  bytevec cmd = {
    0x14,                           // [0] Position cmd
    0x00,                           // [1] unused
    (uint8_t)((pos >> 24) & 0xFF),  // [2-5] position in [Tics]
    (uint8_t)((pos >> 16) & 0xFF),
    (uint8_t)((pos >> 8) & 0xFF),
    (uint8_t)(pos & 0xFF),
    message_counter,                 // [6] Incrementing counter/ "Timestamp"
    (uint8_t)digital_out.to_ulong()  // [7] Digital output array
  };

  return cmd;
}

static bytevec velocity_msg(int16_t vel, uint8_t message_counter)
{
  bytevec cmd = {
    0x25,                          // [0] Velocity cmd
    (uint8_t)((vel >> 8) & 0xFF),  // [1-2] Velocity in [rpm]
    (uint8_t)(vel & 0xFF),
    message_counter,  // [3] Counter
  };

  return cmd;
}

/**
 * UNTESTED
 */
static bytevec torque_msg(int16_t torque, uint8_t message_counter)
{
  torque = std::clamp(torque, (const int16_t)-1024, (const int16_t)1024);

  bytevec cmd = {
    0x16,                             // [0] Torque cmd
    (uint8_t)((torque >> 8) & 0xFF),  // [1-2] Torque between [-1024, 1024] in "engineering units", fraction of the maximal possible torque.
    (uint8_t)(torque & 0xFF),
    message_counter,  // [3] Incrementing counter/ "Timestamp"
  };

  return cmd;
}
}  // namespace cprcan
