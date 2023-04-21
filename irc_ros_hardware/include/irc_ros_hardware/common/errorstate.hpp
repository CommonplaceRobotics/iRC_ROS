#pragma once

#include <bitset>
#include <string>
#include <tuple>

/**
 * @brief A class for handling module errors, both for the CPRCANv2 and CRI implementations. The
 * error descriptions are taken from the CPRCANv2 protocol description, which should also be
 * consulted for more information.
 */
class ErrorState
{
public:
  /**
   * @brief Over temperature
   * The temperature of the motor controller or the motor is above the defined
   * value in the parameters.
   */
  bool temp = true;

  /**
   * @brief Emergency stop / no voltage
   * The voltage at the motor controller is below the set limit value. This
   * may indicate a defective fuse or emergency stop.
   */
  bool estop = true;

  /**
   * @brief Motor not enabled
   * The movement of the motor is not enabled, it is not in control
   */
  bool mne = true;

  /**
   * @brief Communication Failure
   * The motor controller requires CAN messages at regular intervals. If the
   * distance between the messages is too large or the messages do not
   * arrive, the motor controller stops the movement
   */
  bool com = true;

  /**
   * @brief Following error
   * The motor controller monitors the following error, if this is greater
   * than the value set in the parameters, the motor controller stops the
   * movement.
   */
  bool lag = true;

  /**
   * @brief Encoder error
   * The motor controller has detected an encoder error. Errors can be
   * triggered by both the motor or the output encoder.
   */
  bool enc = true;

  /**
   * @brief Driver error
   * A driver error can have various causes. One possible cause is exceeding
   * the maximum speed from the parameters. With the closedloop motor
   * controllers, the error also occurs with problems with the initial rotor
   * position.
   */
  bool drv = true;

  /**
   * @brief Over current
   * The RMS current in the motor controller was above the allowed value in
   * the parameters
   */
  bool oc = true;

  ErrorState() {}

  /**
   * @brief Returns true if any error is present
   */
  bool any() const { return (temp || estop || mne || com || lag || enc || drv || oc); }

  /**
   * @brief Returns true if any error but mne is present
   */
  bool any_except_mne() const { return (temp || estop || com || lag || enc || drv || oc); }

  /**
   * @brief Returns a tuple of all errorstates to simplify comparisons
   */
  const std::tuple<bool, bool, bool, bool, bool, bool, bool, bool> make_tuple() const
  {
    return std::make_tuple(temp, estop, mne, com, lag, enc, drv, oc);
  }

  /**
   * @brief Returns true if the errors are exactly the same for both ErrorStates. Used to check if
   * a change in the error responses has occurred.
   */
  bool operator==(const ErrorState & cmp) { return this->make_tuple() == cmp.make_tuple(); }

  /**
   * @brief Sets the states according to the provided bitset
   */
  void parse(std::bitset<8> error_code)
  {
    oc = error_code[7];
    drv = error_code[6];
    enc = error_code[5];
    lag = error_code[4];
    com = error_code[3];
    mne = error_code[2];
    estop = error_code[1];
    temp = error_code[0];
  }

  /**
   * @brief Converts the ErrorState back into a 8 bit number
   */
  uint8_t to_uint8()
  {
    uint8_t out = 0;
    if (oc) out |= 10000000;
    if (drv) out |= 01000000;
    if (enc) out |= 00100000;
    if (lag) out |= 00010000;
    if (com) out |= 00001000;
    if (mne) out |= 00000100;
    if (estop) out |= 00000010;
    if (temp) out |= 00000001;

    return out;
  }

  /**
   * @brief Returns a string containing the error names present
   */
  std::string str()
  {
    std::string error_msg = "";
    if (temp) {
      error_msg += " TEMP";
    }
    if (estop) {
      error_msg += " ESTOP";
    }
    if (mne) {
      error_msg += " MNE";
    }
    if (com) {
      error_msg += " COM";
    }
    if (lag) {
      error_msg += " LAG";
    }
    if (enc) {
      error_msg += " LAG";
    }
    if (drv) {
      error_msg += " DRV";
    }
    if (oc) {
      error_msg += " OC";
    }
    return error_msg;
  }

  /**
   * @brief Returns a string containing a description of errors present
   */
  std::string str_verbose()
  {
    std::string error_msg = "";
    if (temp) {
      error_msg += " - Overtemperature";
    }
    if (estop) {
      error_msg += " - Supply to low: ESTOP might be pressed";
    }
    if (mne) {
      error_msg += " - Motor not enabled";
    }
    if (com) {
      error_msg += " - Communication watchdog";
    }
    if (lag) {
      error_msg += " - Position lag";
    }
    if (enc) {
      error_msg += " - Encoder error";
    }
    if (drv) {
      error_msg += " - Driver error";
    }
    if (oc) {
      error_msg += " - Overcurrent";
    }
    return error_msg;
  }
};