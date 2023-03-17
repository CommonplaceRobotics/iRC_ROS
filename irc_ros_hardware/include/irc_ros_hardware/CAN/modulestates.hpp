/**
 * @brief The module states for CAN specific functionality.
 * 
 * If anything from here is useful for CRI it should be copied to common/errorstates.hpp, which at
 * this point should be renamed. Some of the functionality here is only useful for motor modules,
 * not for gpio modules. Currently this is ignored and the unused states will stay at their default
 * values.
 * 
 * This file is used from module.hpp but also from the irc_ros_controllers/dashboard_controller.hpp
 * to interpret the exchanged values. Any changes here need to be adapted in both files.
 */

#pragma once

#include <cstdint>
#include <unordered_map>

namespace irc_hardware
{

// TODO: There are also ids for the open loop versions, add those here as well
enum class HardwareIdent {
  undefined,
  unknown,

  // From other code
  DINRAIL_SUPPORT_MODULE,  // Product code 2 210 011-2
  DINRAIL_OL_STEPPER,      // OL = Open loop
  REBEL_BASE,
  RASPERRY_PI_HEAD,  // EmbeddedControl, old Rebel bases, ...
  MOBILE_PLATFORM_INTERFACE_V2,
  DC_GRIPPER,  // Internal use only

  // From testing modules
  DINRAIL_DIO_MODULE,

  // From the CPRCAN V2 documentation, CL = Closed loop
  DINRAIL_CL_BLDC,
  DINRAIL_CL_STEPPER,
  ONAXIS_CL_BLDC,
  DINRAIL_CL_DC,
  BOOTLOADER_DSPIC33EP128GM706,
  BOOTLOADER_DSPIC33EP256GM706,
  BOOTLOADER_DSPIC33EP128MC506,
};

static const std::unordered_map<uint8_t, HardwareIdent> hardware_id_map = {
  {0x0A, HardwareIdent::REBEL_BASE},
  {0x0B, HardwareIdent::RASPERRY_PI_HEAD},
  {0x0D, HardwareIdent::MOBILE_PLATFORM_INTERFACE_V2},
  {0x20, HardwareIdent::DINRAIL_DIO_MODULE},
  {0x37, HardwareIdent::DINRAIL_SUPPORT_MODULE},
  {0x42, HardwareIdent::DINRAIL_OL_STEPPER},
  {0x50, HardwareIdent::DINRAIL_CL_BLDC},
  {0x51, HardwareIdent::DINRAIL_CL_STEPPER},
  {0x52, HardwareIdent::ONAXIS_CL_BLDC},
  {0x53, HardwareIdent::DINRAIL_CL_DC},
  {0x89, HardwareIdent::BOOTLOADER_DSPIC33EP128GM706},
  {0x8A, HardwareIdent::BOOTLOADER_DSPIC33EP256GM706},
  {0x8B, HardwareIdent::BOOTLOADER_DSPIC33EP128MC506},
  {0xB0, HardwareIdent::DC_GRIPPER}};

enum class ControllerType {
  undefined,
  open_loop,
  closed_loop,
};

enum class MotorState {
  disabled,
  disabling,
  enabled,
  enabling,
};

/**
 * @brief TODO
 */
enum class SetToZeroState {
  not_zeroed,
  zeroing_step1,
  zeroing_step2,
  zeroing_step3,
  zeroing_step4,
  zeroed,
};

/**
 * @brief Once this flag is set  to referenced it should stay there in nearly all cases
 * Only forcing a new alignment and having the process fail can result in the
 * state changing from aligned. As it still is possible we read the flag 
 * continuusly from the status message.
 */
enum class ReferenceState {
  not_required,       // Set for e.g. the ReBeL module, which does not need a referencing process
  unreferenced,       // Initial state for modules which require referencing
  referencing_step1,  // State after the first command is sent to the controller
  referencing_step2,  // State after the second command is sent to the controller
  referenced,         // Set either by the referenced bit or a referencing done message
};

/**
 * @brief Once this flag is set to aligned it should stay there in nearly all cases
 * Only forcing a new reference and having the process fail can result in the
 * state changing from referenced. As it still is possible we read the flag 
 * continuusly from the status message.
*/
enum class RotorAlignmentState {
  unaligned,
  aligning_step1,
  aligning_step2,
  aligned,
};

/**
 * @brief This will either succeed from resetting to reset or timeout and change back to not_reset,
 */
enum class ResetState {
  not_reset,
  resetting,
  reset,
};

/**
 * @brief DinFlags[4] in the motion respone message only on closed loop controllers
 * reports us that we can send motion commands now. Open loop controllers dont
 * have this function implemented as of now and as such are always taken to be
 * ready. The not_implemented state is used in this case.
*/
enum class PositioningReadyState {
  // Open loop, old versions of closed loop
  not_implemented,

  // Closed loop
  not_ready,
  ready,
};

enum class CommandMode {
  none,
  position,
  velocity,
  torque,
};

}  // namespace irc_hardware
