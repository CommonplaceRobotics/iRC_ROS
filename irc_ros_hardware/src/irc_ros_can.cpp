#include "irc_ros_hardware/irc_ros_can.hpp"

#include <chrono>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "irc_ros_hardware/CAN/digital_io.hpp"
#include "irc_ros_hardware/CAN/joint.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace irc_hardware
{

IrcRosCan::IrcRosCan() { can_interface_ = std::make_shared<CAN::CanInterfaceSocketCAN>(); }

IrcRosCan::~IrcRosCan() {}

/**
 * @brief Parses the ros2_control urdf settings and constructs the respective modules.
 */
hardware_interface::CallbackReturn IrcRosCan::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // CAN socket
  can_socket_ = default_can_socket_;

  if (info_.hardware_parameters.count("can_socket") > 0) {
    can_socket_ = info_.hardware_parameters.at("can_socket");
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("iRC_ROS"), "No can_socket specified in urdf, using default value");
  }

  RCLCPP_INFO(
    rclcpp::get_logger("iRC_ROS"), "CAN socket set as %s in the urdf file", can_socket_.c_str());

  // The first module usually starts with this id
  int can_id = 0x10;

  for (auto & joint : info.joints) {
    // CAN id
    if (joint.parameters.count("can_id") > 0) {
      can_id = stod(joint.parameters.at("can_id"));
    } else {
      RCLCPP_WARN(
        rclcpp::get_logger("iRC_ROS"),
        "Joint \"%s\" no can_id specified in urdf, using default value", joint.name.c_str());
    }

    // Check if a different module already uses the same CAN id
    for (auto & [module_name, module] : modules_) {
      if (module->can_id_ == can_id) {
        RCLCPP_ERROR(
          rclcpp::get_logger("iRC_ROS"), "Joint \"%s\": can id 0x%02x is already in use by %s",
          joint.name.c_str(), can_id, module->get_name().c_str());

        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    // Create joint object since all required information is now available
    std::shared_ptr<Joint> j = std::make_shared<Joint>(joint.name, can_interface_, can_id);

    // Increment assumed can id for next module
    can_id += default_can_id_step_;

    //Gear scale
    if (joint.parameters.count("gear_scale") > 0) {
      j->tics_over_degree_ = stod(joint.parameters.at("gear_scale"));
    } else {
      j->tics_over_degree_ = default_gear_scale_;

      RCLCPP_WARN(
        rclcpp::get_logger("iRC_ROS"),
        "Joint \"%s\" no gear_scale specified in urdf, using default value", joint.name.c_str());
    }

    // Controller type specific parameters
    std::string controller_type;

    // Default values
    controller_type = "unspecified";
    j->temperature_scale_ = 0.1;
    j->positioningReadyState = PositioningReadyState::not_implemented;

    if (joint.parameters.count("controller_type") > 0) {
      controller_type = joint.parameters.at("controller_type");
      if (controller_type == "open_loop") {
        j->controllerType = ControllerType::open_loop;
        j->temperature_scale_ = 0.1;
        j->positioningReadyState = PositioningReadyState::not_implemented;
      } else if (controller_type == "closed_loop") {
        j->controllerType = ControllerType::closed_loop;
        j->temperature_scale_ = 0.01;
        j->positioningReadyState = PositioningReadyState::not_ready;
      } else {
        RCLCPP_WARN(
          rclcpp::get_logger("iRC_ROS"),
          "Joint \"%s\" no valid controller type specified, assuming that there is no positioning "
          "ready bit being sent",
          joint.name.c_str());
      }
    } else {
      RCLCPP_WARN(
        rclcpp::get_logger("iRC_ROS"),
        "Joint \"%s\" no controller type specified, assuming that there is no positioning ready "
        "bit being sent",
        joint.name.c_str());
    }

    // Referencing required?

    j->referenceState = ReferenceState::not_required;  // Default value

    if (joint.parameters.count("referencing_required") > 0) {
      std::string referencing_required = joint.parameters.at("referencing_required");

      if (
        referencing_required == "1" || referencing_required == "true" ||
        referencing_required == "True") {
        j->referenceState = ReferenceState::unreferenced;

        // Check for the priority of the joint in the referencing process
        if (joint.parameters.count("referencing_priority") > 0) {
          int prio = stoi(joint.parameters.at("referencing_priority"));
          j->reference_priority_ = prio;
        }

      } else if (
        referencing_required == "0" || referencing_required == "false" ||
        referencing_required == "False") {
        j->referenceState = ReferenceState::not_required;
      } else {
        RCLCPP_ERROR(
          rclcpp::get_logger("iRC_ROS"), "Joint %s: Unknown value for referencing_required: %s",
          joint.name.c_str(), referencing_required.c_str());
      }
    }
    RCLCPP_INFO(
      rclcpp::get_logger("iRC_ROS"), "Joint %s: Referencing_required: %d", joint.name.c_str(),
      j->referenceState == ReferenceState::unreferenced);

    // Joint configuration summary
    RCLCPP_INFO(
      rclcpp::get_logger("iRC_ROS"),
      "Joint \"%s\" specified with CAN_ID: 0x%02x Controller type: %s Gear scale: %lf",
      joint.name.c_str(), j->can_id_, controller_type.c_str(), j->tics_over_degree_);

    modules_[joint.name] = j;

    // Dont put any module motor startup commands here yet, CAN is likely not up yet
  }

  for (auto & gpio : info.gpios) {
    // CAN id
    if (gpio.parameters.count("can_id") > 0) {
      can_id = stod(gpio.parameters.at("can_id"));
    } else {
      RCLCPP_WARN(
        rclcpp::get_logger("iRC_ROS"), "GPIO \"%s\" no can_id specified in urdf, using ",
        gpio.name.c_str());
    }

    // Check if a different module already uses the same CAN id
    for (auto & [module_name, module] : modules_) {
      if (module->can_id_ == can_id) {
        RCLCPP_ERROR(
          rclcpp::get_logger("iRC_ROS"), "GPIO \"%s\": can id 0x%02x is already in use by %s",
          gpio.name.c_str(), can_id, module->get_name().c_str());

        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    // Create dio object since all required information is now available
    std::shared_ptr<DIO> d = std::make_shared<DIO>(gpio.name, can_interface_, can_id);

    // Increment assumed can id for next module
    can_id += default_can_id_step_;

    RCLCPP_INFO(
      rclcpp::get_logger("iRC_ROS"), "DIO \"%s\" specified with CAN_ID: 0x%02x", gpio.name.c_str(),
      d->can_id_);

    modules_[gpio.name] = d;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief Activates the CAN Interfaces and tries to ping each module.
 */
hardware_interface::CallbackReturn IrcRosCan::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  const hardware_interface::CallbackReturn result = can_interface_->connect(can_socket_)
                                                      ? hardware_interface::CallbackReturn::SUCCESS
                                                      : hardware_interface::CallbackReturn::FAILURE;

  for (auto & [module_name, module] : modules_) {
    // Ping modules to request startup message which contains module information
    module->ping();
  }

  return result;
}

/**
 * @brief Prepares the motors for movement. No mode specific preparation is done yet.
 */
hardware_interface::CallbackReturn IrcRosCan::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  // Check if the interface is still alive
  if (!can_interface_->is_connected()) {
    RCLCPP_ERROR(rclcpp::get_logger("iRC_ROS"), "CAN interface has died!");
    return hardware_interface::CallbackReturn::FAILURE;
  }

  // Insert all modules in a multimap with the priority being the key
  std::multimap<int, Module::Ptr> referencing_priority_map;

  for (auto & [module_name, module] : modules_) {
    referencing_priority_map.insert(
      std::pair<int, Module::Ptr>(module->reference_priority_, module));
  }

  std::chrono::time_point<std::chrono::steady_clock> start_point;

  // And go through the modules now sorted in the order of referencing priority
  for (auto & [priority, module] : referencing_priority_map) {
    // Reset all errors on startup
    start_point = std::chrono::steady_clock::now();
    while (!module->errorState.any_except_mne()) {
      if (std::chrono::steady_clock::now() - start_point > reset_timeout_) {
        RCLCPP_ERROR(
          rclcpp::get_logger("iRC_ROS"), "Failed to reset module 0x%2x", module->can_id_);
        return hardware_interface::CallbackReturn::FAILURE;
      }

      module->reset_error();

      module->read_can();
      module->write_can();

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // If referencing is required do that now
    start_point = std::chrono::steady_clock::now();
    while (module->referenceState != ReferenceState::referenced &&
           module->referenceState != ReferenceState::not_required) {
      // Enable the motors for referencing
      while (module->motorState != MotorState::enabled) {
        if (module->errorState.any_except_mne()) {
          module->reset_error();
        }
        module->enable_motor();

        module->write_can();
        module->read_can();

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }

      // Start referencing
      if (module->referenceState == ReferenceState::unreferenced) {
        module->referencing();
      }

      module->write_can();
      module->read_can();

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Allow resetting during the start of the loop
    module->may_reset_ = true;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief Changes the command mode (part 1). This sets the command modes anew, while the second
 * method resets the modules.
 *
 * TODO: dont return OK on failure, check if all interfaces are found and no duplicates
 */
hardware_interface::return_type IrcRosCan::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  // Reset command mode and goal states
  for (auto stop : stop_interfaces) {
    for (auto && [module_name, module] : modules_) {
      if (stop == module->get_name() + "/" + hardware_interface::HW_IF_POSITION) {
        module->commandMode = CommandMode::none;
        module->set_pos_ = std::numeric_limits<double>::quiet_NaN();
      } else if (stop == module->get_name() + "/" + hardware_interface::HW_IF_VELOCITY) {
        module->commandMode = CommandMode::none;
        module->set_vel_ = std::numeric_limits<double>::quiet_NaN();
      } else if (stop == module->get_name() + "/" + hardware_interface::HW_IF_EFFORT) {
        module->commandMode = CommandMode::none;
        module->set_torque_ = std::numeric_limits<double>::quiet_NaN();
      }
    }
  }

  // Set command modes for starting interfaces
  for (auto start : start_interfaces) {
    for (auto && [module_name, module] : modules_) {
      if (start == module->get_name() + "/" + hardware_interface::HW_IF_POSITION) {
        module->commandMode = CommandMode::position;
      } else if (start == module->get_name() + "/" + hardware_interface::HW_IF_VELOCITY) {
        module->commandMode = CommandMode::velocity;
      } else if (start == module->get_name() + "/" + hardware_interface::HW_IF_EFFORT) {
        module->commandMode = CommandMode::torque;
      }
    }
  }
  return hardware_interface::return_type::OK;
}

/**
 * @brief Changes the command mode (part 2). This does not block, so the error reset and
 * motor enable might fail.
 * 
 * TODO: dont return OK on failure, check if all interfaces are found and no duplicates
 */
hardware_interface::return_type IrcRosCan::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  // Make sure that a module that is stopped and not started again is reset, thus not moving
  for (auto stop : stop_interfaces) {
    for (auto && [module_name, module] : modules_) {
      if (
        stop == module->get_name() + "/" + hardware_interface::HW_IF_POSITION ||
        stop == module->get_name() + "/" + hardware_interface::HW_IF_VELOCITY ||
        stop == module->get_name() + "/" + hardware_interface::HW_IF_EFFORT) {
        module->reset_error();
      }
    }
  }

  // Start the modules that are used with the new interfaces again
  for (auto start : start_interfaces) {
    for (auto && [module_name, module] : modules_) {
      if (
        start == module->get_name() + "/" + hardware_interface::HW_IF_POSITION ||
        start == module->get_name() + "/" + hardware_interface::HW_IF_VELOCITY ||
        start == module->get_name() + "/" + hardware_interface::HW_IF_EFFORT) {
        module->prepare_movement();

        // Allow resetting during the start of the loop
        module->may_reset_ = true;
      }
    }
  }

  return hardware_interface::return_type::OK;
}

/**
 * @brief Stops the motors
 */
hardware_interface::CallbackReturn IrcRosCan::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  for (auto && [module_name, module] : modules_) {
    module->disable_motor();
  }

  // TODO: Wait for the success of the disabling?
  return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief Deactivates the CAN Interfaces
 */
hardware_interface::CallbackReturn IrcRosCan::on_cleanup(
  const rclcpp_lifecycle::State & previous_state)
{
  hardware_interface::CallbackReturn result = can_interface_->disconnect()
                                                ? hardware_interface::CallbackReturn::SUCCESS
                                                : hardware_interface::CallbackReturn::FAILURE;

  return result;
}

/**
 * @brief Reads the incoming data and updates the ros2_control-workaround-double-copies.
 */
hardware_interface::return_type IrcRosCan::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  for (auto & [module_name, module] : modules_) {
    module->read_can();
    module->update_double_copies();
    module->dashboard_command();

    // If an error occurs and we are neither in the process of fixing the error nor have the option
    // to reset the error then stop the hardware interface to avoid unforeseen movements.
    if (
      module->errorState.any() && module->resetState != ResetState::resetting &&
      module->motorState != MotorState::enabling && !module->may_reset_) {
      RCLCPP_ERROR(
        rclcpp::get_logger("iRC_ROS"),
        "0x%2x: Error detected with no recovery option, stopping entire hardware interface for "
        "safety reasons",
        module->can_id_);

      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}

/**
 * @brief Sends the movement and io commands to the modules
*/
hardware_interface::return_type IrcRosCan::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (auto & [module_name, module] : modules_) {
    module->write_can();
  }

  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> IrcRosCan::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Joint specific state_interfaces
  for (auto && joint : info_.joints) {
    //Add the position, velocity states
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.name, hardware_interface::HW_IF_POSITION, &(modules_[joint.name]->pos_)));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.name, hardware_interface::HW_IF_VELOCITY, &(modules_[joint.name]->vel_)));

    //Dashboard specific states (same for gpios and joints)

    // Device info
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.name, "can_id", &(modules_[joint.name]->can_id_double_)));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.name, "hardware_ident", &(modules_[joint.name]->hardware_ident_double_)));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.name, "version_major", &(modules_[joint.name]->version_major_double_)));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.name, "version_minor", &(modules_[joint.name]->version_minor_double_)));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.name, "controller_type", &(modules_[joint.name]->controller_type_double_)));

    // Physical information
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.name, "temperature_board", &(modules_[joint.name]->temperature_board_)));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.name, "temperature_motor", &(modules_[joint.name]->temperature_motor_)));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.name, "supply_voltage", &(modules_[joint.name]->supply_voltage_double_)));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.name, "motor_current", &(modules_[joint.name]->motor_current_double_)));

    // States
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.name, "command_mode", &(modules_[joint.name]->command_mode_double_)));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.name, "error_state", &(modules_[joint.name]->error_state_double_)));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.name, "motor_state", &(modules_[joint.name]->motor_state_double_)));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.name, "reset_state", &(modules_[joint.name]->reset_state_double_)));
  }

  // DIO specific state_interfaces
  for (auto && gpio : info_.gpios) {
    size_t din_counter = 0;
    for (auto && si : gpio.state_interfaces) {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        gpio.name, si.name, &modules_[gpio.name]->digital_in_double_[din_counter]));

      din_counter++;
    }

    // Dashboard specific states, not all contain useful information as joint only states are
    // included as well to make it easier to implement the dashboard

    // Device info
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      gpio.name, "can_id", &(modules_[gpio.name]->can_id_double_)));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      gpio.name, "hardware_ident", &(modules_[gpio.name]->hardware_ident_double_)));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      gpio.name, "version_major", &(modules_[gpio.name]->version_major_double_)));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      gpio.name, "version_minor", &(modules_[gpio.name]->version_minor_double_)));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      gpio.name, "controller_type", &(modules_[gpio.name]->controller_type_double_)));

    // Physical information
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      gpio.name, "temperature_board", &(modules_[gpio.name]->temperature_board_)));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      gpio.name, "temperature_motor", &(modules_[gpio.name]->temperature_motor_)));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      gpio.name, "supply_voltage", &(modules_[gpio.name]->supply_voltage_double_)));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      gpio.name, "motor_current", &(modules_[gpio.name]->motor_current_double_)));

    // States
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      gpio.name, "command_mode", &(modules_[gpio.name]->command_mode_double_)));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      gpio.name, "error_state", &(modules_[gpio.name]->error_state_double_)));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      gpio.name, "motor_state", &(modules_[gpio.name]->motor_state_double_)));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      gpio.name, "reset_state", &(modules_[gpio.name]->reset_state_double_)));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> IrcRosCan::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (auto && joint : info_.joints) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint.name, hardware_interface::HW_IF_POSITION, &(modules_[joint.name]->set_pos_)));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint.name, hardware_interface::HW_IF_VELOCITY, &(modules_[joint.name]->set_vel_)));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint.name, hardware_interface::HW_IF_EFFORT, &(modules_[joint.name]->set_torque_)));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint.name, "dashboard_command", &(modules_[joint.name]->dashboard_cmd_double_)));
  }

  for (auto && gpio : info_.gpios) {
    size_t dout_counter = 0;
    for (auto && ci : gpio.command_interfaces) {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        gpio.name, ci.name, &modules_[gpio.name]->digital_out_double_[dout_counter]));
      dout_counter++;
    }
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      gpio.name, "dashboard_command", &(modules_[gpio.name]->dashboard_cmd_double_)));
  }

  return command_interfaces;
}
}  // namespace irc_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(irc_hardware::IrcRosCan, hardware_interface::SystemInterface)
