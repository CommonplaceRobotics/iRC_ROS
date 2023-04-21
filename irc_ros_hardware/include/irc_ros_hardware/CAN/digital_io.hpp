#pragma once

#include <bitset>
#include <memory>

#include "module.hpp"
namespace irc_hardware
{
class DIO : public Module
{
public:
  DIO(std::string name, std::shared_ptr<CAN::CanInterface> can_interface, int can_id);

  void position_cmd() override;

  void read_can() override;
  void write_can() override;

  void velocity_cmd() {}
  void torque_cmd() {}

  void set_position_to_zero() {}
  void referencing() {}
  void rotor_alignment() {}
};

}  // namespace irc_hardware
