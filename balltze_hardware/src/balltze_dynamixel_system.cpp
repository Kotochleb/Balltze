#include "balltze_hardware/balltze_dynamixel_system.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdio>
#include <regex>
#include <cmath>
#include <iostream>
#include <fstream>
#include <string>
#include <thread>

#include "rclcpp/logging.hpp"

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace std::chrono_literals;

namespace balltze_dynamixel_system {

CallbackReturn BalltzeDynamixelSystem::on_init(const hardware_interface::HardwareInfo& hardware_info) {
  if (hardware_interface::SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  if (info_.hardware_parameters.size() != hardware_parameters_.size()) {
    RCLCPP_FATAL(rclcpp::get_logger("BalltzeDynamixelSystem"), "System %s has %zu parameters. %zu expected.",
                info_.name.c_str(), info_.hardware_parameters.size(), hardware_parameters_.size());
    return CallbackReturn::ERROR;
  }

  for (auto & parameter : hardware_parameters_) {
    if (info_.hardware_parameters.find(parameter) == info_.hardware_parameters.end()) {
     RCLCPP_FATAL(rclcpp::get_logger("BalltzeDynamixelSystem"), "System '%s' does not have '%s' parameter specified.",
                  info_.name.c_str(), parameter.c_str());
    return CallbackReturn::ERROR;
    }
  }  


  for (const hardware_interface::ComponentInfo& joint : info_.joints) {
    if (joint.command_interfaces.size() != command_interfaces_.size()) {
      RCLCPP_FATAL(rclcpp::get_logger("BalltzeDynamixelSystem"), "Joint '%s' has %zu command interfaces found. 3 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    for (std::size_t i = 0; i < joint.command_interfaces.size(); i++) {
      if (joint.command_interfaces[i].name != command_interfaces_[i]) {
        RCLCPP_FATAL(rclcpp::get_logger("BalltzeDynamixelSystem"), "Joint '%s' have %s command interfaces found. '%s' expected.",
                    joint.name.c_str(), joint.command_interfaces[i].name.c_str(), command_interfaces_[i].c_str());
        return CallbackReturn::ERROR;
      }
    }

    if (joint.state_interfaces.size() != state_interfaces_.size()) {
      RCLCPP_FATAL(rclcpp::get_logger("BalltzeDynamixelSystem"), "Joint '%s' has %zu state interface. 3 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    for (std::size_t i = 0; i < joint.state_interfaces.size(); i++) {
      if (joint.state_interfaces[0].name != state_interfaces_[i]) {
        RCLCPP_FATAL(rclcpp::get_logger("BalltzeDynamixelSystem"), "Joint '%s' have '%s' state interface. '%s' expected.",
                    joint.name.c_str(), joint.state_interfaces[0].name.c_str(), state_interfaces_[i].c_str());
        return CallbackReturn::ERROR;
      }
    }

    if (joint.parameters.size() != 1) {
      RCLCPP_FATAL(rclcpp::get_logger("BalltzeDynamixelSystem"), "Joint '%s' has %zu parameters. 1 expected.",
                   joint.name.c_str(), joint.parameters.size());
      return CallbackReturn::ERROR;
    }

    if (joint.parameters.find("id") == joint.parameters.end()) {
      RCLCPP_FATAL(rclcpp::get_logger("BalltzeDynamixelSystem"), "Joint '%s' does not have 'id' parameter specified.",
                   joint.name.c_str());
      return CallbackReturn::ERROR;
    }
  }

  // Check get serial port name and check is matches '/dev/<serial_port>' convention
  serial_port_ = info_.hardware_parameters.at("serial_port");
  std::regex expr ("/dev/[a-zA-Z0-9]*$");
  if (!std::regex_match(serial_port_, expr)) {
    RCLCPP_FATAL(rclcpp::get_logger("BalltzeDynamixelSystem"),
      "Device name: (%s) does not match convention '/dev/<name>'.", serial_port_.c_str());
    return CallbackReturn::ERROR;
  }

  // Initialise port handlers for Dynamixel
  portHandler_ = std::shared_ptr<dynamixel::PortHandler>(dynamixel::PortHandler::getPortHandler(serial_port_.c_str()));
  packetHandler_ = std::shared_ptr<dynamixel::PacketHandler>(dynamixel::PacketHandler::getPacketHandler(1.0));

  // Get baudrate parameter and check if is int
  const std::string baudrate_str = info_.hardware_parameters.at("baudrate");
  try {
    baudrate_ = std::stoi(baudrate_str);
  }
  catch(std::invalid_argument const& ex) {
    RCLCPP_FATAL(rclcpp::get_logger("BalltzeDynamixelSystem"), "'baudrate' parameter for system '%s' is incorrect type.",
                  info_.name.c_str());
    return CallbackReturn::ERROR;
  }

  // Get return_delay parameter and check if is int
  const std::string return_delay_str = info_.hardware_parameters.at("return_delay");
  try {
    return_delay_ = std::stoi(return_delay_str);
  }
  catch(std::invalid_argument const& ex) {
    RCLCPP_FATAL(rclcpp::get_logger("BalltzeDynamixelSystem"), "'return_delay' parameter for system '%s' is incorrect type.",
                  info_.name.c_str());
    return CallbackReturn::ERROR;
  }

  for (auto& joint : info_.joints) {
    RCLCPP_INFO(rclcpp::get_logger("BalltzeDynamixelSystem"), "Joint '%s' found", joint.name.c_str());

    try {
      const int id = std::stoi(joint.parameters.at("id"));
      motors_.insert({joint.name, std::make_unique<ax_12_a_motor::AX12AMotor>(id, portHandler_, packetHandler_)});
    }
    catch(std::invalid_argument const& ex) {
      RCLCPP_FATAL(rclcpp::get_logger("BalltzeDynamixelSystem"), "Joint '%s' parameter: 'id' is invalid type.",
                   joint.name.c_str());
      return CallbackReturn::ERROR;
    }
  }

  broadcast_motor_ = std::make_unique<ax_12_a_motor::AX12AMotor>(254, portHandler_, packetHandler_);

  return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> BalltzeDynamixelSystem::export_state_interfaces() {
  std::vector<StateInterface> state_interfaces;
  for (auto & joint : info_.joints)   {
    state_interfaces.emplace_back(
      StateInterface(joint.name, hardware_interface::HW_IF_POSITION, &pos_state_[joint.name]));
    state_interfaces.emplace_back(
      StateInterface(joint.name, hardware_interface::HW_IF_VELOCITY, &vel_state_[joint.name]));
    state_interfaces.emplace_back(
      StateInterface(joint.name, hardware_interface::HW_IF_EFFORT, &eff_state_[joint.name]));
  }

  return state_interfaces;
}

std::vector<CommandInterface> BalltzeDynamixelSystem::export_command_interfaces() {
  std::vector<CommandInterface> command_interfaces;
  for (auto & joint : info_.joints) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint.name, hardware_interface::HW_IF_POSITION, &pos_commands_[joint.name]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint.name, hardware_interface::HW_IF_VELOCITY, &vel_commands_[joint.name]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint.name, hardware_interface::HW_IF_EFFORT, &eff_commands_[joint.name]));
  }

  return command_interfaces;
}

CallbackReturn BalltzeDynamixelSystem::on_configure(const rclcpp_lifecycle::State&)
{
  
  // Check if serial port exists
  if (!std::ifstream(serial_port_.c_str()).good()) {
    RCLCPP_FATAL(rclcpp::get_logger("BalltzeDynamixelSystem"), "No device: %s.", serial_port_.c_str());
    return CallbackReturn::ERROR;
  }

  // Check if specyfied serial port is FT232 and check if it is possible to change latency
  std::stringstream latency_file_path_strem;
  size_t dev_name_idx = serial_port_.find_last_of('/'); 
  auto const dev_name = serial_port_.substr(dev_name_idx + 1);
  latency_file_path_strem << "/sys/bus/usb-serial/devices/" << dev_name << "/latency_timer";
  auto const latency_file_path = latency_file_path_strem.str();
  RCLCPP_DEBUG(rclcpp::get_logger("BalltzeDynamixelSystem"), "Changing FT232 latency timer at path '%s'.", latency_file_path.c_str());

  if (!std::ifstream(latency_file_path.c_str()).good()) {
    RCLCPP_FATAL(rclcpp::get_logger("BalltzeDynamixelSystem"), "No file: %s. Failed to change FT232 latency timer.", latency_file_path.c_str());
    return CallbackReturn::ERROR;
  }

  // Set latency of FT232 to 1us
  std::ofstream latency_timer_file;
  try {
    latency_timer_file.open(latency_file_path.c_str(), std::ofstream::out | std::ofstream::trunc);
    latency_timer_file << 1;
    RCLCPP_DEBUG(rclcpp::get_logger("BalltzeDynamixelSystem"), "Succeeded to change FT232 latency timer at device '%s'.", serial_port_.c_str());
  }
  catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("BalltzeDynamixelSystem"), "Failed to change FT232 latency timer at device '%s'.", serial_port_.c_str());
    return CallbackReturn::ERROR;
  }
  latency_timer_file.close();

  if (!portHandler_->openPort()) {
    RCLCPP_FATAL(rclcpp::get_logger("BalltzeDynamixelSystem"), "Failed to open serial port at device '%s'.", serial_port_.c_str());
    return CallbackReturn::ERROR;
  } else {
    RCLCPP_DEBUG(rclcpp::get_logger("BalltzeDynamixelSystem"), "Succeeded to open serial port at device '%s'.", serial_port_.c_str());
  }

  if (!portHandler_->setBaudRate(baudrate_)) {
    RCLCPP_FATAL(rclcpp::get_logger("BalltzeDynamixelSystem"), "Failed to set the baudrate_!");
    return CallbackReturn::ERROR;
  } else {
    RCLCPP_DEBUG(rclcpp::get_logger("BalltzeDynamixelSystem"), "Succeeded to set the baudrate_.");
  }
  
  return CallbackReturn::SUCCESS;
}

CallbackReturn BalltzeDynamixelSystem::on_activate(const rclcpp_lifecycle::State&)
{
  // Check if motors are connected and can be reached
  RCLCPP_INFO(rclcpp::get_logger("BalltzeDynamixelSystem"), "Pinging motors...");
  for (auto & joint : info_.joints) {
    if (motors_.at(joint.name)->ping()) {
      RCLCPP_DEBUG(rclcpp::get_logger("BalltzeDynamixelSystem"), "Motor at joint '%s' successfully pinged.", joint.name.c_str());
    }
    else {
      RCLCPP_FATAL(rclcpp::get_logger("BalltzeDynamixelSystem"), "Failed to ping motor at joint '%s'!", joint.name.c_str());
      return CallbackReturn::ERROR;
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("BalltzeDynamixelSystem"), "Successfully pinged all motors.");


  // Reboot motors to clear all errors
  RCLCPP_INFO(rclcpp::get_logger("BalltzeDynamixelSystem"), "Rebooting motors...");
  for (auto & joint : info_.joints) {
    motors_.at(joint.name)->reboot();
  }

  // Wait for the reboot
  std::this_thread::sleep_for(1000ms);

  // Check if motors rebooted
  for (auto & joint : info_.joints) {
    if (motors_.at(joint.name)->ping()) {
      RCLCPP_DEBUG(rclcpp::get_logger("BalltzeDynamixelSystem"), "Motor at joint '%s' successfully pinged after reboot.", joint.name.c_str());
    }
    else {
      RCLCPP_FATAL(rclcpp::get_logger("BalltzeDynamixelSystem"), "Failed to ping motor at joint '%s' after reboot!", joint.name.c_str());
      return CallbackReturn::ERROR;
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("BalltzeDynamixelSystem"), "Successfully rebooted motors.");
  motors_connected_ = true;

  // Set return delay to sepcyfied in perameter
  RCLCPP_INFO(rclcpp::get_logger("BalltzeDynamixelSystem"), "Setting return delay for motors...");
  for (auto & joint : info_.joints) {
    motors_.at(joint.name)->set_return_dealy(return_delay_);
    auto motor_error = motors_.at(joint.name)->get_motor_error_report();
    if (motor_error->checksum || motor_error->instruction) {
      RCLCPP_ERROR(rclcpp::get_logger("BalltzeDynamixelSystem"), "Failed to set return delay at joint '%s'!", joint.name.c_str());
      return CallbackReturn::ERROR;
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("BalltzeDynamixelSystem"), "Successfully set return delay for all motors.");

  // Enable motors
  RCLCPP_INFO(rclcpp::get_logger("BalltzeDynamixelSystem"), "Enabling torque...");
  for (auto & joint : info_.joints) {
    motors_.at(joint.name)->enable_torque(true);
    auto motor_error = motors_.at(joint.name)->get_motor_error_report();
    if (motor_error->checksum || motor_error->instruction) {
      RCLCPP_ERROR(rclcpp::get_logger("BalltzeDynamixelSystem"), "Failed to enable torque for joint'%s'!", joint.name.c_str());
      return CallbackReturn::ERROR;
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("BalltzeDynamixelSystem"), "Successfully enabled torque for all motors.");

  // Blink LED 3 times to indicate correct setup
  for (size_t i = 0; i < 3; i++) {
    for (auto & joint : info_.joints) {
      motors_.at(joint.name)->set_led(true);
    }

    std::this_thread::sleep_for(1000ms);

    for (auto & joint : info_.joints) {
      motors_.at(joint.name)->set_led(false);
    }
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn BalltzeDynamixelSystem::on_deactivate(const rclcpp_lifecycle::State&)
{
  // Turn off torque and LEDs
  if (motors_connected_) {
    for (auto & joint : info_.joints) {
      motors_.at(joint.name)->enable_torque(false);
      motors_.at(joint.name)->set_led(false);
    }
  }
  motors_connected_ = false;
  return CallbackReturn::SUCCESS;
}

CallbackReturn BalltzeDynamixelSystem::on_cleanup(const rclcpp_lifecycle::State&)
{
  // Turn off torque and LEDs
  if (motors_connected_) {
    for (auto & joint : info_.joints) {
      motors_.at(joint.name)->enable_torque(false);
      motors_.at(joint.name)->set_led(false);
    }
  }
  motors_connected_ = false;
  return CallbackReturn::SUCCESS;
}

CallbackReturn BalltzeDynamixelSystem::on_shutdown(const rclcpp_lifecycle::State&)
{
  // Turn off torque and LEDs
  if (motors_connected_) {
    for (auto & joint : info_.joints) {
      motors_.at(joint.name)->enable_torque(false);
      motors_.at(joint.name)->set_led(false);
    }
  }
  motors_connected_ = false;
  return CallbackReturn::SUCCESS;
}

CallbackReturn BalltzeDynamixelSystem::on_error(const rclcpp_lifecycle::State&)
{
  // Keep current torque state and turn on LED
  if (motors_connected_) {
    for (auto & joint : info_.joints) {
      motors_.at(joint.name)->set_led(true);
    }
  }
  motors_connected_ = false;
  return CallbackReturn::SUCCESS;
}

return_type BalltzeDynamixelSystem::read(const rclcpp::Time&, const rclcpp::Duration&)
{
  for (auto & joint : info_.joints) {
    motors_.at(joint.name)->qeurry_joint();
    auto motor_error = motors_.at(joint.name)->get_motor_error_report();
    if (motor_error->checksum) {
      pos_state_[joint.name] = std::numeric_limits<double>::quiet_NaN();
      vel_state_[joint.name] = std::numeric_limits<double>::quiet_NaN();
      eff_state_[joint.name] = std::numeric_limits<double>::quiet_NaN();
      RCLCPP_ERROR(rclcpp::get_logger("BalltzeDynamixelSystem"), "Failed read state at joint '%s'!", joint.name.c_str());
    }
    else if (motor_error->overload) {
      RCLCPP_ERROR(rclcpp::get_logger("BalltzeDynamixelSystem"), "Motor at joint '%s' is overloaded!", joint.name.c_str());
      return return_type::ERROR;
    }
    else if (motor_error->overheat) {
      RCLCPP_ERROR(rclcpp::get_logger("BalltzeDynamixelSystem"), "Motor at joint '%s' is overheated!", joint.name.c_str());
      return return_type::ERROR;
    }
    else {
      pos_state_[joint.name] =  motors_.at(joint.name)->get_position();
      vel_state_[joint.name] =  motors_.at(joint.name)->get_velocity();
      eff_state_[joint.name] =  motors_.at(joint.name)->get_effort();
    }
  }
  
  return return_type::OK;
}

return_type BalltzeDynamixelSystem::write(const rclcpp::Time&, const rclcpp::Duration&)
{
  for (auto & joint : info_.joints) {
    motors_.at(joint.name)->set_state(
      pos_commands_.at(joint.name),
      vel_commands_.at(joint.name),
      eff_commands_.at(joint.name)
    );

    auto motor_error = motors_.at(joint.name)->get_motor_error_report();
    if (motor_error->checksum) {
      pos_state_[joint.name] = std::numeric_limits<double>::quiet_NaN();
      vel_state_[joint.name] = std::numeric_limits<double>::quiet_NaN();
      eff_state_[joint.name] = std::numeric_limits<double>::quiet_NaN();
      RCLCPP_ERROR(rclcpp::get_logger("BalltzeDynamixelSystem"), "Failed to set state at joint '%s'!", joint.name.c_str());
      return return_type::ERROR;
    }
    else if (motor_error->range) {
      pos_state_[joint.name] = std::numeric_limits<double>::quiet_NaN();
      vel_state_[joint.name] = std::numeric_limits<double>::quiet_NaN();
      eff_state_[joint.name] = std::numeric_limits<double>::quiet_NaN();
      RCLCPP_ERROR(rclcpp::get_logger("BalltzeDynamixelSystem"), "Instruction for motor '%s' is out of range!", joint.name.c_str());
      return return_type::ERROR;
    }
    else if (motor_error->overload) {
      RCLCPP_ERROR(rclcpp::get_logger("BalltzeDynamixelSystem"), "Motor at joint '%s' is overloaded!", joint.name.c_str());
      return return_type::ERROR;
    }
    else if (motor_error->overheat) {
      RCLCPP_ERROR(rclcpp::get_logger("BalltzeDynamixelSystem"), "Motor at joint '%s' is overheated!", joint.name.c_str());
      return return_type::ERROR;
    }
  }

  broadcast_motor_->action();
  
  return return_type::OK;
}

}  // namespace dynamixel_ax12a_system

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(balltze_dynamixel_system::BalltzeDynamixelSystem, hardware_interface::SystemInterface)