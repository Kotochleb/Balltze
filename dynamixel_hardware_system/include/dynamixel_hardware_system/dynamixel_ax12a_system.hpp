#ifndef DYNAMIXEL_AX12A_SYSTEM_
#define DYNAMIXEL_AX12A_SYSTEM_

#include <map>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "dynamixel_sdk/dynamixel_sdk.h"

#include "dynamixel_hardware_system/ax_12_a_motor.hpp"
#include "dynamixel_hardware_system/visibility_control.hpp"


namespace dynamixel_ax12a_system {

using return_type = hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using StateInterface = hardware_interface::StateInterface;
using CommandInterface = hardware_interface::CommandInterface;

class DynamixelAX12ASystem : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DynamixelAX12ASystem)

  DYNAMIXEL_AX12A_SYSTEM_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo& hardware_info) override;

  DYNAMIXEL_AX12A_SYSTEM_PUBLIC
  std::vector<StateInterface> export_state_interfaces() override;

  DYNAMIXEL_AX12A_SYSTEM_PUBLIC
  std::vector<CommandInterface> export_command_interfaces() override;

  DYNAMIXEL_AX12A_SYSTEM_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;

  DYNAMIXEL_AX12A_SYSTEM_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;

  DYNAMIXEL_AX12A_SYSTEM_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;

  DYNAMIXEL_AX12A_SYSTEM_PUBLIC
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State&);

  DYNAMIXEL_AX12A_SYSTEM_PUBLIC
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State&);

  DYNAMIXEL_AX12A_SYSTEM_PUBLIC
  CallbackReturn on_error(const rclcpp_lifecycle::State&);

  DYNAMIXEL_AX12A_SYSTEM_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time&, const rclcpp::Duration&) override;

  DYNAMIXEL_AX12A_SYSTEM_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time&, const rclcpp::Duration&) override;


private:
  std::string serial_port_;
  int baudrate_;
  int return_delay_;
  bool motors_connected_ = false;

  std::shared_ptr<dynamixel::PortHandler> portHandler_;
  std::shared_ptr<dynamixel::PacketHandler> packetHandler_;
  std::unique_ptr<ax_12_a_motor::AX12AMotor> broadcast_motor_; 

  // Store the command for the simulated robot
  std::map<std::string, std::unique_ptr<ax_12_a_motor::AX12AMotor>> motors_;
  std::map<std::string, double> pos_commands_;
  std::map<std::string, double> vel_commands_;
  std::map<std::string, double> eff_commands_;
  std::map<std::string, double> pos_state_;
  std::map<std::string, double> vel_state_;
  std::map<std::string, double> eff_state_;

  const std::vector<std::string> hardware_parameters_ = {
    "serial_port",
    "baudrate",
    "return_delay"
  };
  const std::vector<std::string> command_interfaces_ = {
    hardware_interface::HW_IF_POSITION,
    hardware_interface::HW_IF_VELOCITY,
    hardware_interface::HW_IF_EFFORT
  };

  const std::vector<std::string> state_interfaces_ = {
    hardware_interface::HW_IF_POSITION,
    hardware_interface::HW_IF_VELOCITY,
    hardware_interface::HW_IF_EFFORT
  };

};
}



#endif //DYNAMIXEL_AX12A_SYSTEM_