#ifndef BALLTZE_HARDWARE_
#define BALLTZE_HARDWARE_

#include <condition_variable>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "dynamixel_sdk/dynamixel_sdk.h"

#include "balltze_hardware/visibility_control.hpp"


namespace balltze_tip_gpio {

using return_type = hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using StateInterface = hardware_interface::StateInterface;
using CommandInterface = hardware_interface::CommandInterface;

class BalltzeTipGPIO : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(BalltzeDynamixelSystem)

  BALLTZE_HARDWARE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo& hardware_info) override;

  BALLTZE_HARDWARE_PUBLIC
  std::vector<StateInterface> export_state_interfaces() override;

  BALLTZE_HARDWARE_PUBLIC
  std::vector<CommandInterface> export_command_interfaces() override;

  BALLTZE_HARDWARE_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;

  BALLTZE_HARDWARE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;

  BALLTZE_HARDWARE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;

  BALLTZE_HARDWARE_PUBLIC
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State&);

  BALLTZE_HARDWARE_PUBLIC
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State&);

  BALLTZE_HARDWARE_PUBLIC
  CallbackReturn on_error(const rclcpp_lifecycle::State&);

  BALLTZE_HARDWARE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time&, const rclcpp::Duration&) override;

  BALLTZE_HARDWARE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time&, const rclcpp::Duration&) override;



private:
  std::unique_ptr<std::string> pin_paths_; 

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
    hardware_interface::HW_IF_VELOCITY
  };

  const std::vector<std::string> state_interfaces_ = {
    hardware_interface::HW_IF_POSITION,
    hardware_interface::HW_IF_VELOCITY,
    hardware_interface::HW_IF_EFFORT
  };

};
}

#endif //BALLTZE_HARDWARE_