#ifndef BALLTZE_SIMULATION_
#define BALLTZE_SIMULATION_

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

#include "balltze_simulation/visibility_control.hpp"

#include "balltze_simulation/gz_tip.hpp"


namespace balltze_tip_sensor {

using return_type = hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using StateInterface = hardware_interface::StateInterface;
using CommandInterface = hardware_interface::CommandInterface;

class BalltzeTipSensor : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(BalltzeDynamixelSystem)

  BALLTZE_SIMULATION_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo& hardware_info) override;

  BALLTZE_SIMULATION_PUBLIC
  std::vector<StateInterface> export_state_interfaces() override;

  BALLTZE_SIMULATION_PUBLIC
  std::vector<CommandInterface> export_command_interfaces() override;

  BALLTZE_SIMULATION_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;

  BALLTZE_SIMULATION_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;

  BALLTZE_SIMULATION_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;

  BALLTZE_SIMULATION_PUBLIC
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State&);

  BALLTZE_SIMULATION_PUBLIC
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State&);

  BALLTZE_SIMULATION_PUBLIC
  CallbackReturn on_error(const rclcpp_lifecycle::State&);

  BALLTZE_SIMULATION_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time&, const rclcpp::Duration&) override;

  BALLTZE_SIMULATION_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time&, const rclcpp::Duration&) override;



private:
  std::map<std::string, gz_tip::GZTip> tip_stensors_;
  std::map<std::string, bool> contact_states_;

  ignition::transport::Node node_;
  std::chrono::steady_clock::duration stamp_;
  std::chrono::steady_clock::duration timeout_;

  void clk_cb(const ignition::msgs::Clock &msg);

  const std::vector<std::string> hardware_parameters_ = {
    "topic",
  };

  static constexpr uint8_t GPIO_LOGICAL_INTERFACE {"logical"};

};
}

#endif //BALLTZE_SIMULATION_