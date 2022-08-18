#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdio>

#include<iostream>
#include<fstream>
#include<string>

#include "rclcpp/rclcpp.hpp"


#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"

#include "dynamixel_sdk/dynamixel_sdk.h"

namespace dynamixel_motor_controller {

struct State {
  float position = 0.0;
  float velocity = 0.0;
  float effort = 0.0;
};

struct Safety {
  float temp_lim;
  float min_voltage;
  float max_voltage;
  float max_effort;
};

struct Motor {
  uint8_t id = 0;
  State state;
  State gloal;
  State lower_limit;
  State upper_limit;
  Safety safety;
  float voltage = 0.0;
  float temperature = 0.0;
  uint8_t error_code;
  uint8_t error_cnt;
  bool enabled = false;
  std::chrono::time_point<std::chrono::system_clock> last_error_time;
  builtin_interfaces::msg::Duration time_from_start;
};

class DynamixelMotorControllerNode : public rclcpp::Node
{
public:
  DynamixelMotorControllerNode();

private:
  void timer_callback();
  void setup_serial_port();
  void querry_joints();
  void querry_state();
  void setup_motors();
  void publish_joint_state();
  void publish_controller_state();
  void write_setpoint();
  void torque_enable(bool state);
  void check_bus_errors();

  float data_to_pos(uint8_t *data);
  float data_to_vel(uint8_t *data);
  float data_to_eff(uint8_t *data);
  void pos_to_data(float val, uint8_t *data);
  void vel_to_data(float val, uint8_t *data);
  void eff_to_data(float val, uint8_t *data);

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_subscriber_;


  void trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);

  rclcpp::TimerBase::SharedPtr timer_;
  dynamixel::PortHandler * portHandler_;
  dynamixel::PacketHandler * packetHandler_;
  int dxl_comm_result_ = COMM_TX_FAIL;
  std::string serial_port_;
  int baudrate_;
  std::map<std::string, std::shared_ptr<Motor>> motors_;
  float protocol_ver_;
  int return_delay_;
  int bus_error_count_;
  float error_timeout_;
  std::string joint_state_msg_link_;
  bool e_stop_;
  rclcpp::Time last_command_time_;

  std::chrono::time_point<std::chrono::system_clock> last_msg_printed_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Publisher<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr controller_state_publisher_;
};

static constexpr uint8_t GOAL_POSITION_ADDRES = 30;
static constexpr uint8_t GOAL_VELOCITY_ADDRES = 32;
static constexpr uint8_t GOAL_EFFORT_ADDRES = 34;
static constexpr uint8_t GOAL_NUM_BUTES = 6;

static constexpr uint8_t STATE_POSITION_ADDRESS = 36;
static constexpr uint8_t STATE_VELOCITY_ADDRESS = 38;
static constexpr uint8_t STATE_EFFORT_ADDRESS = 40;
static constexpr uint8_t STATE_JOINT_NUM_BUTES = 6;

static constexpr uint8_t STATE_VOLTAGE_ADDRESS = 42;
static constexpr uint8_t STATE_TEMPERATURE_ADDRESS = 43;
static constexpr uint8_t STATE_PARAMS_NUM_BUTES = 2;

static constexpr uint8_t TORQUE_ENABLE_ADDRESS = 24;
static constexpr uint8_t TORQUE_ENABLE_NUM_BUTES = 1;

static constexpr uint8_t ERROR_BIT_VOLTAGE = 1;
static constexpr uint8_t ERROR_BIT_ANGLE = 2;
static constexpr uint8_t ERROR_BIT_OVERHEAT = 4;
static constexpr uint8_t ERROR_BIT_RANGE = 8;
static constexpr uint8_t ERROR_BIT_CHECKSUM = 16;
static constexpr uint8_t ERROR_BIT_OVERLOAD = 32;
static constexpr uint8_t ERROR_BIT_INSTRUCTION = 64;
}