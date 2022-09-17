#include "balltze_hardware/dynamixel_motor_node.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdio>

#include <regex>
#include <cmath>

#include<iostream>
#include<fstream>
#include<string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"

#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace std::chrono_literals;
using namespace dynamixel_motor_controller;
using std::placeholders::_1;

DynamixelMotorControllerNode::DynamixelMotorControllerNode() : 
  Node("dynamixel_motor_controller_node") {

  this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
  this->declare_parameter<int>("baudrate", 1000000);
  this->declare_parameter<int>("return_delay", 50);
  this->declare_parameter<int>("bus_error_count", 20);
  this->declare_parameter<float>("error_timeout", 5.0);
  this->declare_parameter<std::string>("joint_state_msg_link", "base_link");
  this->declare_parameter<std::vector<std::string>>("joint_names");
  this->declare_parameter<float>("frequency", 20.0);
  


  std::vector<std::string> joint_names;
  this->get_parameter("joint_names", joint_names);
  for (auto & joint : joint_names) {
    auto motor = std::make_shared<Motor>();
    this->declare_parameter<uint8_t>(joint + ".id", 0);
    this->declare_parameter<float>(joint + ".lower_limit.position", -M_PI/2.0f);
    this->declare_parameter<float>(joint + ".lower_limit.velocity", -6.17);
    this->declare_parameter<float>(joint + ".lower_limit.effort", -1.5);
    this->declare_parameter<float>(joint + ".upper_limit.position", M_PI/2.0f);
    this->declare_parameter<float>(joint + ".upper_limit.velocity", 6.17);
    this->declare_parameter<float>(joint + ".upper_limit.effort", 1.5);
    this->declare_parameter<float>(joint + ".safety.temp_lim", 70.0f);
    this->declare_parameter<float>(joint + ".safety.min_voltage", 6.0f);
    this->declare_parameter<float>(joint + ".safety.max_voltage", 14.0f);
    this->declare_parameter<float>(joint + ".safety.max_effort", 1.5f);


    this->get_parameter(joint + ".id", motor->id);
    this->get_parameter(joint + ".default.position", motor->state.position);
    this->get_parameter(joint + ".lower_limit.position", motor->lower_limit.position);
    this->get_parameter(joint + ".lower_limit.velocity", motor->lower_limit.velocity);
    this->get_parameter(joint + ".lower_limit.effort", motor->lower_limit.effort);
    this->get_parameter(joint + ".upper_limit.position", motor->upper_limit.position);
    this->get_parameter(joint + ".upper_limit.velocity", motor->upper_limit.velocity);
    this->get_parameter(joint + ".upper_limit.effort", motor->upper_limit.effort);
    this->get_parameter(joint + ".safety.temp_lim", motor->safety.temp_lim);
    this->get_parameter(joint + ".safety.min_voltage", motor->safety.min_voltage);
    this->get_parameter(joint + ".safety.max_voltage", motor->safety.max_voltage);
    this->get_parameter(joint + ".safety.max_effort", motor->safety.max_effort);

    motors_.insert({joint, motor});
  }

  this->get_parameter("serial_port", serial_port_);
  this->get_parameter("baudrate", baudrate_);
  this->get_parameter("return_delay", return_delay_);

  this->get_parameter("bus_error_count", bus_error_count_);
  this->get_parameter("error_timeout", error_timeout_);
  this->get_parameter("joint_state_msg_link", joint_state_msg_link_);

  float frequency;
  this->get_parameter("frequency", frequency);

  setup_serial_port();
  RCLCPP_INFO(this->get_logger(), "Serial port initialised.");

  for (auto const& [joint, motor] : motors_) {
    int ping_resp = packetHandler_->ping(portHandler_, motor->id, &motor->error_code);
    if (ping_resp) {
      RCLCPP_FATAL(this->get_logger(), "Can not ping motor attached to joint: %s with ID: %u.", joint.c_str(), motor->id);
      exit(1);
    }
    if (motor->error_code & ERROR_BIT_CHECKSUM) {
      RCLCPP_FATAL(this->get_logger(), "Bus error 0x%02X for motor with id: %u!", motor->error_code, motor->id);
      exit(1);
    }
  }
  RCLCPP_INFO(this->get_logger(), "Succesfully pinged all motors.");

  torque_enable(true);
  querry_state();
  querry_joints();


  last_command_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);




  joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  controller_state_publisher_ = this->create_publisher<
    control_msgs::msg::JointTrajectoryControllerState>("controller_state", 10);

  trajectory_subscriber_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "joint_trajectory", rclcpp::SystemDefaultsQoS(),
    std::bind(&DynamixelMotorControllerNode::trajectory_callback, this, _1));

  timer_ = this->create_wall_timer(
    std::chrono::duration<float>(1.0f/frequency),
    std::bind(&DynamixelMotorControllerNode::timer_callback, this));


  RCLCPP_INFO(this->get_logger(), "Node initialised.");
}

void DynamixelMotorControllerNode::trajectory_callback(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
  auto joint_names = msg->joint_names;
  auto joint_traj = msg->points;
  if (joint_names.size() != joint_traj.size()) {
    RCLCPP_ERROR(this->get_logger(),
      "Provided %lu joint names which does not match %lu joint trajectory points!",
      joint_names.size(), joint_traj.size());
    return;
  }

  // if (last_command_time_ >= msg->header.stamp) {
  //   RCLCPP_WARN(this->get_logger(), "Dropping incomming trajectory. Older than previous one!");
  //   return;
  // }
  last_command_time_ = msg->header.stamp;

  for (std::size_t i = 0; i <= joint_names.size(); i++) {
    auto joint_name = joint_names.at(i);
    if (motors_.find(joint_name) == motors_.end()) {
      RCLCPP_WARN(this->get_logger(), "Joint '%s' not found!", joint_name.c_str());
      continue;
    }

    auto positions = joint_traj[i].positions;
    if (positions.size() == 0) {
      RCLCPP_WARN(this->get_logger(), "No position provided for joint '%s'", joint_name.c_str());
      continue;
    }
    float target_pos = positions.at(0);
    if (target_pos >= motors_.at(joint_name)->upper_limit.position) {
      target_pos = motors_.at(joint_name)->upper_limit.position;
    }
    else if (target_pos <= motors_.at(joint_name)->lower_limit.position) {
      target_pos = motors_.at(joint_name)->lower_limit.position;
    }
    motors_.at(joint_name)->gloal.position = target_pos;


    auto velocities = joint_traj[i].velocities;
    float target_vel = 0.0f;
    if (velocities.size() > 0) {
      target_vel = velocities.at(0);
      if (target_vel >= motors_.at(joint_name)->upper_limit.velocity) {
        target_vel = motors_.at(joint_name)->upper_limit.velocity;
      }
      else if (target_vel <= motors_.at(joint_name)->lower_limit.velocity) {
        target_vel = motors_.at(joint_name)->lower_limit.velocity;
      }
    }
    else {
      target_vel = motors_.at(joint_name)->upper_limit.velocity;
    }
    motors_.at(joint_name)->gloal.velocity = target_vel;


    auto effort = joint_traj[i].effort;
    float target_eff = 0.0f;
    if (effort.size() > 0) {
      target_eff = effort.at(0);
      if (target_eff >= motors_.at(joint_name)->upper_limit.effort) {
        target_eff = motors_.at(joint_name)->upper_limit.effort;
      }
      else if (target_eff <= motors_.at(joint_name)->lower_limit.effort) {
        target_eff = motors_.at(joint_name)->lower_limit.effort;
      }
    }
    else {
      target_eff = motors_.at(joint_name)->upper_limit.effort;
    }

    motors_.at(joint_name)->gloal.effort = target_eff;

    motors_.at(joint_name)->time_from_start = joint_traj.at(i).time_from_start;
  }
}

void DynamixelMotorControllerNode::publish_joint_state() {
  auto joint_msg = sensor_msgs::msg::JointState();
  joint_msg.header.stamp = this->get_clock()->now();
  joint_msg.header.frame_id = joint_state_msg_link_;
  for (auto const& [joint, motor] : motors_) {
    joint_msg.name.push_back(joint);
    joint_msg.position.push_back(motor->state.position);
    joint_msg.velocity.push_back(motor->state.velocity);
    joint_msg.effort.push_back(motor->state.effort);
  }
  joint_state_publisher_->publish(joint_msg);
}

void DynamixelMotorControllerNode::publish_controller_state() {
  auto controller_state_msg = control_msgs::msg::JointTrajectoryControllerState();
  controller_state_msg.header.stamp = this->get_clock()->now();
  controller_state_msg.header.frame_id = joint_state_msg_link_;
  for (auto const& [joint, motor] : motors_) {
    controller_state_msg.joint_names.push_back(joint);
    controller_state_msg.desired.positions.push_back(motor->state.position);
    controller_state_msg.actual.positions.push_back(motor->gloal.position);
    controller_state_msg.error.positions.push_back(motor->gloal.position - motor->state.position);

    controller_state_msg.desired.velocities.push_back(motor->state.velocity);
    controller_state_msg.actual.velocities.push_back(motor->gloal.velocity);
    controller_state_msg.error.velocities.push_back(motor->gloal.velocity - motor->state.velocity);

    controller_state_msg.desired.accelerations.push_back(NAN);
    controller_state_msg.actual.accelerations.push_back(NAN);
    controller_state_msg.error.accelerations.push_back(NAN);

    controller_state_msg.desired.effort.push_back(motor->state.effort);
    controller_state_msg.actual.effort.push_back(motor->gloal.effort);
    controller_state_msg.error.effort.push_back(motor->gloal.effort - motor->state.effort);

    controller_state_msg.desired.time_from_start = motor->time_from_start;
  }
  controller_state_publisher_->publish(controller_state_msg);
}

void DynamixelMotorControllerNode::setup_serial_port() {
  std::regex expr ("/dev/[a-zA-Z0-9]*$");
  if (!std::regex_match(serial_port_, expr)) {
    RCLCPP_FATAL(this->get_logger(),
      "Device name: (%s) does not match convention /dev/name.", serial_port_.c_str());
    exit(1);
  }

  if (!std::ifstream(serial_port_.c_str()).good()) {
    RCLCPP_FATAL(this->get_logger(), "No device: %s.", serial_port_.c_str());
    exit(1);
  }


  std::stringstream latency_file_path_strem;
  size_t dev_name_idx = serial_port_.find_last_of('/'); 
  auto const dev_name = serial_port_.substr(dev_name_idx + 1);
  latency_file_path_strem << "/sys/bus/usb-serial/devices/" << dev_name << "/latency_timer";
  auto const latency_file_path = latency_file_path_strem.str();
  RCLCPP_DEBUG(this->get_logger(), "Changing FT232 latency timer at path '%s'.", latency_file_path.c_str());

  if (!std::ifstream(latency_file_path.c_str()).good()) {
    RCLCPP_FATAL(this->get_logger(), "No file: %s. Failed to change FT232 latency timer.", latency_file_path.c_str());
    exit(1);
  }

  std::ofstream latency_timer_file;
  try {
    latency_timer_file.open(latency_file_path.c_str(), std::ofstream::out | std::ofstream::trunc);
    latency_timer_file << 1;
    RCLCPP_DEBUG(this->get_logger(), "Succeeded to change FT232 latency timer at device '%s'.", serial_port_.c_str());
  }
  catch (const std::exception& e) {
    RCLCPP_FATAL(this->get_logger(), "Failed to change FT232 latency timer at device '%s'.", serial_port_.c_str());
    exit(1);
  }
  latency_timer_file.close();


  portHandler_ = dynamixel::PortHandler::getPortHandler(serial_port_.c_str());
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(1.0);

  if (!portHandler_->openPort()) {
    RCLCPP_FATAL(this->get_logger(), "Failed to open serial port at device '%s'.", serial_port_.c_str());
    exit(1);
  } else {
    RCLCPP_DEBUG(this->get_logger(), "Succeeded to open serial port at device '%s'.", serial_port_.c_str());
  }


  if (!portHandler_->setBaudRate(baudrate_)) {
    RCLCPP_FATAL(this->get_logger(), "Failed to set the baudrate!");
    exit(1);
  } else {
    RCLCPP_DEBUG(this->get_logger(), "Succeeded to set the baudrate.");
  }
}

void DynamixelMotorControllerNode::querry_joints() {
  for (auto const& [joint, motor] : motors_) {
    uint8_t data[STATE_JOINT_NUM_BUTES];
    packetHandler_->readTxRx(portHandler_, motor->id, STATE_POSITION_ADDRESS,
      STATE_JOINT_NUM_BUTES, data, &motor->error_code);

    if (motor->error_code & ERROR_BIT_CHECKSUM) {
      motor->error_cnt++;
      motor->last_error_time = std::chrono::system_clock::now();
      RCLCPP_WARN(this->get_logger(), "Bus error 0x%02X for motor with id: %u!", motor->error_code, motor->id);
      if (motor->error_cnt > bus_error_count_) {
        RCLCPP_FATAL(this->get_logger(),
          "Number of errors for motor with id: %u exceeded limit!", motor->id);
      }
    }
    else {
      motor->state.position = data_to_pos(&data[0]);
      motor->state.velocity = data_to_vel(&data[2]);
      if (motor->enabled) {
        motor->state.effort = data_to_eff(&data[4]);
      }
      else {
        motor->state.effort = 0.0f;
      }
    }
  }
}

void DynamixelMotorControllerNode::querry_state() {
  for (auto const& [joint, motor] : motors_) {
    uint8_t data[STATE_PARAMS_NUM_BUTES];
    packetHandler_->regWriteTxRx(portHandler_, motor->id, STATE_VOLTAGE_ADDRESS,
      STATE_PARAMS_NUM_BUTES, data, &motor->error_code);

    if (motor->error_code & ERROR_BIT_CHECKSUM) {
      motor->error_cnt++;
      motor->last_error_time = std::chrono::system_clock::now();
      RCLCPP_WARN(this->get_logger(), "Bus error for motor with id: %u!", motor->id);
      if (motor->error_cnt > bus_error_count_) {
        RCLCPP_FATAL(this->get_logger(),
          "Number of errors for motor with id: %u exceeded limit!", motor->id);
      }
    }
    else {
      motor->voltage = float(data[0]) / 10.0f;
      motor->temperature = float(data[1]);
    }
  }
}


void DynamixelMotorControllerNode::torque_enable(bool state) {
  bool suceeded = true;
  std::string state_str = (suceeded ? "enable" : "disbale");
  for (auto const& [joint, motor] : motors_) {
    uint8_t data = uint8_t(state);
    packetHandler_->writeTxRx(portHandler_, motor->id, TORQUE_ENABLE_ADDRESS,
      TORQUE_ENABLE_NUM_BUTES, &data, &motor->error_code);

    if (motor->error_code & ERROR_BIT_CHECKSUM) {
      suceeded = false;
      motor->error_cnt++;
      motor->last_error_time = std::chrono::system_clock::now();
      RCLCPP_WARN(this->get_logger(), "Bus error for motor with id: %u!", motor->id);
    }
    else {
      motor->enabled = state;
    }
  }
  if (suceeded) {
    RCLCPP_INFO(this->get_logger(), "Motors %sd.", state_str.c_str());
  }
  else {
    RCLCPP_WARN(this->get_logger(), "Failed to %s motors!", state_str.c_str());
  }
}


void DynamixelMotorControllerNode::check_bus_errors() {
  auto now = std::chrono::system_clock::now();
  for (auto const& [joint, motor] : motors_) {
    if (motor->error_code) {
      auto diff = std::chrono::duration_cast<std::chrono::seconds>(
        now - motor->last_error_time).count();
      if (motor->error_cnt > bus_error_count_) {
        RCLCPP_FATAL(this->get_logger(),
          "Number of errors for motor with id: %u exceeded limit!", motor->id);
        e_stop_ = true;
      }
      else if (diff > error_timeout_) {
        motor->error_cnt = 0;
        motor->last_error_time = std::chrono::system_clock::now();
      }

      if (motor->error_code & ERROR_BIT_VOLTAGE) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          RCUTILS_S_TO_NS(5.0f),
          "Undervoltage at motor with id: %u.",
          motor->id);
      }
      if (motor->error_code & ERROR_BIT_ANGLE) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          RCUTILS_S_TO_NS(2.0f),
          "Position out of range at motor with id: %u.",
          motor->id);
      }
      if (motor->error_code & ERROR_BIT_OVERHEAT) {
        RCLCPP_ERROR_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          RCUTILS_S_TO_NS(2.0f),
          "Motor with id: %u overheated.",
          motor->id);
        e_stop_ = true;
      }
      if (motor->error_code & ERROR_BIT_OVERLOAD) {
        RCLCPP_FATAL_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          RCUTILS_S_TO_NS(2.0f),
          "Motor with id: %u overloaded.",
          motor->id);
        e_stop_ = true;
      }
    }
  }
}


// void DynamixelMotorControllerNode::setup_motors() {
//   for (auto const& [joint, motor] : motors_) {
//     uint8_t data[STATE_PARAMS_NUM_BUTES];
    // packetHandler_->regWriteTxRx(portHandler_, motor->id, STATE_VOLTAGE_ADDRESS,
    //   STATE_PARAMS_NUM_BUTES, packetHandler_, &motor->error_code);

//     if (motor->error_code) {
//       motor->error_cnt++;
//       motor->last_error_time = *this->get_clock()->now();
//       RCLCPP_WARN(this->get_logger(), "Bus error for motor with id: %u!", motor->id);
//       if (motor->error_cnt > bus_error_count_) {
//         RCLCPP_FATAL(this->get_logger(),
//           "Number of errors for motor with id: %u exceeded limit!", motor->id);
//       }
//     }
//     else {
//       motor->voltage = float(data[0]) / 10.0f;
//       motor->temperature = float(data[1]);
//     }
//   }
// }


float DynamixelMotorControllerNode::data_to_pos(uint8_t *data) {
  int16_t raw_pos = (int16_t(data[1]) << 8) + int16_t(data[0]);
  // raw position to deg
  float pos_deg = float(raw_pos) / 1023.0f * 300.0f - 150.0f;
  // return position in rad
  return pos_deg / 360.0f * M_PI;
}

float DynamixelMotorControllerNode::data_to_vel(uint8_t *data) {
  int16_t raw_vel = (uint16_t(data[1]) << 8) + uint16_t(data[0]);
  if (raw_vel >= 1023) {
    raw_vel = -1 * (raw_vel - 1024);
  }
  // raw velocity to RPM
  float velocity = float(raw_vel) * 0.111f;
  // return in rad/s
  return velocity * (M_PI * 2.0f) / 60.0f;
}

float DynamixelMotorControllerNode::data_to_eff(uint8_t *data) {
  int16_t raw_eff = (int16_t(data[1]) << 8) + int16_t(data[0]);
  if (raw_eff >= 1023) {
    raw_eff = -1 * (raw_eff - 1024);
  }
  // normalise raw effort
  float effort = float(raw_eff) / 1023.0f;
  // return in nm
  return effort * 1.5;
}


void DynamixelMotorControllerNode::timer_callback() {
  querry_joints();
  publish_joint_state();
  publish_controller_state();
  check_bus_errors();
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamixelMotorControllerNode>());
  rclcpp::shutdown();
  return 0;
}
