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

#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace std::chrono_literals;
using namespace dynamixel_motor_controller;
using namespace std::chrono_literals;

DynamixelMotorControllerNode::DynamixelMotorControllerNode() : 
  Node("dynamixel_motor_controller_node") {

  this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
  this->declare_parameter<int>("baudrate", 1000000);
  this->declare_parameter<int>("return_delay", 50);
  this->declare_parameter<int>("bus_error_count", 20);
  this->declare_parameter<float>("error_clear_timeout", 5.0);
  this->declare_parameter<std::string>("joint_state_msg_link", "base_link");
  this->declare_parameter<std::vector<std::string>>("joint_names");


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

    // motors_.insert(std::initializer_list<std::pair<std::string, Motor>(joint,motor));
    motors_.insert({joint, motor});
  }

  this->get_parameter("serial_port", serial_port_);
  this->get_parameter("baudrate", baudrate_);
  this->get_parameter("return_delay", return_delay_);

  this->get_parameter("bus_error_count", bus_error_count_);
  this->get_parameter("error_clear_timeout", error_clear_timeout_);
  this->get_parameter("joint_state_msg_link", joint_state_msg_link_);

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

  querry_state();
  querry_joints();


  joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_state", 10);
  

  timer_ = this->create_wall_timer(
    50ms,
    std::bind(&DynamixelMotorControllerNode::timer_callback,
    this));

  RCLCPP_INFO(this->get_logger(), "Node initialised.");
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
      motor->state.effort = data_to_eff(&data[4]);
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


// void DynamixelMotorControllerNode::setup_motors() {
//   for (auto const& [joint, motor] : motors_) {
//     uint8_t data[STATE_PARAMS_NUM_BUTES];
//     packetHandler_->regWriteTxRx(portHandler_, motor->id, STATE_VOLTAGE_ADDRESS,
//       STATE_PARAMS_NUM_BUTES, packetHandler_, &motor->error_code);

//     if (motor->error_code) {
//       motor->error_cnt++;
//       motor->last_error_time = this->get_clock()->now();
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
  // raw velocity to RPM
  float velocity = float(raw_vel) * 0.111f;
  // return in rad/s
  return velocity * (M_PI * 2.0f) / 60.0f;
}

float DynamixelMotorControllerNode::data_to_eff(uint8_t *data) {
  int16_t raw_eff = (int16_t(data[1]) << 8) + int16_t(data[0]);
  // normalise raw effort
  float effort = float(raw_eff) / 1023.0f;
  // return in nm
  return effort * 1.5;
}


void DynamixelMotorControllerNode::timer_callback() {
  querry_joints();
  publish_joint_state();
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamixelMotorControllerNode>());
  rclcpp::shutdown();
  return 0;
}
