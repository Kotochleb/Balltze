#include "balltze_msgs/message_relay.hpp"

#include <map>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

MessageRelay::MessageRelay(): Node("message_relay_node") {    
    this->declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>());
    this->get_parameter<std::vector<std::string>>("joints", joint_names_);

    for (auto & joint_name : joint_names_) {
      float command_offset = 0.0;
      float state_offset = 0.0;

      const std::string command_param_name = "offset.command." + joint_name;
      this->declare_parameter<float>(command_param_name, 0.0);
      this->get_parameter<float>(command_param_name, command_offset);
      command_offset_.insert({joint_name, command_offset});

      const std::string state_param_name = "offset.state." + joint_name;
      this->declare_parameter<float>(state_param_name, 0.0);
      this->get_parameter<float>(state_param_name, state_offset);
      state_offset_.insert({joint_name, state_offset});

      RCLCPP_INFO(this->get_logger(),
        "\nRegistered joint:\n\t\tName: \"%s\"\n\t\tCommand offset %.2f\n\t\tState offset %.2f",
        joint_name.c_str(), command_offset, state_offset);
    }

    joint_states_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, std::bind(&MessageRelay::joint_states_cb, this,  std::placeholders::_1));

    joint_trajectory_subscriber_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        "/joint_trajectory", 10, std::bind(&MessageRelay::joint_trajecotry_cb, this,  std::placeholders::_1));

    joint_states_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states/offset", 1);
    joint_trajectory_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "joint_trajectory/offset", 1);

    RCLCPP_INFO(this->get_logger(), "Node initialized!");

}

void MessageRelay::joint_states_cb(const sensor_msgs::msg::JointState::SharedPtr msg) {
  sensor_msgs::msg::JointState out_msg;
  out_msg.header = msg->header;
  out_msg.name = msg->name;
  out_msg.position = msg->position;
  out_msg.velocity = msg->velocity;
  out_msg.effort = msg->effort;

  for (std::size_t i = 0; i < msg->name.size(); i++) {
    if (state_offset_.find(msg->name[i]) != state_offset_.end()) {
      out_msg.position[i] += state_offset_.at(msg->name[i]);
      if (out_msg.position[i] > M_PI) {
        out_msg.position[i] -= M_PI;
      }
      else if (out_msg.position[i] < -M_PI) {
        out_msg.position[i] += M_PI;
      }
    }
  }
  joint_states_publisher_->publish(out_msg);
}


void MessageRelay::joint_trajecotry_cb(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
  trajectory_msgs::msg::JointTrajectory out_msg;
  out_msg.header = msg->header;
  out_msg.joint_names = msg->joint_names;
  out_msg.points = msg->points;

  for (std::size_t i = 0; i < msg->joint_names.size(); i++) {
    if (command_offset_.find(msg->joint_names[i]) != command_offset_.end()) {
      out_msg.points[0].positions[i] += command_offset_.at(msg->joint_names[i]);
      if (out_msg.points[0].positions[i] > M_PI) {
        out_msg.points[0].positions[i] -= M_PI;
      }
      else if (out_msg.points[0].positions[i] < -M_PI) {
        out_msg.points[0].positions[i] += M_PI;
      }
    }
  }
  joint_trajectory_publisher_->publish(out_msg);
}
