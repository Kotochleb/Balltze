#ifndef MESSAGE_REALY_HPP
#define MESSAGE_REALY_HPP

#include <map>

#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

class MessageRelay: public rclcpp::Node
{
public:
  MessageRelay();

private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscriber_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_subscriber_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_publisher_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_publisher_;

  std::vector<std::string> joint_names_;
  std::map<std::string, float> state_offset_;
  std::map<std::string, float> command_offset_;

  void joint_states_cb(const sensor_msgs::msg::JointState::SharedPtr msg);
  void joint_trajecotry_cb(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
};

#endif // MESSAGE_REALY_HPP