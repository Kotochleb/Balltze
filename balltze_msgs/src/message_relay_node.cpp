#include "balltze_msgs/message_relay.hpp"

int main(int argc, char** argv )
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MessageRelay>());
  rclcpp::shutdown();
  return 0;
}