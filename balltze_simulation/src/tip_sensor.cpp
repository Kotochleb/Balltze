#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "champ_msgs/ContactsStamped.h"

#include "balltze_simulation/gz_tip.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class TipSensor : public rclcpp::Node
{
  public:
    TipSensor() : Node("minimal_publisher"), count_(0) {
      auto gz_node_ = std::make_shared<ignition::transport::Node>();
      for (auto & name : tip_names_) {
        tips_.insert({name, std::make_unique<gz_tip::GZTip>(gz_node_, "/balltze/" + name +"_tip/contact", 30)});
      }

      publisher_ = this->create_publisher<champ_msgs::msg::ContactsStamped>("foot_contacts", 20);
      timer_ = this->create_wall_timer(500ms, std::bind(&TipSensor::timer_callback, this));
    }

  private:
    void timer_callback() {
      auto message = champ_msgs::msg::ContactsStamped();

      message

      publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<champ_msgs::msg::ContactsStamped>::SharedPtr publisher_;
    std::map<std::string, std::unique_ptr<gz_tip::GZTip>> tips_;
    std::vector<std::string> tip_names_ = {"fl, fr, rl, rr"};
    std::shared_ptr<ignition::transport::Node> gz_node_
    size_t count_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TipSensor>());
  rclcpp::shutdown();
  return 0;
}