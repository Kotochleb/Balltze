#include <chrono>
#include <iostream>
#include <string>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>


#include <ignition/msgs/contact.pb.h>
#include <ignition/msgs/contacts.pb.h>

#include "balltze_simulation/gz_tip.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  auto node = std::make_shared<ignition::transport::Node>();

  gz_tip::GZTip tip_fl(node, "/balltze/fl_tip/contact", 20);
  gz_tip::GZTip tip_rl(node, "/balltze/rl_tip/contact", 20);

  tip_fl.subscribe();
  std::cout << "started subscribing" << std::endl;

  ignition::transport::NetworkClock clock("/world/empty/clock", ignition::transport::NetworkClock::TimeBase::SIM);

  while (true) {
    std::cout << std::chrono::duration_cast<std::chrono::nanoseconds>
      (tip_fl.get_contact_stamp()).count() << std::endl;

    auto wall_time = clock.Time();
    auto last_fl_tip = tip_fl.get_contact_stamp();
    // auto last_rl_tip = tip_rl.get_contact_stamp();
    if ((wall_time - last_fl_tip) > std::chrono::duration_cast<std::chrono::nanoseconds>(200ms)) {
      std::cout << "not touching" << std::endl;
    }
    else {
      std::cout << "touching touching" << std::endl;
    }
    std::this_thread::sleep_for(100ms);
  }
  return 0;
}