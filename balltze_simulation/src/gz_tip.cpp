#include "balltze_simulation/gz_tip.hpp"

#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include <ignition/transport.hh>
#include <ignition/msgs/time.pb.h>
#include <ignition/msgs/Utility.hh>
#include <ignition/msgs/contacts.pb.h>

namespace gz_tip {

using namespace std::placeholders;

void GZTip::subscribe() {
  if (!node_->Subscribe<ignition::msgs::Contacts>(topic_, callback_, opts_)) {
    std::stringstream error_msg;
    error_msg << "Error subscribing to topic [" << topic_ << "]";
    throw std::runtime_error(error_msg.str());
  }
}

void GZTip::unsubscribe() {
  if (!node_->Unsubscribe(topic_)) {
    std::stringstream error_msg;
    error_msg << "Error unsubscribing to topic [" << topic_ << "]";
    throw std::runtime_error(error_msg.str());
  }
}

std::chrono::steady_clock::duration GZTip::get_contact_stamp() {
  return stamp_;
}

void GZTip::contact_cb(const ignition::msgs::Contacts &msg) {
  stamp_ = ignition::msgs::Convert(msg.header().stamp());
}

}