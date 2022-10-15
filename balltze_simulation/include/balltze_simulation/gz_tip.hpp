#ifndef GZ_TIP_
#define GZ_TIP_

#include <chrono>
#include <memory>
#include <string>

#include <ignition/transport.hh>
#include <ignition/msgs/clock.pb.h>
#include <ignition/msgs/contacts.pb.h>

namespace gz_tip {

using namespace std::placeholders;

class GZTip {
public:
  GZTip(const std::shared_ptr<ignition::transport::Node> node,
        const std::string topic, const unsigned int msg_per_sec) :
        node_(node), topic_(topic) {opts_.SetMsgsPerSec(msg_per_sec);};
  
  void subscribe();
  void unsubscribe();
  std::chrono::steady_clock::duration get_contact_stamp();

private:
  const std::shared_ptr<ignition::transport::Node> node_;
  const std::string topic_;
  ignition::transport::SubscribeOptions opts_;
  std::function<void(const ignition::msgs::Contacts &msg)> callback_ = std::bind(&GZTip::contact_cb, this, _1);
  std::chrono::steady_clock::duration stamp_ = std::chrono::steady_clock::duration::zero();
  void contact_cb(const ignition::msgs::Contacts &msg);
};

}


#endif // GZ_TIP_