#ifndef AX_12_A_MOTOR_
#define AX_12_A_MOTOR_

#include <memory>
#include <vector>
#include <cmath>

#include "dynamixel_sdk/dynamixel_sdk.h"

namespace ax_12_a_motor {

struct ErrorReport {
  bool error_occured = false;
  bool voltage = false;
  bool angle = false;
  bool overheat = false;
  bool range = false;
  bool checksum = false;
  bool overload = false;
  bool instruction = false;
  uint8_t count = 0;
};

class AX12AMotor {
public:
  AX12AMotor(
    const uint8_t id,
    const std::shared_ptr<dynamixel::PortHandler> portHandler,
    const std::shared_ptr<dynamixel::PacketHandler> packetHandler) : 
    id_(id), portHandler_(portHandler), packetHandler_(packetHandler) {};
  bool ping();
  void qeurry_joint();
  void query_stats();
  void enable_torque(bool state);
  void action() {packetHandler_->action(portHandler_.get(), id_);}
  void reboot() {packetHandler_->reboot(portHandler_.get(), id_);}
  void clear_error_cnt() {error_cnt_ = 0;}

  void set_led(bool state);
  void set_return_dealy(uint8_t return_delay);
  void set_state(const float pos, const float vel, const float eff);

  bool get_motor_enabled() {return enabled_;};
  float get_position() {return position_;}
  float get_velocity() {return velocity_;}
  float get_effort() {return effort_;}
  float get_voltage() {return voltage_;}
  float get_temperature() {return temperature_;}
  std::shared_ptr<ErrorReport> get_motor_error_report();

  
private:
  float data_to_pos(uint8_t *data);
  float data_to_vel(uint8_t *data);
  float data_to_eff(uint8_t *data);
  void pos_to_data(float val, uint8_t *data);
  void vel_to_data(float val, uint8_t *data);
  void eff_to_data(float val, uint8_t *data);
  bool bus_error_occured() {return error_code_ & ERROR_BIT_CHECKSUM_;}

  const uint8_t id_;
  const std::shared_ptr<dynamixel::PortHandler> portHandler_;
  const std::shared_ptr<dynamixel::PacketHandler> packetHandler_;


  uint8_t error_code_;
  uint8_t error_cnt_;
  float position_;
  float velocity_;
  float effort_;
  float voltage_;
  float temperature_;
  bool enabled_ = false;
  
  static constexpr float MOTOR_ANGLE_RANGE_ {300.0f / 360.0f * (2 * M_PI)};
  static constexpr float MOTOR_MAX_VELOCITY_ {114.0f * (2.0f * M_PI / 60.0f)};
  static constexpr float MOTOR_MAX_EFFORT_ {1.5f};

  static constexpr uint8_t GOAL_POSITION_ADDRES_ {30};
  static constexpr uint8_t GOAL_VELOCITY_ADDRES_ {32};
  static constexpr uint8_t GOAL_EFFORT_ADDRES_ {34};
  static constexpr uint8_t GOAL_NUM_BUTES_ {6};

  static constexpr uint8_t STATE_POSITION_ADDRESS_ {36};
  static constexpr uint8_t STATE_VELOCITY_ADDRESS_ {38};
  static constexpr uint8_t STATE_EFFORT_ADDRESS_ {40};
  static constexpr uint8_t STATE_NUM_BUTES_ {6};
  static constexpr uint8_t INFO_VOLTAGE_ADDRESS_ {42};
  static constexpr uint8_t INFO_TEMPERATURE_ADDRESS_ {43};
  static constexpr uint8_t INFO_NUM_BUTES_ {2};

  static constexpr uint8_t TORQUE_ENABLE_ADDRESS_ {24};
  static constexpr uint8_t TORQUE_ENABLE_NUM_BUTES_ {1};

  static constexpr uint8_t LED_ADDRESS_ {25};
  static constexpr uint8_t LED_NUM_BUTES_ {1};

  static constexpr uint8_t RETURN_DELAY_ADDRESS_ {5};
  static constexpr uint8_t RETURN_DELAY_NUM_BUTES_ {1};

  static constexpr uint8_t ERROR_BIT_VOLTAGE_ {1};
  static constexpr uint8_t ERROR_BIT_ANGLE_ {2};
  static constexpr uint8_t ERROR_BIT_OVERHEAT_ {4};
  static constexpr uint8_t ERROR_BIT_RANGE_ {8};
  static constexpr uint8_t ERROR_BIT_CHECKSUM_ {16};
  static constexpr uint8_t ERROR_BIT_OVERLOAD_ {32};
  static constexpr uint8_t ERROR_BIT_INSTRUCTION_ {64};
};

}


#endif // AX_12_A_MOTOR_