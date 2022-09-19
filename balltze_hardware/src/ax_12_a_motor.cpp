#include "balltze_hardware/ax_12_a_motor.hpp"

#include <memory>
#include <vector>
#include <cfloat>
#include <limits>
#include <cmath>

#include "dynamixel_sdk/dynamixel_sdk.h"

namespace ax_12_a_motor {

bool AX12AMotor::ping() {
  int ping_resp = packetHandler_->ping(portHandler_.get(), id_, &error_code_);
  if (ping_resp) {
    return false;
  }
  return true;
}


void AX12AMotor::enable_torque(bool state) {
  uint8_t data = uint8_t(state);
  packetHandler_->writeTxRx(portHandler_.get(), id_, TORQUE_ENABLE_ADDRESS_,
    TORQUE_ENABLE_NUM_BUTES_, &data, &error_code_);

  if (bus_error_occured()) {
    error_cnt_++;
  }
  else {
    enabled_ = state;
  }
}


void AX12AMotor::set_led(bool state) {
  uint8_t data = uint8_t(state);
  packetHandler_->writeTxRx(portHandler_.get(), id_, LED_ADDRESS_,
    LED_NUM_BUTES_, &data, &error_code_);

  if (bus_error_occured()) {
    error_cnt_++;
  }
}


void AX12AMotor::set_return_dealy(uint8_t return_delay) {
  uint8_t data = uint8_t(return_delay);
  packetHandler_->writeTxRx(portHandler_.get(), id_, RETURN_DELAY_ADDRESS_,
    RETURN_DELAY_NUM_BUTES_, &data, &error_code_);

  if (bus_error_occured()) {
    error_cnt_++;
  }
}


std::shared_ptr<ErrorReport> AX12AMotor::get_motor_error_report() {
  auto error_report = std::make_shared<ErrorReport>();

  if (error_code_) {
    error_report->error_occured = true;
    if (error_code_ & ERROR_BIT_VOLTAGE_) {
      error_report->voltage = true;
    }
    if (error_code_ & ERROR_BIT_ANGLE_) {
      error_report->angle = true;
    }
    if (error_code_ & ERROR_BIT_OVERHEAT_) {
      error_report->overheat = true;
    }
    if (error_code_ & ERROR_BIT_RANGE_) {
      error_report->range = true;
    }
    if (error_code_ & ERROR_BIT_CHECKSUM_) {
      error_report->checksum = true;
    }
    if (error_code_ & ERROR_BIT_OVERLOAD_) {
      error_report->overload = true;
    }
    if (error_code_ & ERROR_BIT_INSTRUCTION_) {
      error_report->instruction = true;
    }
  }
  error_report->count = error_cnt_;
  return error_report;
}


float AX12AMotor::data_to_pos(uint8_t *data) {
  int16_t raw_pos = (int16_t(data[1]) << 8) + int16_t(data[0]);
  // raw position to deg
  float pos_deg = float(raw_pos) / 1023.0f * 300.0f - 150.0f;
  // return position in rad
  return -pos_deg / 180.0f * M_PI;
}


float AX12AMotor::data_to_vel(uint8_t *data) {
  int16_t raw_vel = (uint16_t(data[1]) << 8) + uint16_t(data[0]);
  if (raw_vel >= 1023) {
    raw_vel = -1 * (raw_vel - 1024);
  }
  // raw velocity to RPM
  float velocity = float(raw_vel) * 0.111f;
  // return in rad/s
  return -velocity * (M_PI * 2.0f) / 60.0f;
}


float AX12AMotor::data_to_eff(uint8_t *data) {
  int16_t raw_eff = (int16_t(data[1]) << 8) + int16_t(data[0]);
  if (raw_eff >= 1023) {
    raw_eff = -1 * (raw_eff - 1024);
  }
  // normalise raw effort
  float effort = float(raw_eff) / 1023.0f;
  // return in nm
  return -effort * 1.5;
}


void AX12AMotor::qeurry_joint() {  
  uint8_t data[STATE_NUM_BUTES_];
  packetHandler_->readTxRx(portHandler_.get(), id_, STATE_POSITION_ADDRESS_,
    STATE_NUM_BUTES_, data, &error_code_);

  if (bus_error_occured()) {
    error_cnt_++;
    position_ = std::numeric_limits<double>::quiet_NaN();
    velocity_ = std::numeric_limits<double>::quiet_NaN();
    effort_ = std::numeric_limits<double>::quiet_NaN();
  }
  else {
    position_ = data_to_pos(&data[0]);
    velocity_ = data_to_vel(&data[2]);
    if (enabled_) {
      effort_ = data_to_eff(&data[4]);
    }
    else {
      effort_ = 0.0f;
    }
  }
}


void AX12AMotor::pos_to_data(float val, uint8_t *data) {
  // offset center of rotation by half of the available rotation
  float offset_pos = -val + (MOTOR_ANGLE_RANGE_ / 2.0f);
  float ang_normalised = offset_pos / MOTOR_ANGLE_RANGE_;
  int16_t raw_data = int16_t(ang_normalised * 1023.0f);

  data[0] = raw_data & 0x00FF;
  data[1] = (raw_data & 0xFF00) >> 8;
}


void AX12AMotor::vel_to_data(float val, uint8_t *data) {
  // stop motor istaed of setting max speed if value it close to 0.0 rad/s
  if (std::isnan(val)) {
    data[0] = 1;
    data[1] = 0;
    return;
  }

  // prevent getting close to integer equal to 0 setting motor to full speed
  if (std::fabs(val) < 0.1) {
    val = 0.1;
  }
  // offset center of rotation by half of the available rotation
  float normalised_vel = std::fabs(val) / MOTOR_MAX_VELOCITY_;
  int16_t raw_data = int16_t(normalised_vel * 1023.0f);

  data[0] = raw_data & 0x00FF;
  data[1] = (raw_data & 0xFF00) >> 8;
}


void AX12AMotor::eff_to_data(float val, uint8_t *data) {
  if (std::isnan(val)) {
    data[0] = 1;
    data[1] = 0;
    return;
  }

  // offset center of rotation by half of the available rotation
  float eff_normalised = std::fabs(val) / MOTOR_MAX_EFFORT_;
  int16_t raw_data = int16_t(eff_normalised * 1023.0f);

  data[0] = raw_data & 0x00FF;
  data[1] = (raw_data & 0xFF00) >> 8;
}


void AX12AMotor::set_state(float pos, float vel, float eff) {
  uint8_t data[GOAL_NUM_BUTES_];

  pos_to_data(pos, &data[0]);
  vel_to_data(vel, &data[2]);
  eff_to_data(eff, &data[4]);

  packetHandler_->regWriteTxRx(portHandler_.get(), id_, GOAL_POSITION_ADDRES_,
      GOAL_NUM_BUTES_, data, &error_code_);
  if (bus_error_occured()) {
    error_cnt_++;
  }
}


void AX12AMotor::query_stats() {
  uint8_t data[INFO_NUM_BUTES_];
  packetHandler_->readTxRx(portHandler_.get(), id_, INFO_VOLTAGE_ADDRESS_,
    INFO_NUM_BUTES_, data, &error_code_);

  if (bus_error_occured()) {
    error_cnt_++;
  }
  else {
    voltage_ = float(data[0]) / 10.0f;
    temperature_ = float(data[1]);
  }
}

}