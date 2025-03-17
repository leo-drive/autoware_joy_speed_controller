// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/joy_controller/joy_controller.hpp"
#include "autoware/joy_controller/joy_converter/ds4_joy_converter.hpp"
#include "autoware/joy_controller/joy_converter/g29_joy_converter.hpp"
#include "autoware/joy_controller/joy_converter/p65_joy_converter.hpp"
#include "autoware/joy_controller/joy_converter/xbox_joy_converter.hpp"

#include <tier4_api_utils/tier4_api_utils.hpp>

#include <algorithm>
#include <memory>
#include <string>

namespace {
using autoware::joy_controller::GateModeType;
using autoware::joy_controller::GearShiftType;
using autoware::joy_controller::TurnSignalType;
using GearShift = tier4_external_api_msgs::msg::GearShift;
using TurnSignal = tier4_external_api_msgs::msg::TurnSignal;
using GateMode = tier4_control_msgs::msg::GateMode;

GearShiftType getUpperShift(const GearShiftType &shift) {
  if (shift == GearShift::NONE) {
    return GearShift::PARKING;
  }
  if (shift == GearShift::PARKING) {
    return GearShift::REVERSE;
  }
  if (shift == GearShift::REVERSE) {
    return GearShift::NEUTRAL;
  }
  if (shift == GearShift::NEUTRAL) {
    return GearShift::DRIVE;
  }
  if (shift == GearShift::DRIVE) {
    return GearShift::LOW;
  }
  if (shift == GearShift::LOW) {
    return GearShift::LOW;
  }

  return GearShift::NONE;
}

GearShiftType getLowerShift(const GearShiftType &shift) {
  if (shift == GearShift::NONE) {
    return GearShift::PARKING;
  }
  if (shift == GearShift::PARKING) {
    return GearShift::PARKING;
  }
  if (shift == GearShift::REVERSE) {
    return GearShift::PARKING;
  }
  if (shift == GearShift::NEUTRAL) {
    return GearShift::REVERSE;
  }
  if (shift == GearShift::DRIVE) {
    return GearShift::NEUTRAL;
  }
  if (shift == GearShift::LOW) {
    return GearShift::DRIVE;
  }

  return GearShift::NONE;
}

const char *getShiftName(const GearShiftType &shift) {
  if (shift == GearShift::NONE) {
    return "NONE";
  }
  if (shift == GearShift::PARKING) {
    return "PARKING";
  }
  if (shift == GearShift::REVERSE) {
    return "REVERSE";
  }
  if (shift == GearShift::NEUTRAL) {
    return "NEUTRAL";
  }
  if (shift == GearShift::DRIVE) {
    return "DRIVE";
  }
  if (shift == GearShift::LOW) {
    return "LOW";
  }

  return "NOT_SUPPORTED";
}

const char *getTurnSignalName(const TurnSignalType &turn_signal) {
  if (turn_signal == TurnSignal::NONE) {
    return "NONE";
  }
  if (turn_signal == TurnSignal::LEFT) {
    return "LEFT";
  }
  if (turn_signal == TurnSignal::RIGHT) {
    return "RIGHT";
  }
  if (turn_signal == TurnSignal::HAZARD) {
    return "HAZARD";
  }

  return "NOT_SUPPORTED";
}

const char *getGateModeName(const GateModeType &gate_mode) {
  using tier4_control_msgs::msg::GateMode;

  if (gate_mode == GateMode::AUTO) {
    return "AUTO";
  }
  if (gate_mode == GateMode::EXTERNAL) {
    return "EXTERNAL";
  }

  return "NOT_SUPPORTED";
}

} // namespace

namespace autoware::joy_controller {
void AutowareJoyControllerNode::onJoy() {
  const auto msg = sub_joy_.takeData();
  if (!msg) {
    return;
  }

  last_joy_received_time_ = msg->header.stamp;
  if (joy_type_ == "G29") {
    joy_ = std::make_shared<const G29JoyConverter>(*msg);
  } else if (joy_type_ == "DS4") {
    joy_ = std::make_shared<const DS4JoyConverter>(*msg);
  } else if (joy_type_ == "XBOX") {
    joy_ = std::make_shared<const XBOXJoyConverter>(*msg);
  } else {
    joy_ = std::make_shared<const P65JoyConverter>(*msg);
  }

  if (joy_->shift_up() || joy_->shift_down() || joy_->shift_drive() ||
      joy_->shift_reverse()) {
    publishShift();
  }

  if (joy_->turn_signal_left() || joy_->turn_signal_right() ||
      joy_->clear_turn_signal()) {
    publishTurnSignal();
  }

  if (joy_->gate_mode()) {
    publishGateMode();
  }

  if (joy_->autoware_engage() || joy_->autoware_disengage()) {
    publishAutowareEngage();
  }

  if (joy_->vehicle_engage() || joy_->vehicle_disengage()) {
    publishVehicleEngage();
  }

  if (joy_->emergency_stop()) {
    sendEmergencyRequest(true);
  }

  if (joy_->clear_emergency_stop()) {
    sendEmergencyRequest(false);
  }
}

bool AutowareJoyControllerNode::isDataReady() {
  // Joy
  {
    if (!joy_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(),
                           std::chrono::milliseconds(5000).count(),
                           "waiting for joy msg...");
      return false;
    }

    constexpr auto timeout = 2.0;
    const auto time_diff = this->now() - last_joy_received_time_;
    if (time_diff.seconds() > timeout) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(),
                           std::chrono::milliseconds(5000).count(),
                           "joy msg is timeout");
      return false;
    }
  }
  return true;
}

void AutowareJoyControllerNode::onTimer() {
  onJoy();

  if (!isDataReady()) {
    return;
  }

  publishControlCommand();
  publishHeartbeat();
}

float AutowareJoyControllerNode::exponentialSmooth(float currentVelocity,
                                                   float targetVelocity,
                                                   float accelSmoothingFactor,
                                                   float decelSmoothingFactor) {
  // Check if the current velocity is higher than the target velocity
  // (deceleration case)
  if (currentVelocity > targetVelocity) {
    // Apply exponential smoothing with deceleration smoothing factor
    float newVelocity =
        currentVelocity + (targetVelocity - currentVelocity) *
                              (1 - std::exp(-2 * decelSmoothingFactor));
    // Clamp the velocity to ensure it doesn't go below 0 and ensure small
    // velocities are set to 0
    return (std::abs(newVelocity) < 0.01f) ? 0.0f : newVelocity;
  }
  // If the velocity is lower than the target, we just apply regular smoothing
  // (acceleration case)
  else {
    float newVelocity =
        currentVelocity + (targetVelocity - currentVelocity) *
                              (1 - std::exp(-accelSmoothingFactor));
    // Clamp the velocity to ensure it doesn't go below 0 and ensure small
    // velocities are set to 0
    return (std::abs(newVelocity) < 0.01f) ? 0.0f : newVelocity;
  }
}

void AutowareJoyControllerNode::publishControlCommand() {

  cmd.stamp = this->now();
  {

    target_steering_angle_ = steer_ratio_ * joy_->steer();
    if (std::abs(target_steering_angle_ - set_steering_angle_) < steer_rate_) {
      set_steering_angle_ = target_steering_angle_;
    } else if (set_steering_angle_ < target_steering_angle_) {
      set_steering_angle_ += steer_rate_;
    } else if (set_steering_angle_ > target_steering_angle_) {
      set_steering_angle_ -= steer_rate_;
    }
    cmd.lateral.steering_tire_angle = set_steering_angle_;
    cmd.lateral.steering_tire_rotation_rate = steering_angle_velocity_;

    if (joy_->accel()) {
      target_velocity_ += velocity_gain_;
      target_velocity_ = std::min(target_velocity_, (max_velocity_));
    }

    if (joy_->brake()) {
      target_velocity_ -= velocity_gain_;
      target_velocity_ = std::max(target_velocity_, (0.0));
    }
  }

  {

    cmd.longitudinal.velocity = exponentialSmooth(
        std::fabs(cmd.longitudinal.velocity), target_velocity_,
        accel_smoothing_factor_, decel_smoothing_factor_);

    if (gear_shift.gear_shift.data == GearShift::REVERSE) {
      cmd.longitudinal.velocity *= -1.0f;
    }
  }

  if (emergency_request_.emergency) {
    cmd.longitudinal.velocity = 0;
    cmd.longitudinal.acceleration = 0;
    target_velocity_ = 0;
  }
  pub_control_command_->publish(cmd);
}

void AutowareJoyControllerNode::publishShift() {
  autoware_vehicle_msgs::msg::GearCommand gear_cmd_;

  gear_shift.stamp = this->now();

  if (joy_->shift_up()) {
    gear_shift.gear_shift.data = getUpperShift(prev_shift_);
  }

  if (joy_->shift_down()) {
    gear_shift.gear_shift.data = getLowerShift(prev_shift_);
  }

  if (joy_->shift_drive()) {
    gear_shift.gear_shift.data = GearShift::DRIVE;
  }

  if (joy_->shift_reverse()) {
    gear_shift.gear_shift.data = GearShift::REVERSE;
  }

  RCLCPP_INFO(get_logger(), "GearShift::%s",
              getShiftName(gear_shift.gear_shift.data));

  switch (gear_shift.gear_shift.data) {
  case GearShift::PARKING:
    gear_cmd_.command = autoware_vehicle_msgs::msg::GearCommand::PARK;
    break;
  case GearShift::REVERSE:
    gear_cmd_.command = autoware_vehicle_msgs::msg::GearCommand::REVERSE;
    break;
  case GearShift::NEUTRAL:
    gear_cmd_.command = autoware_vehicle_msgs::msg::GearCommand::NEUTRAL;
    break;
  case GearShift::DRIVE:
    gear_cmd_.command = autoware_vehicle_msgs::msg::GearCommand::DRIVE;
    break;
  case GearShift::LOW:
    gear_cmd_.command = autoware_vehicle_msgs::msg::GearCommand::DRIVE;
    break;
  }
  gear_cmd_.stamp = this->now();
  pub_shift_->publish(gear_cmd_);
  prev_shift_ = gear_shift.gear_shift.data;
}

void AutowareJoyControllerNode::publishTurnSignal() {
  tier4_external_api_msgs::msg::TurnSignalStamped turn_signal;
  autoware_vehicle_msgs::msg::TurnIndicatorsCommand turn_indicator_msg;
  autoware_vehicle_msgs::msg::HazardLightsCommand hazard_signal_msg;

  turn_signal.stamp = this->now();

  turn_indicator_msg.stamp = this->now();
  hazard_signal_msg.stamp = this->now();

  if (joy_->turn_signal_left() && joy_->turn_signal_right()) {
    turn_signal.turn_signal.data = TurnSignal::HAZARD;
  } else if (joy_->turn_signal_left()) {
    turn_signal.turn_signal.data = TurnSignal::LEFT;
  } else if (joy_->turn_signal_right()) {
    turn_signal.turn_signal.data = TurnSignal::RIGHT;
  }

  if (joy_->clear_turn_signal()) {
    turn_signal.turn_signal.data = TurnSignal::NONE;
  }

  RCLCPP_INFO(get_logger(), "TurnSignal::%s",
              getTurnSignalName(turn_signal.turn_signal.data));

  if (turn_signal.turn_signal.data == TurnSignal::HAZARD) {
    hazard_signal_msg.command =
        autoware_vehicle_msgs::msg::HazardLightsCommand::ENABLE;
  } else {
    hazard_signal_msg.command =
        autoware_vehicle_msgs::msg::HazardLightsCommand::DISABLE;
  }

  if (turn_signal.turn_signal.data == TurnSignal::LEFT) {
    turn_indicator_msg.command =
        autoware_vehicle_msgs::msg::TurnIndicatorsCommand::ENABLE_LEFT;
  } else if (turn_signal.turn_signal.data == TurnSignal::RIGHT) {
    turn_indicator_msg.command =
        autoware_vehicle_msgs::msg::TurnIndicatorsCommand::ENABLE_RIGHT;
  } else {
    turn_indicator_msg.command =
        autoware_vehicle_msgs::msg::TurnIndicatorsCommand::DISABLE;
  }

  pub_hazard_signal_->publish(hazard_signal_msg);
  pub_turn_indicator_signal_->publish(turn_indicator_msg);
}

void AutowareJoyControllerNode::publishGateMode() {
  tier4_control_msgs::msg::GateMode gate_mode;

  if (prev_gate_mode_ == GateMode::AUTO) {
    gate_mode.data = GateMode::EXTERNAL;
  }

  if (prev_gate_mode_ == GateMode::EXTERNAL) {
    gate_mode.data = GateMode::AUTO;
  }

  RCLCPP_INFO(get_logger(), "GateMode::%s", getGateModeName(gate_mode.data));

  pub_gate_mode_->publish(gate_mode);
  prev_gate_mode_ = gate_mode.data;
}

void AutowareJoyControllerNode::publishHeartbeat() {
  tier4_external_api_msgs::msg::Heartbeat heartbeat;
  heartbeat.stamp = this->now();
  pub_heartbeat_->publish(heartbeat);
}

void AutowareJoyControllerNode::sendEmergencyRequest(bool emergency) {
  RCLCPP_INFO(get_logger(), "%s emergency stop",
              emergency_request_.emergency ? "Set" : "Clear");
  (void)emergency;
  emergency_request_.stamp = this->now();
  emergency_request_.emergency = !emergency_request_.emergency;
  pub_vehicle_emergency_->publish(emergency_request_);
}

void AutowareJoyControllerNode::publishAutowareEngage() {

  vehicle_engage_request_.stamp = this->now();
  vehicle_engage_request_.engage = !vehicle_engage_request_.engage;

  RCLCPP_INFO(get_logger(), "Autoware Engage: %s",
              vehicle_engage_request_.engage ? "true" : "false");
  pub_autoware_engage_->publish(vehicle_engage_request_);
}

void AutowareJoyControllerNode::publishVehicleEngage() {
  autoware_vehicle_msgs::msg::Engage engage;

  if (joy_->vehicle_engage()) {
    engage.engage = true;
    RCLCPP_INFO(get_logger(), "Vehicle Engage");
  }

  if (joy_->vehicle_disengage()) {
    engage.engage = false;
    RCLCPP_INFO(get_logger(), "Vehicle Disengage");
  }

  pub_vehicle_engage_->publish(engage);
}

void AutowareJoyControllerNode::initTimer(double period_s) {
  const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(period_s));
  timer_ = rclcpp::create_timer(
      this, get_clock(), period_ns,
      std::bind(&AutowareJoyControllerNode::onTimer, this));
}

AutowareJoyControllerNode::AutowareJoyControllerNode(
    const rclcpp::NodeOptions &node_options)
    : Node("autoware_joy_controller", node_options) {
  // Parameter
  joy_type_ = declare_parameter<std::string>("joy_type");
  update_rate_ = declare_parameter<double>("update_rate");
  steer_ratio_ = declare_parameter<double>("steer_ratio");
  steer_rate_ = declare_parameter<double>("steer_rate");
  steering_angle_velocity_ =
      declare_parameter<double>("steering_angle_velocity");
  velocity_gain_ = declare_parameter<double>("control_command.velocity_gain");

  max_velocity_ = declare_parameter<double>("control_command.max_velocity");
  accel_smoothing_factor_ =
      declare_parameter<double>("control_command.accel_smooth_factor");
  decel_smoothing_factor_ =
      declare_parameter<double>("control_command.decel_smooth_factor");

  RCLCPP_INFO(get_logger(), "Joy type: %s", joy_type_.c_str());

  // Callback Groups
  callback_group_services_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Publisher
  pub_control_command_ =
      this->create_publisher<autoware_control_msgs::msg::Control>(
          "/control/command/control_cmd", 1);
  pub_shift_ = this->create_publisher<autoware_vehicle_msgs::msg::GearCommand>(
      "/control/command/gear_cmd", 1);
  pub_turn_indicator_signal_ =
      this->create_publisher<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>(
          "/control/command/turn_indicators_cmd", 1);
  pub_hazard_signal_ =
      this->create_publisher<autoware_vehicle_msgs::msg::HazardLightsCommand>(
          "/control/command/hazard_lights_cmd", 1);

  pub_gate_mode_ = this->create_publisher<tier4_control_msgs::msg::GateMode>(
      "/control/current_gate_mode", 1);
  pub_heartbeat_ =
      this->create_publisher<tier4_external_api_msgs::msg::Heartbeat>(
          "output/heartbeat", 1);
  pub_vehicle_engage_ =
      this->create_publisher<autoware_vehicle_msgs::msg::Engage>(
          "output/vehicle_engage", 1);

  pub_vehicle_emergency_ =
      this->create_publisher<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>(
          "/control/command/emergency_cmd", 1);

  pub_autoware_engage_ =
      this->create_publisher<autoware_vehicle_msgs::msg::Engage>(
          "/autoware/engage", 1);

  // Timer
  initTimer(1.0 / update_rate_);
}
} // namespace autoware::joy_controller

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
    autoware::joy_controller::AutowareJoyControllerNode)
