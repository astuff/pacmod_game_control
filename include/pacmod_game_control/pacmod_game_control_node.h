/*
 * Unpublished Copyright (c) 2009-2021 AutonomouStuff, LLC, All Rights Reserved.
 *
 * This file is part of the PACMod ROS 1.0 driver which is released under the MIT license.
 * See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
 */

#ifndef PACMOD_GAME_CONTROL_PACMOD_GAME_CONTROL_H
#define PACMOD_GAME_CONTROL_PACMOD_GAME_CONTROL_H

#include "pacmod_game_control/controllers.h"

#include <limits>
#include <memory>
#include <vector>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <pacmod3_msgs/msg/steering_cmd.hpp>
#include <pacmod3_msgs/msg/system_cmd_bool.hpp>
#include <pacmod3_msgs/msg/system_cmd_float.hpp>
#include <pacmod3_msgs/msg/system_cmd_int.hpp>
#include <pacmod3_msgs/msg/system_rpt_bool.hpp>
#include <pacmod3_msgs/msg/system_rpt_int.hpp>
#include <pacmod3_msgs/msg/vehicle_speed_rpt.hpp>
#include <std_msgs/msg/bool.hpp>

enum class VehicleType
{
  POLARIS_GEM,
  POLARIS_RANGER,
  LEXUS_RX_450H,
  INTERNATIONAL_PROSTAR,
  FREIGHTLINER_CASCADIA,
  JUPITER_SPIRIT
};

// constants
const float ROT_RANGE_SCALER_LB = 0.05;
const float ACCEL_SCALE_FACTOR = 0.6;
const float ACCEL_OFFSET = 0.21;
const float STEER_SCALE_FACTOR = 1.5;
const float STEER_OFFSET = 1.0;
const float MAX_ROT_RAD_LEXUS_RX_450H = 8.5;
const float MAX_ROT_RAD_FREIGHTLINER_CASCADIA = 14.0;
const float MAX_ROT_RAD_JUPITER_SPIRIT = 8.5;
const float MAX_ROT_RAD_DEFAULT = 10.9956;
const uint16_t NUM_WIPER_STATES = 8;
const uint16_t WIPER_STATE_START_VALUE = 0;
const uint16_t NUM_HEADLIGHT_STATES = 3;
const uint16_t HEADLIGHT_STATE_START_VALUE = 0;
const uint16_t INVALID = -1;

class GameControlNode : public rclcpp::Node
{
public:
  GameControlNode();
  void Init();

private:
  // Subscriber callbacks
  void GamepadCb(const sensor_msgs::msg::Joy::SharedPtr msg);
  void VehicleSpeedCb(const pacmod3_msgs::msg::VehicleSpeedRpt::SharedPtr msg);
  void PacmodEnabledCb(const std_msgs::msg::Bool::SharedPtr msg);
  void ShiftRptCb(const pacmod3_msgs::msg::SystemRptInt::SharedPtr msg);
  void TurnRptCb(const pacmod3_msgs::msg::SystemRptInt::SharedPtr msg);
  void LightsRptCb(const pacmod3_msgs::msg::SystemRptInt::SharedPtr msg);
  void HornRptCb(const pacmod3_msgs::msg::SystemRptBool::SharedPtr msg);
  void WiperRptCb(const pacmod3_msgs::msg::SystemRptInt::SharedPtr msg);

  // Command publishing
  void PublishCommands();
  void PublishAccelerator();
  void PublishBrake();
  void PublishSteering();
  void PublishShifting();
  void PublishTurnSignal();
  void PublishLights();
  void PublishHorn();
  void PublishWipers();

  // Startup checks
  bool RunStartupChecks();
  bool CheckVehicleType();
  bool CheckControllerType();
  bool CheckScaleValues();

  rclcpp::Publisher<pacmod3_msgs::msg::SystemCmdInt>::SharedPtr turn_signal_cmd_pub_;
  rclcpp::Publisher<pacmod3_msgs::msg::SystemCmdInt>::SharedPtr headlight_cmd_pub_;
  rclcpp::Publisher<pacmod3_msgs::msg::SystemCmdBool>::SharedPtr horn_cmd_pub_;
  rclcpp::Publisher<pacmod3_msgs::msg::SystemCmdInt>::SharedPtr wiper_cmd_pub_;
  rclcpp::Publisher<pacmod3_msgs::msg::SystemCmdInt>::SharedPtr shift_cmd_pub_;
  rclcpp::Publisher<pacmod3_msgs::msg::SystemCmdFloat>::SharedPtr accelerator_cmd_pub_;
  rclcpp::Publisher<pacmod3_msgs::msg::SteeringCmd>::SharedPtr steering_cmd_pub_;
  rclcpp::Publisher<pacmod3_msgs::msg::SystemCmdFloat>::SharedPtr brake_cmd_pub_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<pacmod3_msgs::msg::VehicleSpeedRpt>::SharedPtr speed_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Subscription<pacmod3_msgs::msg::SystemRptInt>::SharedPtr lights_sub_;
  rclcpp::Subscription<pacmod3_msgs::msg::SystemRptBool>::SharedPtr horn_sub_;
  rclcpp::Subscription<pacmod3_msgs::msg::SystemRptInt>::SharedPtr wiper_sub_;

  std::unique_ptr<controllers::Controller> controller_ = nullptr;

  bool lights_api_available_ = false;
  bool horn_api_available_ = false;
  bool wiper_api_available_ = false;

  VehicleType vehicle_type_;
  float max_rot_rad_ = MAX_ROT_RAD_DEFAULT;

  float max_veh_speed_ = std::numeric_limits<float>::quiet_NaN();
  float accel_scale_val_ = 1.0;
  float brake_scale_val_ = 1.0;
  float steering_max_speed_ = std::numeric_limits<float>::quiet_NaN();

  // Pacmod commands
  bool enable_cmd_ = false;
  bool clear_override_cmd_ = false;
  float last_brake_cmd_ = 0.0;
  int headlight_cmd_ = 0;
  int wiper_cmd_ = 0;

  // Pacmod status reports
  bool pacmod_enabled_rpt_ = false;
  pacmod3_msgs::msg::VehicleSpeedRpt::SharedPtr veh_speed_rpt_ = NULL;
};

#endif  // PACMOD_GAME_CONTROL_PACMOD_GAME_CONTROL_H
