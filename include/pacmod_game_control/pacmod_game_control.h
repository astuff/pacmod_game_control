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

#include <ros/ros.h>
#include <pacmod3_msgs/SystemCmdBool.h>
#include <pacmod3_msgs/SystemCmdFloat.h>
#include <pacmod3_msgs/SystemCmdInt.h>
#include <pacmod3_msgs/SystemRptBool.h>
#include <pacmod3_msgs/SystemRptInt.h>
#include <pacmod3_msgs/VehicleSpeedRpt.h>
#include <std_msgs/Bool.h>

enum class VehicleType
{
  FORD_RANGER,
  FORD_TRANSIT,
  FREIGHTLINER_CASCADIA,
  HONDA_CRV,
  INTERNATIONAL_PROSTAR,
  JUPITER_SPIRIT,
  LEXUS_RX_450H,
  POLARIS_GEM,
  POLARIS_RANGER,
};

// Constants
const float ROT_RANGE_SCALER_LB = 0.05;
const float ACCEL_SCALE_FACTOR = 0.6;
const float ACCEL_OFFSET = 0.21;
const float STEER_SCALE_FACTOR = 1.5;
const float STEER_OFFSET = 1.0;
const uint16_t NUM_WIPER_STATES = 8;
const uint16_t WIPER_STATE_START_VALUE = 0;
const uint16_t NUM_HEADLIGHT_STATES = 3;
const uint16_t HEADLIGHT_STATE_START_VALUE = 0;
const uint16_t INVALID = -1;

// Steering wheel maximums
const float MAX_ROT_RAD_DEFAULT = 10.9956;
const float MAX_ROT_RAD_FORD_RANGER = 10.29;
const float MAX_ROT_RAD_FORD_TRANSIT = 10.21;
const float MAX_ROT_RAD_FREIGHTLINER_CASCADIA = 14.0;
const float MAX_ROT_RAD_HONDA_CRV = 7.4;
const float MAX_ROT_RAD_JUPITER_SPIRIT = 8.5;
const float MAX_ROT_RAD_LEXUS_RX_450H = 8.5;

class GameControl
{
public:
  void Init();

private:
  // Subscriber callbacks
  void GamepadCb(const sensor_msgs::Joy::ConstPtr& msg);
  void VehicleSpeedCb(const pacmod3_msgs::VehicleSpeedRpt::ConstPtr& msg);
  void PacmodEnabledCb(const std_msgs::Bool::ConstPtr& msg);
  void ShiftRptCb(const pacmod3_msgs::SystemRptInt::ConstPtr& msg);
  void TurnRptCb(const pacmod3_msgs::SystemRptInt::ConstPtr& msg);
  void LightsRptCb(const pacmod3_msgs::SystemRptInt::ConstPtr& msg);
  void HornRptCb(const pacmod3_msgs::SystemRptBool::ConstPtr& msg);
  void WiperRptCb(const pacmod3_msgs::SystemRptInt::ConstPtr& msg);

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
  bool CheckVehicleType(const ros::NodeHandle& nodeH);
  bool CheckControllerType(const ros::NodeHandle& nodeH);
  bool CheckScaleValues(const ros::NodeHandle& nodeH);

  ros::Publisher turn_signal_cmd_pub_;
  ros::Publisher headlight_cmd_pub_;
  ros::Publisher horn_cmd_pub_;
  ros::Publisher wiper_cmd_pub_;
  ros::Publisher shift_cmd_pub_;
  ros::Publisher accelerator_cmd_pub_;
  ros::Publisher steering_cmd_pub_;
  ros::Publisher brake_cmd_pub_;

  ros::Subscriber joy_sub_;
  ros::Subscriber speed_sub_;
  ros::Subscriber enable_sub_;
  ros::Subscriber lights_sub_;
  ros::Subscriber horn_sub_;
  ros::Subscriber wiper_sub_;

  ros::NodeHandle nh_;

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
  pacmod3_msgs::VehicleSpeedRpt::ConstPtr veh_speed_rpt_ = NULL;
};

#endif  // PACMOD_GAME_CONTROL_PACMOD_GAME_CONTROL_H
