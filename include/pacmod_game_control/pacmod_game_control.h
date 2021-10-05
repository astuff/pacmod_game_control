/*
 * Unpublished Copyright (c) 2009-2021 AutonomouStuff, LLC, All Rights Reserved.
 *
 * This file is part of the PACMod ROS 1.0 driver which is released under the MIT license.
 * See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
 */

#ifndef PACMOD_GAME_CONTROL_PUBLISH_CONTROL_H
#define PACMOD_GAME_CONTROL_PUBLISH_CONTROL_H

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
  POLARIS_GEM,
  POLARIS_RANGER,
  LEXUS_RX_450H,
  INTERNATIONAL_PROSTAR,
  FREIGHTLINER_CASCADIA,
  VEHICLE_4,
  VEHICLE_5,
  VEHICLE_6,
  JUPITER_SPIRIT
};

// constants
const float ROT_RANGE_SCALER_LB = 0.05;
const float ACCEL_SCALE_FACTOR = 0.6;
const float ACCEL_OFFSET = 0.21;
const float STEER_SCALE_FACTOR = 1.5;
const float STEER_OFFSET = 1.0;
const float MAX_ROT_RAD_VEHICLE2 = 8.5;
const float MAX_ROT_RAD_VEHICLE4 = 8.5;
const float MAX_ROT_RAD_VEHICLE5 = 8.1;
const float MAX_ROT_RAD_VEHICLE6 = 8.5;
const float MAX_ROT_RAD_FREIGHTLINER_CASCADIA = 14.0;
const float MAX_ROT_RAD_JUPITER_SPIRIT = 8.5;
const float MAX_ROT_RAD_DEFAULT = 10.9956;
const uint16_t NUM_WIPER_STATES = 8;
const uint16_t WIPER_STATE_START_VALUE = 0;
const uint16_t NUM_HEADLIGHT_STATES = 3;
const uint16_t HEADLIGHT_STATE_START_VALUE = 0;
const uint16_t INVALID = -1;

class GameControl
{
public:
  void init();

private:
  // Subscriber callbacks
  void callback_control(const sensor_msgs::Joy::ConstPtr& msg);
  void callback_veh_speed(const pacmod3_msgs::VehicleSpeedRpt::ConstPtr& msg);
  void callback_pacmod_enable(const std_msgs::Bool::ConstPtr& msg);
  void callback_shift_rpt(const pacmod3_msgs::SystemRptInt::ConstPtr& msg);
  void callback_turn_rpt(const pacmod3_msgs::SystemRptInt::ConstPtr& msg);
  void callback_lights_rpt(const pacmod3_msgs::SystemRptInt::ConstPtr& msg);
  void callback_horn_rpt(const pacmod3_msgs::SystemRptBool::ConstPtr& msg);
  void callback_wiper_rpt(const pacmod3_msgs::SystemRptInt::ConstPtr& msg);

  // Command publishing
  void publish_steering_message();
  void publish_turn_signal_message();
  void publish_shifting_message();
  void publish_accelerator_message();
  void publish_brake_message();
  void publish_lights();
  void publish_horn();
  void publish_wipers();

  void PublishCommands();

  // Startup checks
  bool run_startup_checks_error();
  bool check_vehicle_type(const ros::NodeHandle& nodeH);
  bool check_controller_type(const ros::NodeHandle& nodeH);
  bool check_scale_values(const ros::NodeHandle& nodeH);

  // ROS node handle
  ros::NodeHandle n;

  // ROS publishers
  ros::Publisher turn_signal_cmd_pub_;
  ros::Publisher headlight_cmd_pub_;
  ros::Publisher horn_cmd_pub_;
  ros::Publisher wiper_cmd_pub_;
  ros::Publisher shift_cmd_pub_;
  ros::Publisher accelerator_cmd_pub_;
  ros::Publisher steering_cmd_pub_;
  ros::Publisher brake_cmd_pub_;
  ros::Publisher enable_pub_;

  // ROS subscribers
  ros::Subscriber joy_sub_;
  ros::Subscriber speed_sub_;
  ros::Subscriber enable_sub_;
  ros::Subscriber shift_sub_;
  ros::Subscriber turn_sub_;
  ros::Subscriber lights_sub_;
  ros::Subscriber horn_sub_;
  ros::Subscriber wiper_sub_;

  std::unique_ptr<controllers::Controller> controller_;
  bool local_enable_;

  bool lights_api_available_ = false;
  bool horn_api_available_ = false;
  bool wiper_api_available_ = false;

  VehicleType vehicle_type_;
  float max_rot_rad_ = MAX_ROT_RAD_DEFAULT;

  float max_veh_speed_ = std::numeric_limits<float>::quiet_NaN();
  float accel_scale_val_ = 1.0;
  float brake_scale_val_ = 1.0;
  float steering_max_speed_ = std::numeric_limits<float>::quiet_NaN();


  // bool prev_enable_ = false;
  bool last_pacmod_state_ = false;
  int headlight_state_ = 0;
  int wiper_state_ = 0;


  float last_brake_cmd_ = 0.0;

  bool enable_cmd_ = false;
  bool clear_override_cmd_ = false;

  // Pacmod status reports
  bool pacmod_enabled_rpt_ = false;
  bool prev_pacmod_enabled_rpt_ = false;
  int turn_signal_rpt_ = pacmod3_msgs::SystemRptInt::TURN_NONE;
  pacmod3_msgs::VehicleSpeedRpt::ConstPtr veh_speed_rpt_ = NULL;
  int shift_rpt_ = 0;
};

#endif  // PACMOD_GAME_CONTROL_PUBLISH_CONTROL_H
