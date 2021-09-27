/*
 * Unpublished Copyright (c) 2009-2018 AutonomouStuff, LLC, All Rights Reserved.
 *
 * This file is part of the PACMod ROS 1.0 driver which is released under the MIT license.
 * See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
 */

#ifndef PACMOD_GAME_CONTROL_PUBLISH_CONTROL_H
#define PACMOD_GAME_CONTROL_PUBLISH_CONTROL_H

#include "pacmod_game_control/globals.h"

#include <vector>
#include <unordered_map>

#include <pacmod_msgs/SteerSystemCmd.h>
#include <pacmod_msgs/SystemCmdBool.h>
#include <pacmod_msgs/SystemCmdFloat.h>
#include <pacmod_msgs/SystemCmdInt.h>
#include <pacmod_msgs/SystemRptInt.h>

class PublishControl
{
public:
  // public functions
  void init();
  void callback_control(const sensor_msgs::Joy::ConstPtr& msg);
  void callback_veh_speed(const pacmod_msgs::VehicleSpeedRpt::ConstPtr& msg);
  void callback_pacmod_enable(const std_msgs::Bool::ConstPtr& msg);
  void callback_shift_rpt(const pacmod_msgs::SystemRptInt::ConstPtr& msg);
  void callback_turn_rpt(const pacmod_msgs::SystemRptInt::ConstPtr& msg);
  void callback_rear_pass_door_rpt(const pacmod_msgs::SystemRptInt::ConstPtr& msg);

  // public variables
  JoyAxis steering_axis = LEFT_STICK_LR;
  float max_rot_rad = MAX_ROT_RAD_DEFAULT;
  int vehicle_type = INVALID;
  GamepadType controller = LOGITECH_F310;
  double max_veh_speed = INVALID;
  double accel_scale_val = 1.0;
  double brake_scale_val = 1.0;
  double steering_max_speed = INVALID;
  std::unordered_map<JoyAxis, int, EnumHash> axes;
  std::unordered_map<JoyButton, int, EnumHash> btns;
  pacmod_msgs::VehicleSpeedRpt::ConstPtr last_speed_rpt = NULL;
  bool pacmod_enable = false;
  bool prev_enable = false;
  bool last_pacmod_state = false;
  bool accel_0_rcvd = false;
  bool brake_0_rcvd = false;
  int headlight_state = 0;
  bool headlight_state_change = false;
  int wiper_state = 0;
  int last_shift_cmd = 0;
  int last_turn_cmd = 0;
  int last_rear_pass_door_cmd = 0;
  float last_brake_cmd = 0.0;

private:
  void check_is_enabled(const sensor_msgs::Joy::ConstPtr& msg);
  void publish_steering_message(const sensor_msgs::Joy::ConstPtr& msg);
  void publish_turn_signal_message(const sensor_msgs::Joy::ConstPtr& msg);
  void publish_rear_pass_door_message(const sensor_msgs::Joy::ConstPtr& msg);
  void publish_shifting_message(const sensor_msgs::Joy::ConstPtr& msg);
  void publish_accelerator_message(const sensor_msgs::Joy::ConstPtr& msg);
  void publish_brake_message(const sensor_msgs::Joy::ConstPtr& msg);
  void publish_lights_horn_wipers_message(const sensor_msgs::Joy::ConstPtr& msg);

  // Startup checks
  bool run_startup_checks_error();
  bool check_steering_stick_left_right(const ros::NodeHandle& nodeH);
  bool check_vehicle_type(const ros::NodeHandle& nodeH);
  bool check_controller_type(const ros::NodeHandle& nodeH);
  bool check_scale_values(const ros::NodeHandle& nodeH);

  // ROS node handle
  ros::NodeHandle n;

  // ROS publishers
  ros::Publisher turn_signal_cmd_pub;
  ros::Publisher rear_pass_door_cmd_pub;
  ros::Publisher headlight_cmd_pub;
  ros::Publisher horn_cmd_pub;
  ros::Publisher wiper_cmd_pub;
  ros::Publisher shift_cmd_pub;
  ros::Publisher accelerator_cmd_pub;
  ros::Publisher steering_set_position_with_speed_limit_pub;
  ros::Publisher brake_set_position_pub;
  ros::Publisher enable_pub;

  // ROS subscribers
  ros::Subscriber joy_sub;
  ros::Subscriber speed_sub;
  ros::Subscriber enable_sub;
  ros::Subscriber shift_sub;
  ros::Subscriber turn_sub;
  ros::Subscriber rear_pass_door_sub;

  // state vectors
  std::vector<float> last_axes;
  std::vector<int> last_buttons;

  // Other Variables
  bool local_enable;
  bool recent_state_change;
  uint8_t state_change_debounce_count;

  // mutex
  std::mutex enable_mutex;
  std::mutex speed_mutex;
  std::mutex state_change_mutex;
  std::mutex shift_mutex;
  std::mutex turn_mutex;
  std::mutex rear_pass_door_mutex;
};

#endif  // PACMOD_GAME_CONTROL_PUBLISH_CONTROL_H
