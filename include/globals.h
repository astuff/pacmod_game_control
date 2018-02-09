/*
* Unpublished Copyright (c) 2009-2018 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the PACMod ROS 1.0 driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/


#include <cstdio>
#include <mutex>
#include <unordered_map>


#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <pacmod_msgs/PositionWithSpeed.h>
#include <pacmod_msgs/PacmodCmd.h>
#include <pacmod_msgs/VehicleSpeedRpt.h>

#ifndef GLOBALS_H
#define GLOBALS_H

enum ShiftState
{
  SHIFT_PARK = 0,
  SHIFT_REVERSE = 1,
  SHIFT_NEUTRAL = 2,
  SHIFT_LOW = 3,
  SHIFT_HIGH = 4
};

enum GamepadType
{
  LOGITECH_F310,
  HRI_SAFE_REMOTE,
  LOGITECH_G29,
  NINTENDO_SWITCH_WIRED_PLUS
};

enum JoyAxis
{
  LEFT_STICK_UD,
  LEFT_STICK_LR,
  RIGHT_STICK_UD,
  RIGHT_STICK_LR,
  DPAD_UD,
  DPAD_LR,
  LEFT_TRIGGER_AXIS,   // Sometimes button, sometimes axis
  RIGHT_TRIGGER_AXIS   // Sometimes button, sometimes axis
};

enum JoyButton
{
  TOP_BTN,
  LEFT_BTN,
  BOTTOM_BTN,
  RIGHT_BTN,
  LEFT_BUMPER,
  RIGHT_BUMPER,
  BACK_SELECT_MINUS,
  START_PLUS,
  LEFT_TRIGGER_BTN,   // Sometimes button, sometimes axis
  RIGHT_TRIGGER_BTN,  // Sometimes button, sometimes axis
  LEFT_STICK_PUSH,
  RIGHT_STICK_PUSH
};

struct EnumHash
{
  template <typename T>
  std::size_t operator()(T t) const
  {
    return static_cast<std::size_t>(t);
  }
};

static float MAX_ROT_RAD = 10.9956;
static const float ROT_RANGE_SCALER_LB = 0.05;
static const uint16_t NUM_WIPER_STATES = 8;
static const uint16_t WIPER_STATE_START_VALUE = 0;
static const uint16_t NUM_HEADLIGHT_STATES = 3;
static const uint16_t HEADLIGHT_STATE_START_VALUE = 0;

static std::unordered_map<JoyAxis, int, EnumHash> axes;
static std::unordered_map<JoyButton, int, EnumHash> btns;

static int vehicle_type = -1;
static int board_rev = 2; // TODO : Change this to -1 when the parameter is added
static GamepadType controller = LOGITECH_F310;
static JoyAxis steering_axis = LEFT_STICK_LR;
static double steering_max_speed = -1.0;
static bool pacmod_enable;
static std::mutex enable_mutex;
static pacmod_msgs::VehicleSpeedRpt::ConstPtr last_speed_rpt = NULL;
static std::mutex speed_mutex;
static std::vector<float> last_axes;
static std::vector<int> last_buttons;
static double max_veh_speed = -1.0;
static double accel_scale_val = 1.0;
static double brake_scale_val = 1.0;
static uint16_t wiper_state = 0;
static uint16_t headlight_state = 0;

static bool enable_accel = false;
static bool enable_brake = false;


static ros::Publisher turn_signal_cmd_pub;
static ros::Publisher headlight_cmd_pub;
static ros::Publisher horn_cmd_pub;
static ros::Publisher wiper_cmd_pub;
static ros::Publisher shift_cmd_pub;
static ros::Publisher accelerator_cmd_pub;
static ros::Publisher steering_set_position_with_speed_limit_pub;
static ros::Publisher brake_set_position_pub;
static ros::Publisher enable_pub;

#endif
