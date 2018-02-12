/*
* Unpublished Copyright (c) 2009-2018 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the PACMod ROS 1.0 driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#ifndef GLOBALS_H
#define GLOBALS_H

#include <cstdio>
#include <mutex>
#include <unordered_map>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <pacmod_msgs/PositionWithSpeed.h>
#include <pacmod_msgs/PacmodCmd.h>
#include <pacmod_msgs/VehicleSpeedRpt.h>


// Enums
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


// static constants
static const float ROT_RANGE_SCALER_LB = 0.05;
static const uint16_t NUM_WIPER_STATES = 8;
static const uint16_t WIPER_STATE_START_VALUE = 0;
static const uint16_t NUM_HEADLIGHT_STATES = 3;
static const uint16_t HEADLIGHT_STATE_START_VALUE = 0;


// ROS publishers
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
