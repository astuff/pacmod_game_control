/*
 * Unpublished Copyright (c) 2009-2018 AutonomouStuff, LLC, All Rights Reserved.
 *
 * This file is part of the PACMod ROS 1.0 driver which is released under the MIT license.
 * See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
 */

#ifndef PACMOD_GAME_CONTROL_GLOBALS_H
#define PACMOD_GAME_CONTROL_GLOBALS_H

#include <cstdio>
#include <mutex>
#include <unordered_map>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <pacmod_msgs/VehicleSpeedRpt.h>

enum GamepadType
{
  LOGITECH_F310,
  HRI_SAFE_REMOTE,
  LOGITECH_G29,
  NINTENDO_SWITCH_WIRED_PLUS,
  XBOX_ONE
};

enum VehicleType
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

enum JoyAxis
{
  LEFT_STICK_UD,
  LEFT_STICK_LR,
  RIGHT_STICK_UD,
  RIGHT_STICK_LR,
  DPAD_UD,
  DPAD_LR,
  LEFT_TRIGGER_AXIS,  // Sometimes button, sometimes axis
  RIGHT_TRIGGER_AXIS  // Sometimes button, sometimes axis
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
const float AXES_MIN = -1.0;
const float AXES_MAX = 1.0;
const uint16_t NUM_WIPER_STATES = 8;
const uint16_t WIPER_STATE_START_VALUE = 0;
const uint16_t NUM_HEADLIGHT_STATES = 3;
const uint16_t HEADLIGHT_STATE_START_VALUE = 0;
const uint16_t INVALID = -1;
const uint16_t BUTTON_DOWN = 1;

#endif  // PACMOD_GAME_CONTROL_GLOBALS_H
