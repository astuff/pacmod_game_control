/*
 * Unpublished Copyright (c) 2009-2021 AutonomouStuff, LLC, All Rights Reserved.
 *
 * This file is part of the PACMod ROS 1.0 driver which is released under the MIT license.
 * See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
 */

#ifndef PACMOD_GAME_CONTROL_CONTROLLERS_H
#define PACMOD_GAME_CONTROL_CONTROLLERS_H

#include "pacmod_game_control/globals.h"

#include <unordered_map>

#include <sensor_msgs/Joy.h>

class Controller
{
public:
  Controller();
  void set_controller_input(const sensor_msgs::Joy& joy_msg);
  virtual float get_accelerator_value();
  virtual float get_brake_value();
  // virtual float get_steering_value();
  virtual int get_turn_signal_cmd();
  virtual int get_shift_cmd();
  virtual bool get_horn_cmd();
  virtual bool get_headlight_change();
  virtual bool get_wiper_change();
  virtual bool get_enable();
  virtual bool get_disable();

protected:
  sensor_msgs::Joy input_msg_;
  std::unordered_map<JoyAxis, int, EnumHash> axes_;
  std::unordered_map<JoyButton, int, EnumHash> btns_;
};

class LogitechG29Controller : public Controller
{
public:
  LogitechG29Controller();
  float get_accelerator_value();
  float get_brake_value();
};

class HriSafeController : public Controller
{
public:
  HriSafeController();
  float get_accelerator_value();
  float get_brake_value();
  int get_turn_signal_cmd();
  bool get_horn_cmd();
  bool get_wiper_change();
  bool get_enable();
  bool get_disable();
};

#endif  // PACMOD_GAME_CONTROL_CONTROLLERS_H
