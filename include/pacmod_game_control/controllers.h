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
  virtual ~Controller();
  void set_controller_input(const sensor_msgs::Joy& joy_msg);
  virtual float accelerator_value();
  virtual float brake_value();
  virtual float steering_value();
  virtual int turn_signal_cmd();
  virtual int shift_cmd();
  virtual bool horn_cmd();
  virtual bool headlight_change();
  virtual bool wiper_change();
  virtual bool enable();
  virtual bool disable();

protected:
  sensor_msgs::Joy input_msg_;
  std::unordered_map<JoyAxis, int, EnumHash> axes_;
  std::unordered_map<JoyButton, int, EnumHash> btns_;
};

class LogitechG29Controller : public Controller
{
public:
  LogitechG29Controller();
  float accelerator_value() override;
  float brake_value() override;
};

class HriSafeController : public Controller
{
public:
  HriSafeController();
  float accelerator_value() override;
  float brake_value() override;
  int turn_signal_cmd() override;
  bool horn_cmd() override;
  bool wiper_change() override;
  bool enable() override;
  bool disable() override;
};

#endif  // PACMOD_GAME_CONTROL_CONTROLLERS_H
