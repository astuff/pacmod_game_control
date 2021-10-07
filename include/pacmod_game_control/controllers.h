/*
 * Unpublished Copyright (c) 2009-2021 AutonomouStuff, LLC, All Rights Reserved.
 *
 * This file is part of the PACMod ROS 1.0 driver which is released under the MIT license.
 * See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
 */

#ifndef PACMOD_GAME_CONTROL_CONTROLLERS_H
#define PACMOD_GAME_CONTROL_CONTROLLERS_H

#include <unordered_map>

#include <sensor_msgs/msg/joy.hpp>

namespace controllers
{
const uint16_t BUTTON_PRESSED = 1;
const uint16_t BUTTON_DEPRESSED = 0;
const float AXES_MIN = -1.0;
const float AXES_MAX = 1.0;

enum class JoyAxis
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

enum class JoyButton
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

class Controller
{
public:
  Controller();
  virtual ~Controller();
  void set_controller_input(const sensor_msgs::msg::Joy& joy_msg);
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
  sensor_msgs::msg::Joy input_msg_;
  sensor_msgs::msg::Joy prev_input_msg_;
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

}  // namespace controllers

#endif  // PACMOD_GAME_CONTROL_CONTROLLERS_H
