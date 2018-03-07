/*
* Unpublished Copyright (c) 2009-2018 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the PACMod ROS 1.0 driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#ifndef PUBLISH_CONTROL_H
#define PUBLISH_CONTROL_H

#include "globals.h"

namespace AS
{
namespace Joystick
{

class PublishControl
{

public:
  // public functions
  virtual void callback_control(const sensor_msgs::Joy::ConstPtr& msg) = 0;
  static void callback_veh_speed(const pacmod_msgs::VehicleSpeedRpt::ConstPtr& msg);
  static void callback_pacmod_enable(const std_msgs::Bool::ConstPtr& msg);
  
  // public variables
  static JoyAxis steering_axis;
  static float max_rot_rad;
  static int vehicle_type;
  static GamepadType controller;
  static int board_rev;
  static double max_veh_speed;
  static double accel_scale_val;
  static double brake_scale_val;
  static double steering_max_speed;
  static std::unordered_map<JoyAxis, int, EnumHash> axes;
  static std::unordered_map<JoyButton, int, EnumHash> btns;
  static pacmod_msgs::VehicleSpeedRpt::ConstPtr last_speed_rpt;
  static bool pacmod_enable;
  
private:

  // private functions
  virtual bool check_is_enabled(const sensor_msgs::Joy::ConstPtr& msg) = 0;
  virtual void publish_steering_message(const sensor_msgs::Joy::ConstPtr& msg) = 0;
  virtual void publish_turn_signal_message(const sensor_msgs::Joy::ConstPtr& msg) = 0;
  virtual void publish_shifting_message(const sensor_msgs::Joy::ConstPtr& msg) = 0;
  virtual void publish_accelerator_message(const sensor_msgs::Joy::ConstPtr& msg) = 0;
  virtual void publish_brake_message(const sensor_msgs::Joy::ConstPtr& msg) = 0;
  virtual void publish_lights_horn_wipers_message(const sensor_msgs::Joy::ConstPtr& msg) = 0;
};

}
}

#endif
