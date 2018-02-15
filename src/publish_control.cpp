/*
* Unpublished Copyright (c) 2009-2018 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the PACMod ROS 1.0 driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#include "publish_control.h"

JoyAxis PublishControl::steering_axis = LEFT_STICK_LR;
float PublishControl::max_rot_rad = 10.9956;
int PublishControl::vehicle_type = -1;
GamepadType PublishControl::controller = LOGITECH_F310;
int PublishControl::board_rev = -1;
double PublishControl::max_veh_speed = -1.0;
double PublishControl::accel_scale_val = 1.0;
double PublishControl::brake_scale_val = 1.0;
double PublishControl::steering_max_speed = -1.0;
std::unordered_map<JoyAxis, int, EnumHash> PublishControl::axes;
std::unordered_map<JoyButton, int, EnumHash> PublishControl::btns;
pacmod_msgs::VehicleSpeedRpt::ConstPtr PublishControl::last_speed_rpt = NULL;
bool PublishControl::pacmod_enable;

/*
 * Called when the node receives a message from the enable topic
 */
void PublishControl::callback_pacmod_enable(const std_msgs::Bool::ConstPtr& msg)
{
  enable_mutex.lock();
  pacmod_enable = msg->data;
  enable_mutex.unlock();
}

/*
 * Called when the node receives a message from the vehicle speed topic
 */
void PublishControl::callback_veh_speed(const pacmod_msgs::VehicleSpeedRpt::ConstPtr& msg)
{
  speed_mutex.lock();
  last_speed_rpt = msg;
  speed_mutex.unlock();
}

