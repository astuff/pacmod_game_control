/*
* Unpublished Copyright (c) 2009-2018 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the PACMod ROS 1.0 driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#include "publish_control_board_rev3.h"

using namespace AS::Joystick;

PublishControlBoardRev3::PublishControlBoardRev3() :
  PublishControl()
{
  // Advertise published messages
  turn_signal_cmd_pub = n.advertise<pacmod_msgs::SystemCmdInt>("/pacmod/as_rx/turn_cmd", 20);
  headlight_cmd_pub = n.advertise<pacmod_msgs::SystemCmdInt>("/pacmod/as_rx/headlight_cmd", 20);
  horn_cmd_pub = n.advertise<pacmod_msgs::SystemCmdBool>("/pacmod/as_rx/horn_cmd", 20);
  wiper_cmd_pub = n.advertise<pacmod_msgs::SystemCmdInt>("/pacmod/as_rx/wiper_cmd", 20);
  shift_cmd_pub = n.advertise<pacmod_msgs::SystemCmdInt>("/pacmod/as_rx/shift_cmd", 20);
  accelerator_cmd_pub = n.advertise<pacmod_msgs::SystemCmdFloat>("/pacmod/as_rx/accel_cmd", 20);
  steering_set_position_with_speed_limit_pub = n.advertise<pacmod_msgs::SteerSystemCmd>("/pacmod/as_rx/steer_cmd", 20);
  brake_set_position_pub = n.advertise<pacmod_msgs::SystemCmdFloat>("/pacmod/as_rx/brake_cmd", 20);
}

void PublishControlBoardRev3::publish_steering_message(const sensor_msgs::Joy::ConstPtr& msg)
{
}

void PublishControlBoardRev3::publish_turn_signal_message(const sensor_msgs::Joy::ConstPtr& msg)
{
}

void PublishControlBoardRev3::publish_shifting_message(const sensor_msgs::Joy::ConstPtr& msg)
{
}

void PublishControlBoardRev3::publish_accelerator_message(const sensor_msgs::Joy::ConstPtr& msg)
{
}

void PublishControlBoardRev3::publish_brake_message(const sensor_msgs::Joy::ConstPtr& msg)
{
}

void PublishControlBoardRev3::publish_lights_horn_wipers_message(const sensor_msgs::Joy::ConstPtr& msg)
{
}
