/*
* Unpublished Copyright (c) 2009-2018 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the PACMod ROS 1.0 driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#include "publish_control.h"

using namespace AS::Joystick;

JoyAxis PublishControl::steering_axis = LEFT_STICK_LR;
float PublishControl::max_rot_rad = MAX_ROT_RAD_DEFAULT;
int PublishControl::vehicle_type = INVALID;
GamepadType PublishControl::controller = LOGITECH_F310;
int PublishControl::board_rev = INVALID;
double PublishControl::max_veh_speed = INVALID;
double PublishControl::accel_scale_val = 1.0;
double PublishControl::brake_scale_val = 1.0;
double PublishControl::steering_max_speed = INVALID;
std::unordered_map<JoyAxis, int, EnumHash> PublishControl::axes;
std::unordered_map<JoyButton, int, EnumHash> PublishControl::btns;
pacmod_msgs::VehicleSpeedRpt::ConstPtr PublishControl::last_speed_rpt = NULL;
bool PublishControl::pacmod_enable;
bool PublishControl::recent_state_change = false;
uint8_t PublishControl::state_change_debounce_count = 0;

PublishControl::PublishControl()
{
  // Subscribe to messages
  joy_sub = n.subscribe("joy", 1000, &PublishControl::callback_control, this);
  speed_sub = n.subscribe("/pacmod/parsed_tx/vehicle_speed_rpt", 20, &PublishControl::callback_veh_speed);
  enable_sub = n.subscribe("/pacmod/as_tx/enable", 20, &PublishControl::callback_pacmod_enable);

  // Advertise published messages
  enable_pub = n.advertise<std_msgs::Bool>("/pacmod/as_rx/enable", 20);
}

/*
 * Called when a game controller message is received
 */
void PublishControl::callback_control(const sensor_msgs::Joy::ConstPtr& msg)
{
  try
  {
    if (check_is_enabled(msg) == true)
    {
      // Steering
      publish_steering_message(msg);

      // Turn signals
      publish_turn_signal_message(msg);

      // Shifting
      publish_shifting_message(msg);

      // Accelerator
      publish_accelerator_message(msg);

      // Brake
      publish_brake_message(msg);

      // Lights and horn
      publish_lights_horn_wipers_message(msg);
    }
  }
  catch (const std::out_of_range& oor)
  {
    ROS_ERROR("An out-of-range exception was caught. This probably means you selected the wrong controller type.");
  }

  last_axes.clear();
  last_axes.insert(last_axes.end(), msg->axes.begin(), msg->axes.end());
}

/*
 * Called when the node receives a message from the enable topic
 */
void PublishControl::callback_pacmod_enable(const std_msgs::Bool::ConstPtr& msg)
{
  state_change_mutex.lock();

  if (!recent_state_change)
  {
    enable_mutex.lock();
    pacmod_enable = msg->data;
    enable_mutex.unlock();
  }
  else
  {
    state_change_debounce_count++;

    if (state_change_debounce_count > STATE_CHANGE_DEBOUNCE_THRESHOLD)
      recent_state_change = false;
  }

  state_change_mutex.unlock();
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

bool PublishControl::check_is_enabled(const sensor_msgs::Joy::ConstPtr& msg)
{
  bool local_enable = false;

  enable_mutex.lock();
  local_enable = pacmod_enable;
  enable_mutex.unlock();

  if (controller == HRI_SAFE_REMOTE)
  {
    // Enable
    if (msg->axes[axes[DPAD_UD]] >= 0.9)
    {
      std_msgs::Bool bool_pub_msg;
      bool_pub_msg.data = true;
      local_enable = true;
      enable_pub.publish(bool_pub_msg);
    }

    // Disable
    if (msg->axes[axes[DPAD_UD]] <= -0.9)
    {
      std_msgs::Bool bool_pub_msg;
      bool_pub_msg.data = false;
      local_enable = false;
      enable_pub.publish(bool_pub_msg);
    }
  }
  else
  {
    // Enable
    if (msg->buttons[btns[START_PLUS]] == BUTTON_DOWN)
    {

      std_msgs::Bool bool_pub_msg;
      bool_pub_msg.data = true;
      local_enable = true;
      enable_pub.publish(bool_pub_msg);
    }

    // Disable
    if (msg->buttons[btns[BACK_SELECT_MINUS]] == BUTTON_DOWN)
    {
      std_msgs::Bool bool_pub_msg;
      bool_pub_msg.data = false;
      local_enable = false;
      enable_pub.publish(bool_pub_msg);
    }
  }

  enable_mutex.lock();
  pacmod_enable = local_enable;
  enable_mutex.unlock();

  return local_enable;
}
