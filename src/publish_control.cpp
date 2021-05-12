/*
* Unpublished Copyright (c) 2009-2021 AutonomouStuff, LLC, All Rights Reserved.
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
pacmod3::VehicleSpeedRpt::ConstPtr PublishControl::last_speed_rpt = NULL;
bool PublishControl::pacmod_enable;
bool PublishControl::prev_enable = false;
bool PublishControl::local_enable = false;
bool PublishControl::last_pacmod_state = false;
bool PublishControl::accel_0_rcvd = false;
bool PublishControl::brake_0_rcvd = false;
int PublishControl::headlight_state = 0;
bool PublishControl::headlight_state_change = false;
uint16_t PublishControl::wiper_state = 0;
bool PublishControl::joystick_fault_detect = false;
double PublishControl::last_joystick_msg_time = 0.0;
bool PublishControl::current_override_state = false;
bool engage_pressed = false;

PublishControl::PublishControl()
{
  // Subscribe to messages
  joy_fault_sub = n.subscribe("diagnostics", 100, &PublishControl::callback_joystick_diagnostics, this);
  joy_sub = n.subscribe("joy", 1000, &PublishControl::callback_control, this);
  speed_sub = n.subscribe("/pacmod/parsed_tx/vehicle_speed_rpt", 20, &PublishControl::callback_veh_speed);

  // Advertise published messages
  enable_pub = n.advertise<std_msgs::Bool>("/pacmod/as_rx/enable", 20);
}

void PublishControl::callback_joystick_diagnostics(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg)
{
  for (auto it = msg->status.begin(); it < msg->status.end(); it++)
  {
    if (it->name.find("Joystick Driver Status") != std::string::npos)
    {
      last_joystick_msg_time = msg->header.stamp.toSec();
      if (it->level != diagnostic_msgs::DiagnosticStatus::OK)
      {
        ROS_ERROR("JOYSTICK FAULT. PACMod will be disabled.");
        joystick_fault_detect = true;
      }
      else
      {
        joystick_fault_detect =  false;
      }
    }
  }

  bool state_changed = false;

  enable_mutex.lock();
  local_enable = pacmod_enable;
  enable_mutex.unlock();

  if (controller == HRI_SAFE_REMOTE)
  {
    // Disable
    if (joystick_fault_detect)
    {
      std_msgs::Bool bool_pub_msg;
      bool_pub_msg.data = false;
      local_enable = false;
      enable_pub.publish(bool_pub_msg);

      state_changed = true;
    }
  }
  else
  {
    // Disable
    if (joystick_fault_detect)
    {
      std_msgs::Bool bool_pub_msg;
      bool_pub_msg.data = false;
      local_enable = false;
      enable_pub.publish(bool_pub_msg);

      state_changed = true;
    }
  }

  if (state_changed)
  {
    enable_mutex.lock();
    pacmod_enable = local_enable;
    enable_mutex.unlock();

    // Global Disable
    publish_disable_on_all_systems(local_enable);
  }
}

/*
 * Called when a game controller message is received
 */
void PublishControl::callback_control(const sensor_msgs::Joy::ConstPtr& msg)
{
  try
  {
    // Only send messages when enabled, or when the state changes between enabled/disabled
    check_is_enabled(msg);

    if (!local_enable)
      publish_disable_on_all_systems(local_enable);

    if ((local_enable == true) ||
        (local_enable != prev_enable) ||
        (current_override_state))
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

      // Hazard lights
      publish_hazard_message(msg);
    }

    prev_enable = local_enable;
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
  if (msg->data == false &&
      PublishControl::last_pacmod_state == true)
    prev_enable = false;

  enable_mutex.lock();
  pacmod_enable = msg->data;
  enable_mutex.unlock();

  PublishControl::last_pacmod_state = msg->data;
}

/*
 * Called when the node receives a message from the vehicle speed topic
 */
void PublishControl::callback_veh_speed(const pacmod3::VehicleSpeedRpt::ConstPtr& msg)
{
  speed_mutex.lock();
  last_speed_rpt = msg;
  speed_mutex.unlock();
}

void PublishControl::callback_global_rpt2(const pacmod3::GlobalRpt2::ConstPtr& msg)
{
  global_rpt2_mutex.lock();
  current_override_state = msg->system_override_active;
  global_rpt2_mutex.unlock();
}

void PublishControl::check_is_enabled(const sensor_msgs::Joy::ConstPtr& msg)
{
  bool state_changed = false;

  enable_mutex.lock();
  local_enable = pacmod_enable;
  enable_mutex.unlock();

  if (controller == HRI_SAFE_REMOTE)
  {
    // Enable
    if (msg->axes[axes[DPAD_UD]] >= 0.9 && !local_enable)
    {
      // Global
      publish_global_message(msg);

      std_msgs::Bool bool_pub_msg;
      bool_pub_msg.data = true;
      local_enable = true;
      enable_pub.publish(bool_pub_msg);

      state_changed = true;
    }

    // Disable
    if (msg->axes[axes[DPAD_UD]] <= -0.9 && local_enable)
    {
      std_msgs::Bool bool_pub_msg;
      bool_pub_msg.data = false;
      local_enable = false;
      enable_pub.publish(bool_pub_msg);

      state_changed = true;
      publish_disable_on_all_systems(local_enable);
    }
  }
  else
  {
    // Enable
    if ((msg->buttons[btns[BACK_SELECT_MINUS]] == BUTTON_DOWN) &&
        (msg->buttons[btns[START_PLUS]] == BUTTON_DOWN))
    {
      if (!engage_pressed)
      {
        if (!local_enable)
        {
          // Global
          publish_global_message(msg);

          std_msgs::Bool bool_pub_msg;
          bool_pub_msg.data = true;
          local_enable = true;
          enable_pub.publish(bool_pub_msg);

          state_changed = true;
        }
        else
        {
          std_msgs::Bool bool_pub_msg;
          bool_pub_msg.data = false;
          local_enable = false;
          enable_pub.publish(bool_pub_msg);

          state_changed = true;
        }
        engage_pressed = true;
      }
    }
    else if ((msg->buttons[btns[BACK_SELECT_MINUS]] == BUTTON_DOWN) ||
             (msg->buttons[btns[START_PLUS]] == BUTTON_DOWN))
    {
      // Disable
      if ((local_enable) && !engage_pressed)
      {
        std_msgs::Bool bool_pub_msg;
        bool_pub_msg.data = false;
        local_enable = false;
        enable_pub.publish(bool_pub_msg);

        state_changed = true;
        engage_pressed = true;
        publish_disable_on_all_systems(local_enable);
      }
    }
    else
    {
      state_changed = false;
      engage_pressed = false;
    }
  }

  if (state_changed)
  {
    enable_mutex.lock();
    pacmod_enable = local_enable;
    enable_mutex.unlock();
  }
}
