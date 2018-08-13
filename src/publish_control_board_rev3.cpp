/*
* Unpublished Copyright (c) 2009-2018 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the PACMod ROS 1.0 driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#include "publish_control_board_rev3.h"

using namespace AS::Joystick;

int PublishControlBoardRev3::last_shift_cmd = SHIFT_NEUTRAL;
int PublishControlBoardRev3::last_turn_cmd = SIGNAL_OFF;
float PublishControlBoardRev3::last_brake_cmd = 0.0;

PublishControlBoardRev3::PublishControlBoardRev3() :
  PublishControl()
{
  // Subscribe to messages
  enable_sub = n.subscribe("/pacmod/as_tx/enabled", 20, &PublishControl::callback_pacmod_enable);
  shift_sub = n.subscribe("/pacmod/parsed_tx/shift_rpt", 20, &PublishControlBoardRev3::callback_shift_rpt);
  turn_sub = n.subscribe("/pacmod/parsed_tx/turn_rpt", 20, &PublishControlBoardRev3::callback_turn_rpt);

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

void PublishControlBoardRev3::callback_shift_rpt(const pacmod_msgs::SystemRptInt::ConstPtr& msg)
{
  shift_mutex.lock();
  // Store the latest value read from the gear state to be sent on enable/disable
  last_shift_cmd = msg->output;
  shift_mutex.unlock();
}

void PublishControlBoardRev3::callback_turn_rpt(const pacmod_msgs::SystemRptInt::ConstPtr& msg)
{
  turn_mutex.lock();
  // Store the latest value read from the gear state to be sent on enable/disable
  last_turn_cmd = msg->output;
  turn_mutex.unlock();
}

void PublishControlBoardRev3::publish_steering_message(const sensor_msgs::Joy::ConstPtr& msg)
{
  // Steering
  // Axis 0 is left thumbstick, axis 3 is right. Speed in rad/sec.
  pacmod_msgs::SteerSystemCmd steer_msg;

  steer_msg.enable = local_enable;
  steer_msg.ignore_overrides = false;

  // If the enable flag just went to true, send an override clear
  if (!prev_enable && local_enable)
    steer_msg.clear_override = true;

  float range_scale;
  if (vehicle_type == VEHICLE_4 || vehicle_type == VEHICLE_6)
    range_scale = 1.0;
  else
    range_scale = fabs(msg->axes[axes[steering_axis]]) * (STEER_OFFSET - ROT_RANGE_SCALER_LB) + ROT_RANGE_SCALER_LB;

  float speed_scale = 1.0;
  bool speed_valid = false;
  float current_speed = 0.0;

  speed_mutex.lock();

  if (last_speed_rpt != NULL)
    speed_valid = last_speed_rpt->vehicle_speed_valid;

  if (speed_valid)
    current_speed = last_speed_rpt->vehicle_speed;

  speed_mutex.unlock();

  if (speed_valid)
    speed_scale = STEER_OFFSET - fabs((current_speed / (max_veh_speed * STEER_SCALE_FACTOR))); //Never want to reach 0 speed scale.

  steer_msg.command = (range_scale * max_rot_rad) * msg->axes[axes[steering_axis]];
  steer_msg.rotation_rate = steering_max_speed * speed_scale;
  steering_set_position_with_speed_limit_pub.publish(steer_msg);
}

void PublishControlBoardRev3::publish_turn_signal_message(const sensor_msgs::Joy::ConstPtr& msg)
{
  pacmod_msgs::SystemCmdInt turn_signal_cmd_pub_msg;

  turn_signal_cmd_pub_msg.enable = local_enable;
  turn_signal_cmd_pub_msg.ignore_overrides = false;

  // If the enable flag just went to true, send an override clear
  if (!prev_enable && local_enable)
    turn_signal_cmd_pub_msg.clear_override = true;
  
  if (msg->axes[axes[DPAD_LR]] == AXES_MAX)
    turn_signal_cmd_pub_msg.command = SIGNAL_LEFT;
  else if (msg->axes[axes[DPAD_LR]] == AXES_MIN)
    turn_signal_cmd_pub_msg.command = SIGNAL_RIGHT;
  else if (local_enable != prev_enable)
  {
    if (vehicle_type == VEHICLE_6)
      turn_signal_cmd_pub_msg.command = SIGNAL_OFF;
    else
      turn_signal_cmd_pub_msg.command = last_turn_cmd;
  }
  else
    turn_signal_cmd_pub_msg.command = SIGNAL_OFF;

  // Hazard lights (both left and right turn signals)
  if (controller == HRI_SAFE_REMOTE)
  {
    if(msg->axes[2] < -0.5)
      turn_signal_cmd_pub_msg.command = SIGNAL_HAZARD;

    if ((last_axes.empty() ||
        last_axes[2] != msg->axes[2]) ||
        (local_enable != prev_enable))
      turn_signal_cmd_pub.publish(turn_signal_cmd_pub_msg);
  }
  else
  {
    if (msg->axes[axes[DPAD_UD]] == AXES_MIN)
      turn_signal_cmd_pub_msg.command = SIGNAL_HAZARD;

    if ((last_axes.empty() ||
        last_axes[axes[DPAD_LR]] != msg->axes[axes[DPAD_LR]] ||
        last_axes[axes[DPAD_UD]] != msg->axes[axes[DPAD_UD]]) ||
        (local_enable != prev_enable))
    {
      turn_signal_cmd_pub.publish(turn_signal_cmd_pub_msg);
    }
  }
}

void PublishControlBoardRev3::publish_shifting_message(const sensor_msgs::Joy::ConstPtr& msg)
{
  // Only shift if brake command is higher than 25%
  if (last_brake_cmd > 0.25)
  {
    // Shifting: reverse
    if (msg->buttons[btns[RIGHT_BTN]] == BUTTON_DOWN)
    {
      pacmod_msgs::SystemCmdInt shift_cmd_pub_msg;
      shift_cmd_pub_msg.enable = local_enable;
      shift_cmd_pub_msg.ignore_overrides = false;

      // If the enable flag just went to true, send an override clear
      if (!prev_enable && local_enable)
        shift_cmd_pub_msg.clear_override = true;

      shift_cmd_pub_msg.command = SHIFT_REVERSE;
      shift_cmd_pub.publish(shift_cmd_pub_msg);
    }

    // Shifting: drive/high
    if (msg->buttons[btns[BOTTOM_BTN]] == BUTTON_DOWN)
    {
      pacmod_msgs::SystemCmdInt shift_cmd_pub_msg;
      shift_cmd_pub_msg.enable = local_enable;
      shift_cmd_pub_msg.ignore_overrides = false;

      // If the enable flag just went to true, send an override clear
      if (!prev_enable && local_enable)
        shift_cmd_pub_msg.clear_override = true;

      shift_cmd_pub_msg.command = SHIFT_LOW;
      shift_cmd_pub.publish(shift_cmd_pub_msg);
    }

    // Shifting: park
    if (msg->buttons[btns[TOP_BTN]] == BUTTON_DOWN)
    {
      pacmod_msgs::SystemCmdInt shift_cmd_pub_msg;
      shift_cmd_pub_msg.enable = local_enable;
      shift_cmd_pub_msg.ignore_overrides = false;

      // If the enable flag just went to true, send an override clear
      if (!prev_enable && local_enable)
        shift_cmd_pub_msg.clear_override = true;

      shift_cmd_pub_msg.command = SHIFT_PARK;
      shift_cmd_pub.publish(shift_cmd_pub_msg);
    }

    // Shifting: neutral
    if (msg->buttons[btns[LEFT_BTN]] == BUTTON_DOWN)
    {
      pacmod_msgs::SystemCmdInt shift_cmd_pub_msg;
      shift_cmd_pub_msg.enable = local_enable;
      shift_cmd_pub_msg.ignore_overrides = false;

      // If the enable flag just went to true, send an override clear
      if (!prev_enable && local_enable)
        shift_cmd_pub_msg.clear_override = true;

      shift_cmd_pub_msg.command = SHIFT_NEUTRAL;
      shift_cmd_pub.publish(shift_cmd_pub_msg);
    }
  }

  // If only an enable/disable button was pressed
  if (local_enable != prev_enable)
  {
    pacmod_msgs::SystemCmdInt shift_cmd_pub_msg;
    shift_cmd_pub_msg.enable = local_enable;
    shift_cmd_pub_msg.ignore_overrides = false;

    // If the enable flag just went to true, send an override clear
    if (!prev_enable && local_enable)
      shift_cmd_pub_msg.clear_override = true;

    shift_cmd_pub_msg.command = last_shift_cmd;
    shift_cmd_pub.publish(shift_cmd_pub_msg);
  }
}

void PublishControlBoardRev3::publish_accelerator_message(const sensor_msgs::Joy::ConstPtr& msg)
{
  pacmod_msgs::SystemCmdFloat accelerator_cmd_pub_msg;

  accelerator_cmd_pub_msg.enable = local_enable;
  accelerator_cmd_pub_msg.ignore_overrides = false;

  // If the enable flag just went to true, send an override clear
  if (!prev_enable && local_enable)
    accelerator_cmd_pub_msg.clear_override = true;

  if (controller == HRI_SAFE_REMOTE)
  {
    // Accelerator
    if (msg->axes[axes[RIGHT_STICK_UD]] >= 0.0)
    {
      // only consider center-to-up range as accelerator motion
      accelerator_cmd_pub_msg.command = accel_scale_val * (msg->axes[axes[RIGHT_STICK_UD]]) * ACCEL_SCALE_FACTOR + ACCEL_OFFSET;
    }
  }
  else if(controller == LOGITECH_G29)
  {
    if (msg->axes[axes[RIGHT_TRIGGER_AXIS]] != 0)
      PublishControl::accel_0_rcvd = true;

    if (PublishControl::accel_0_rcvd)
    {
      if (vehicle_type == LEXUS_RX_450H ||
          vehicle_type == VEHICLE_4 ||
          vehicle_type == VEHICLE_5 ||
          vehicle_type == VEHICLE_6)
        accelerator_cmd_pub_msg.command = accel_scale_val * (0.5 * (msg->axes[axes[RIGHT_TRIGGER_AXIS]] + 1.0));
      else
        accelerator_cmd_pub_msg.command = accel_scale_val * (0.5 * (msg->axes[axes[RIGHT_TRIGGER_AXIS]] + 1.0)) * ACCEL_SCALE_FACTOR + ACCEL_OFFSET;
    }
    else
    {
      accelerator_cmd_pub_msg.command = 0;
    }
  }
  else
  {
    if (msg->axes[axes[RIGHT_TRIGGER_AXIS]] != 0)
      PublishControl::accel_0_rcvd = true;

    if (PublishControl::accel_0_rcvd)
    {
      if (vehicle_type == LEXUS_RX_450H ||
          vehicle_type == VEHICLE_4 ||
          vehicle_type == VEHICLE_5 ||          
          vehicle_type == VEHICLE_6)
        accelerator_cmd_pub_msg.command = accel_scale_val * (-0.5 * (msg->axes[axes[RIGHT_TRIGGER_AXIS]] - 1.0));
      else
        accelerator_cmd_pub_msg.command = accel_scale_val * (-0.5 * (msg->axes[axes[RIGHT_TRIGGER_AXIS]] - 1.0)) * ACCEL_SCALE_FACTOR + ACCEL_OFFSET;
    }
    else
    {
      accelerator_cmd_pub_msg.command = 0;
    }
  }

  accelerator_cmd_pub.publish(accelerator_cmd_pub_msg);
}

void PublishControlBoardRev3::publish_brake_message(const sensor_msgs::Joy::ConstPtr& msg)
{
  pacmod_msgs::SystemCmdFloat brake_msg;

  brake_msg.enable = local_enable;
  brake_msg.ignore_overrides = false;

  // If the enable flag just went to true, send an override clear
  if (!prev_enable && local_enable)
    brake_msg.clear_override = true;

  if (controller == HRI_SAFE_REMOTE)
  {
    brake_msg.command = (msg->axes[axes[RIGHT_STICK_UD]] > 0.0) ? 0.0 : -(brake_scale_val * msg->axes[4]);
  }
  else if(controller == LOGITECH_G29)
  {
    if (msg->axes[axes[LEFT_TRIGGER_AXIS]] != 0)
      PublishControl::brake_0_rcvd = true;

    if (PublishControl::brake_0_rcvd)
    {
      brake_msg.command = ((msg->axes[axes[LEFT_TRIGGER_AXIS]] + 1.0) / 2.0) * brake_scale_val;
    }
    else
    {
      brake_msg.command = 0;
    }
  }
  else
  {
    if (msg->axes[axes[LEFT_TRIGGER_AXIS]] != 0)
      PublishControl::brake_0_rcvd = true;

    if (PublishControl::brake_0_rcvd)
    {
      brake_msg.command = -((msg->axes[axes[LEFT_TRIGGER_AXIS]] - 1.0) / 2.0) * brake_scale_val;
    }
    else
    {
      brake_msg.command = 0;
    }
  }

  last_brake_cmd = brake_msg.command;

  brake_set_position_pub.publish(brake_msg);
}

void PublishControlBoardRev3::publish_lights_horn_wipers_message(const sensor_msgs::Joy::ConstPtr& msg)
{
  if ((vehicle_type == LEXUS_RX_450H ||
       vehicle_type == VEHICLE_5) &&
      controller != HRI_SAFE_REMOTE)
  {
    pacmod_msgs::SystemCmdInt headlight_cmd_pub_msg;
    headlight_cmd_pub_msg.enable = local_enable;
    headlight_cmd_pub_msg.ignore_overrides = false;

    // Headlights
    if (msg->axes[axes[DPAD_UD]] == AXES_MAX)
    {
      if (vehicle_type == VEHICLE_5)
      {
        if (PublishControl::headlight_state == 4)
          PublishControl::headlight_state = 5;
        else
          PublishControl::headlight_state = 4;
      }
      else
      {	
        // Rotate through headlight states as button is pressed 
        if (!PublishControl::headlight_state_change)
        {
          PublishControl::headlight_state++;
          PublishControl::headlight_state_change = true;					
        }

        if (PublishControl::headlight_state >= NUM_HEADLIGHT_STATES)
          PublishControl::headlight_state = HEADLIGHT_STATE_START_VALUE;
      }

      // If the enable flag just went to true, send an override clear
      if (!prev_enable && local_enable)
      {
        headlight_cmd_pub_msg.clear_override = true;
        PublishControl::headlight_state = HEADLIGHT_STATE_START_VALUE;
      }
    }
    else
    {
      PublishControl::headlight_state_change = false;	
    }

    headlight_cmd_pub_msg.command = PublishControl::headlight_state;
    headlight_cmd_pub.publish(headlight_cmd_pub_msg);

    // Horn
    pacmod_msgs::SystemCmdBool horn_cmd_pub_msg;
    horn_cmd_pub_msg.enable = local_enable;
    horn_cmd_pub_msg.ignore_overrides = false;

    // If the enable flag just went to true, send an override clear
    if (!prev_enable && local_enable)
      horn_cmd_pub_msg.clear_override = true;

    if (msg->buttons[btns[RIGHT_BUMPER]] == BUTTON_DOWN)
      horn_cmd_pub_msg.command = 1;
    else
      horn_cmd_pub_msg.command = 0;

    horn_cmd_pub.publish(horn_cmd_pub_msg);
  }

  if (vehicle_type == INTERNATIONAL_PROSTAR && controller != HRI_SAFE_REMOTE) // Semi
  {
    pacmod_msgs::SystemCmdInt wiper_cmd_pub_msg;
    wiper_cmd_pub_msg.enable = local_enable;
    wiper_cmd_pub_msg.ignore_overrides = false;

    // Windshield wipers
    if (msg->axes[7] == AXES_MAX)
    {
      // Rotate through wiper states as button is pressed 
      PublishControl::wiper_state++;

      if (PublishControl::wiper_state >= NUM_WIPER_STATES)
        PublishControl::wiper_state = WIPER_STATE_START_VALUE;

      // If the enable flag just went to true, send an override clear
      if (!prev_enable && local_enable)
      {
        wiper_cmd_pub_msg.clear_override = true;
        PublishControl::wiper_state = WIPER_STATE_START_VALUE;
      }

      wiper_cmd_pub_msg.command = PublishControl::wiper_state;
    }

    wiper_cmd_pub.publish(wiper_cmd_pub_msg);
  }
}
