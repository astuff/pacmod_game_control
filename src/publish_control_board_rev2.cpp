/*
* Unpublished Copyright (c) 2009-2018 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the PACMod ROS 1.0 driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/
/*
HRI joystick mappings as found by ROS Kinetic Joy node on Ubuntu 16.04
Last modified 2-13-2018 by Lucas Buckland
*/

#include "publish_control_board_rev2.h"

using namespace AS::Joystick;

PublishControlBoardRev2::PublishControlBoardRev2() :
  PublishControl()
{
  // Subscribe to messages
  enable_sub = n.subscribe("/pacmod/as_tx/enable", 20, &PublishControl::callback_pacmod_enable);

  // Advertise published messages
  turn_signal_cmd_pub = n.advertise<pacmod_msgs::PacmodCmd>("/pacmod/as_rx/turn_cmd", 20);
  headlight_cmd_pub = n.advertise<pacmod_msgs::PacmodCmd>("/pacmod/as_rx/headlight_cmd", 20);
  horn_cmd_pub = n.advertise<pacmod_msgs::PacmodCmd>("/pacmod/as_rx/horn_cmd", 20);
  wiper_cmd_pub = n.advertise<pacmod_msgs::PacmodCmd>("/pacmod/as_rx/wiper_cmd", 20);
  shift_cmd_pub = n.advertise<pacmod_msgs::PacmodCmd>("/pacmod/as_rx/shift_cmd", 20);
  accelerator_cmd_pub = n.advertise<pacmod_msgs::PacmodCmd>("/pacmod/as_rx/accel_cmd", 20);
  steering_set_position_with_speed_limit_pub = n.advertise<pacmod_msgs::PositionWithSpeed>("/pacmod/as_rx/steer_cmd", 20);
  brake_set_position_pub = n.advertise<pacmod_msgs::PacmodCmd>("/pacmod/as_rx/brake_cmd", 20);
}

void PublishControlBoardRev2::publish_steering_message(const sensor_msgs::Joy::ConstPtr& msg)
{
  // Steering
  // Axis 0 is left thumbstick, axis 3 is right. Speed in rad/sec.
  pacmod_msgs::PositionWithSpeed steer_msg;

  float range_scale = fabs(msg->axes[axes[steering_axis]]) * (STEER_OFFSET - ROT_RANGE_SCALER_LB) + ROT_RANGE_SCALER_LB;
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

  steer_msg.angular_position = (range_scale * max_rot_rad) * msg->axes[axes[steering_axis]];
  steer_msg.angular_velocity_limit = steering_max_speed * speed_scale;
  steering_set_position_with_speed_limit_pub.publish(steer_msg);
}

void PublishControlBoardRev2::publish_turn_signal_message(const sensor_msgs::Joy::ConstPtr& msg)
{
  pacmod_msgs::PacmodCmd turn_signal_cmd_pub_msg;
  // Hazard lights (both left and right turn signals)
  if (controller == HRI_SAFE_REMOTE)
  {
    // Axis 2 is the "left trigger" and axis 5 is the "right trigger" single
    // axis joysticks on the back of the controller
    if(msg->axes[2] < -0.5)
      turn_signal_cmd_pub_msg.ui16_cmd = SIGNAL_HAZARD;
    else if(msg->axes[5] > 0.5)
      turn_signal_cmd_pub_msg.ui16_cmd = SIGNAL_LEFT;
    else if(msg->axes[5] < -0.5)
      turn_signal_cmd_pub_msg.ui16_cmd = SIGNAL_RIGHT;
    else
      turn_signal_cmd_pub_msg.ui16_cmd = SIGNAL_OFF;

    if (last_axes.empty() || last_axes[2] != msg->axes[2] || last_axes[5] != msg->axes[5])
      turn_signal_cmd_pub.publish(turn_signal_cmd_pub_msg);
  }
  else
  {
    if (msg->axes[axes[DPAD_UD]] == AXES_MIN)
      turn_signal_cmd_pub_msg.ui16_cmd = SIGNAL_HAZARD;
    else if (msg->axes[axes[DPAD_LR]] == AXES_MAX)
      turn_signal_cmd_pub_msg.ui16_cmd = SIGNAL_LEFT;
    else if (msg->axes[axes[DPAD_LR]] == AXES_MIN)
      turn_signal_cmd_pub_msg.ui16_cmd = SIGNAL_RIGHT;
    else
      turn_signal_cmd_pub_msg.ui16_cmd = SIGNAL_OFF;

    if (last_axes.empty() ||
        last_axes[axes[DPAD_LR]] != msg->axes[axes[DPAD_LR]] ||
        last_axes[axes[DPAD_UD]] != msg->axes[axes[DPAD_UD]])
    {
      turn_signal_cmd_pub.publish(turn_signal_cmd_pub_msg);
    }
  }
}

void PublishControlBoardRev2::publish_shifting_message(const sensor_msgs::Joy::ConstPtr& msg)
{
  // Shifting: reverse
  if (msg->buttons[btns[RIGHT_BTN]] == BUTTON_DOWN)
  {
    pacmod_msgs::PacmodCmd shift_cmd_pub_msg;
    shift_cmd_pub_msg.ui16_cmd = SHIFT_REVERSE;
    shift_cmd_pub.publish(shift_cmd_pub_msg);
  }

  // Shifting: drive/high
  if (msg->buttons[btns[BOTTOM_BTN]] == BUTTON_DOWN)
  {
    pacmod_msgs::PacmodCmd shift_cmd_pub_msg;
    shift_cmd_pub_msg.ui16_cmd = SHIFT_LOW;
    shift_cmd_pub.publish(shift_cmd_pub_msg);
  }

  // Shifting: park
  if (msg->buttons[btns[TOP_BTN]] == BUTTON_DOWN)
  {
    pacmod_msgs::PacmodCmd shift_cmd_pub_msg;
    shift_cmd_pub_msg.ui16_cmd = SHIFT_PARK;
    shift_cmd_pub.publish(shift_cmd_pub_msg);
  }

  // Shifting: neutral
  if (msg->buttons[btns[LEFT_BTN]] == BUTTON_DOWN)
  {
    pacmod_msgs::PacmodCmd shift_cmd_pub_msg;
    shift_cmd_pub_msg.ui16_cmd = SHIFT_NEUTRAL;
    shift_cmd_pub.publish(shift_cmd_pub_msg);
  }
}

void PublishControlBoardRev2::publish_accelerator_message(const sensor_msgs::Joy::ConstPtr& msg)
{
  pacmod_msgs::PacmodCmd accelerator_cmd_pub_msg;

  if (controller == HRI_SAFE_REMOTE)
  {
    // Accelerator
    if (msg->axes[axes[RIGHT_STICK_UD]] >= 0.0)
    {
      // only consider center-to-up range as accelerator motion
      accelerator_cmd_pub_msg.f64_cmd = accel_scale_val * (msg->axes[axes[RIGHT_STICK_UD]]) * ACCEL_SCALE_FACTOR 
        + ACCEL_OFFSET;
    }
  }
  else if(controller == LOGITECH_G29)
  {
    if (msg->axes[axes[RIGHT_TRIGGER_AXIS]] != 0)
      PublishControl::accel_0_rcvd = true;

    if (PublishControl::accel_0_rcvd)
    {
      if ((vehicle_type == LEXUS_RX_450H) ||
          (vehicle_type == VEHICLE_4))
        accelerator_cmd_pub_msg.f64_cmd = accel_scale_val * (0.5 * (msg->axes[axes[RIGHT_TRIGGER_AXIS]] + 1.0));
      else
        accelerator_cmd_pub_msg.f64_cmd = accel_scale_val * (0.5 * (msg->axes[axes[RIGHT_TRIGGER_AXIS]] + 1.0)) * ACCEL_SCALE_FACTOR
          + ACCEL_OFFSET;
    }
    else
    {
      accelerator_cmd_pub_msg.f64_cmd = 0;
    }
  }
  else
  {
    if (msg->axes[axes[RIGHT_TRIGGER_AXIS]] != 0)
      PublishControl::accel_0_rcvd = true;

    if (PublishControl::accel_0_rcvd)
    {
      if ((vehicle_type == LEXUS_RX_450H) ||
          (vehicle_type == VEHICLE_4))
        accelerator_cmd_pub_msg.f64_cmd = accel_scale_val * (-0.5 * (msg->axes[axes[RIGHT_TRIGGER_AXIS]] - 1.0));
      else
        accelerator_cmd_pub_msg.f64_cmd = accel_scale_val * (-0.5 * (msg->axes[axes[RIGHT_TRIGGER_AXIS]] - 1.0)) * ACCEL_SCALE_FACTOR + ACCEL_OFFSET;
    }
    else
    {
      accelerator_cmd_pub_msg.f64_cmd = 0;
    }
  }

  accelerator_cmd_pub.publish(accelerator_cmd_pub_msg);
}

void PublishControlBoardRev2::publish_brake_message(const sensor_msgs::Joy::ConstPtr& msg)
{
  pacmod_msgs::PacmodCmd brake_msg;

  if (controller == HRI_SAFE_REMOTE)
  {
    brake_msg.f64_cmd = (msg->axes[axes[RIGHT_STICK_UD]] > 0.0) ? 0.0 : -(brake_scale_val * msg->axes[4]);
  }
  else if(controller == LOGITECH_G29)
  {
    if (msg->axes[axes[LEFT_TRIGGER_AXIS]] != 0)
      PublishControl::brake_0_rcvd = true;

    if (PublishControl::brake_0_rcvd)
      brake_msg.f64_cmd = ((msg->axes[axes[LEFT_TRIGGER_AXIS]] + 1.0) / 2.0) * brake_scale_val;
    else
      brake_msg.f64_cmd = 0;
  }
  else
  {
    if (msg->axes[axes[LEFT_TRIGGER_AXIS]] != 0)
      PublishControl::brake_0_rcvd = true;

    if (PublishControl::brake_0_rcvd)
      brake_msg.f64_cmd = -((msg->axes[axes[LEFT_TRIGGER_AXIS]] - 1.0) / 2.0) * brake_scale_val;
    else
      brake_msg.f64_cmd = 0;
  }

  brake_set_position_pub.publish(brake_msg);
}

void PublishControlBoardRev2::publish_lights_horn_wipers_message(const sensor_msgs::Joy::ConstPtr& msg)
{
  static uint16_t headlight_state = 0;
  static uint16_t wiper_state = 0;
  
  if (vehicle_type == 2 && controller != HRI_SAFE_REMOTE)
  {
    // Headlights
    if (msg->axes[axes[DPAD_UD]] == AXES_MAX)
    {
      // Rotate through headlight states as button is pressed 
      headlight_state++;

      if(headlight_state >= NUM_HEADLIGHT_STATES)
        headlight_state = HEADLIGHT_STATE_START_VALUE;

      pacmod_msgs::PacmodCmd headlight_cmd_pub_msg;
      headlight_cmd_pub_msg.ui16_cmd = headlight_state;
      headlight_cmd_pub.publish(headlight_cmd_pub_msg);
    }

    // Horn
    pacmod_msgs::PacmodCmd horn_cmd_pub_msg;

    if (msg->buttons[btns[RIGHT_BUMPER]] == BUTTON_DOWN)
      horn_cmd_pub_msg.ui16_cmd = 1;
    else
      horn_cmd_pub_msg.ui16_cmd = 0;

    horn_cmd_pub.publish(horn_cmd_pub_msg);
  }

  if (vehicle_type == 3 && controller != HRI_SAFE_REMOTE) // Semi
  {
    // Windshield wipers
    if (msg->buttons[btns[LEFT_BUMPER]] == BUTTON_DOWN)
    {
      // Rotate through wiper states as button is pressed 
      wiper_state++;

      if(wiper_state >= NUM_WIPER_STATES)
        wiper_state = WIPER_STATE_START_VALUE;

      pacmod_msgs::PacmodCmd wiper_cmd_pub_msg;
      wiper_cmd_pub_msg.ui16_cmd = wiper_state;
      wiper_cmd_pub.publish(wiper_cmd_pub_msg);
    }
  }
}
