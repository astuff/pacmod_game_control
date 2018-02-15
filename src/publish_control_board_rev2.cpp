/*
* Unpublished Copyright (c) 2009-2018 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the PACMod ROS 1.0 driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/
/*
HRI joystick mappings as found by ROS Kinetic Joy node on Ubuntu 16.04
Last modified 2-13-2018 by Lucas Buckland

Left thumbstick:
*Axis 0 is left (+1) to right (-1), centered=0.0
*Axis 1 is up (+1) to down (-1), centered=0.0

Triggers:
*Axis 2 is left trigger: up=1.0, center=0.0, down=-1.0
*Axis 5 is right trigger: up=1.0, center=0.0, down=-1.0

Right thumbstick:
*Axis 3 is left (+1) to right (-1), centered=0.0
*Axis 4 is up (+1) to down (-1), centered=0.0

Arrow buttons:
*Axis 6 is left (+1.0) and right (-1.0) arrow buttons, not pressed = 0.0
*Axis 7 is up (+1.0) and down (-1.0) arrow buttons, not pressed = 0.0

Number buttons:
"1" button pressed = button 0 = 1, not pressed = 0
"2" button pressed = button 1 = 1, not pressed = 0
"3" button pressed = button 2 = 1, not pressed = 0
"4" button pressed = button 3 = 1, not pressed = 0
*/

#include "publish_control_board_rev2.h"

PublishControlBoardRev2::PublishControlBoardRev2()
{
  // Subscribe to messages
  joy_sub = n.subscribe("joy", 1000, &PublishControlBoardRev2::callback_control, this);
  speed_sub = n.subscribe("/pacmod/parsed_tx/vehicle_speed_rpt", 20, &PublishControl::callback_veh_speed);
  enable_sub = n.subscribe("/pacmod/as_tx/enable", 20, &PublishControl::callback_pacmod_enable);

  // Advertise published messages
  enable_pub = n.advertise<std_msgs::Bool>("/pacmod/as_rx/enable", 20);
  turn_signal_cmd_pub = n.advertise<pacmod_msgs::PacmodCmd>("/pacmod/as_rx/turn_cmd", 20);
  headlight_cmd_pub = n.advertise<pacmod_msgs::PacmodCmd>("/pacmod/as_rx/headlight_cmd", 20);
  horn_cmd_pub = n.advertise<pacmod_msgs::PacmodCmd>("/pacmod/as_rx/horn_cmd", 20);
  wiper_cmd_pub = n.advertise<pacmod_msgs::PacmodCmd>("/pacmod/as_rx/wiper_cmd", 20);
  shift_cmd_pub = n.advertise<pacmod_msgs::PacmodCmd>("/pacmod/as_rx/shift_cmd", 20);
  accelerator_cmd_pub = n.advertise<pacmod_msgs::PacmodCmd>("/pacmod/as_rx/accel_cmd", 20);
  steering_set_position_with_speed_limit_pub = n.advertise<pacmod_msgs::PositionWithSpeed>("/pacmod/as_rx/steer_cmd", 20);
  brake_set_position_pub = n.advertise<pacmod_msgs::PacmodCmd>("/pacmod/as_rx/brake_cmd", 20);
}

bool PublishControlBoardRev2::check_is_enabled(const sensor_msgs::Joy::ConstPtr& msg)
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
  
  if (msg->axes[axes[DPAD_LR]] == AXES_MAX)
    turn_signal_cmd_pub_msg.ui16_cmd = SIGNAL_LEFT;
  else if (msg->axes[axes[DPAD_LR]] == AXES_MIN)
    turn_signal_cmd_pub_msg.ui16_cmd = SIGNAL_RIGHT;
  else
    turn_signal_cmd_pub_msg.ui16_cmd = SIGNAL_OFF;

  // Hazard lights (both left and right turn signals)
  if (controller == HRI_SAFE_REMOTE)
  {
    if(msg->axes[2] < -0.5)
      turn_signal_cmd_pub_msg.ui16_cmd = SIGNAL_HAZARD;

    if (last_axes.empty() ||
        last_axes[2] != msg->axes[2])
      turn_signal_cmd_pub.publish(turn_signal_cmd_pub_msg);
  }
  else
  {
    if (msg->axes[axes[DPAD_UD]] == AXES_MIN)
      turn_signal_cmd_pub_msg.ui16_cmd = SIGNAL_HAZARD;

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

  // Shifting: drive/low
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

  /* TODO: What??
  // Shifting: high
  if (msg->buttons[6] == 1 &&
     (last_buttons.empty() ||
      last_buttons[6] != msg->buttons[6]))
  {
    pacmod_msgs::PacmodCmd shift_cmd_pub_msg;
    shift_cmd_pub_msg.ui16_cmd = SHIFT_HIGH;
    shift_cmd_pub.publish(shift_cmd_pub_msg);
  }
  */
}

void PublishControlBoardRev2::publish_accelerator_message(const sensor_msgs::Joy::ConstPtr& msg)
{
  pacmod_msgs::PacmodCmd accelerator_cmd_pub_msg;
  bool enable_accel;

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
  else
  {
    if (msg->axes[axes[RIGHT_TRIGGER_AXIS]] != 0)
      enable_accel = true;

    if (enable_accel)
    {
      if ((vehicle_type == VEHICLE_2) ||
          (vehicle_type == VEHICLE_4))
        accelerator_cmd_pub_msg.f64_cmd = (-0.5 * (msg->axes[axes[RIGHT_TRIGGER_AXIS]] - 1.0));
      else
        accelerator_cmd_pub_msg.f64_cmd = (-0.5 * (msg->axes[axes[RIGHT_TRIGGER_AXIS]] - 1.0)) * ACCEL_SCALE_FACTOR
          + ACCEL_OFFSET;
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
  bool enable_brake;

  if (controller == HRI_SAFE_REMOTE)
  {
    brake_msg.f64_cmd = (msg->axes[axes[RIGHT_STICK_UD]] > 0.0) ? 0.0 : -(brake_scale_val * msg->axes[4]);
  }
  else
  {
    if (msg->axes[axes[LEFT_TRIGGER_AXIS]] != 0)
      enable_brake = true;

    if (enable_brake)
    {
      brake_msg.f64_cmd = -((msg->axes[axes[LEFT_TRIGGER_AXIS]] - 1.0) / 2.0) * brake_scale_val;
    }
    else
    {
      brake_msg.f64_cmd = 0;
    }
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

    if (msg->buttons[7] == 1)
      horn_cmd_pub_msg.ui16_cmd = 1;
    else
      horn_cmd_pub_msg.ui16_cmd = 0;

    horn_cmd_pub.publish(horn_cmd_pub_msg);
  }

  if (vehicle_type == 3 && controller != HRI_SAFE_REMOTE) // Semi
  {
    // Windshield wipers
    if (msg->axes[7] == AXES_MAX)
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

/*
 * Called when a game controller message is received
 */
void PublishControlBoardRev2::callback_control(const sensor_msgs::Joy::ConstPtr& msg)
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

  //last_buttons.clear();
  //last_buttons.insert(last_buttons.end(), msg->buttons.begin(), msg->buttons.end());
  last_axes.clear();
  last_axes.insert(last_axes.end(), msg->axes.begin(), msg->axes.end());
}
