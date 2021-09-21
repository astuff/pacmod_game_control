/*
* Unpublished Copyright (c) 2009-2018 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the PACMod ROS 1.0 driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#include "pacmod_game_control/publish_control.h"

#include <unordered_map>

using namespace AS::Joystick;  // NOLINT

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
bool PublishControl::prev_enable = false;
bool PublishControl::local_enable = false;
bool PublishControl::last_pacmod_state = false;
bool PublishControl::accel_0_rcvd = false;
bool PublishControl::brake_0_rcvd = false;
int PublishControl::headlight_state = 0;
bool PublishControl::headlight_state_change = false;
int PublishControl::wiper_state = 0;


int PublishControl::last_shift_cmd = pacmod_msgs::SystemCmdInt::SHIFT_NEUTRAL;
int PublishControl::last_turn_cmd = pacmod_msgs::SystemCmdInt::TURN_NONE;
int PublishControl::last_rear_pass_door_cmd = pacmod_msgs::SystemCmdInt::DOOR_NEUTRAL;
float PublishControl::last_brake_cmd = 0.0;


PublishControl::PublishControl()
{
  // Subscribe to messages
  joy_sub = n.subscribe("joy", 1000, &PublishControl::callback_control, this);
  speed_sub = n.subscribe("/pacmod/parsed_tx/vehicle_speed_rpt", 20, &PublishControl::callback_veh_speed, this);

  enable_sub = n.subscribe("/pacmod/as_tx/enabled", 20, &PublishControl::callback_pacmod_enable, this);
  shift_sub = n.subscribe("/pacmod/parsed_tx/shift_rpt", 20, &PublishControl::callback_shift_rpt, this);
  turn_sub = n.subscribe("/pacmod/parsed_tx/turn_rpt", 20, &PublishControl::callback_turn_rpt, this);
  rear_pass_door_sub = n.subscribe("/pacmod/parsed_tx/rear_pass_door_rpt",
                                   20, &PublishControl::callback_rear_pass_door_rpt, this);

  // Advertise published messages
  enable_pub = n.advertise<std_msgs::Bool>("/pacmod/as_rx/enable", 20);
  turn_signal_cmd_pub = n.advertise<pacmod_msgs::SystemCmdInt>("/pacmod/as_rx/turn_cmd", 20);
  rear_pass_door_cmd_pub = n.advertise<pacmod_msgs::SystemCmdInt>("/pacmod/as_rx/rear_pass_door_cmd", 20);
  headlight_cmd_pub = n.advertise<pacmod_msgs::SystemCmdInt>("/pacmod/as_rx/headlight_cmd", 20);
  horn_cmd_pub = n.advertise<pacmod_msgs::SystemCmdBool>("/pacmod/as_rx/horn_cmd", 20);
  wiper_cmd_pub = n.advertise<pacmod_msgs::SystemCmdInt>("/pacmod/as_rx/wiper_cmd", 20);
  shift_cmd_pub = n.advertise<pacmod_msgs::SystemCmdInt>("/pacmod/as_rx/shift_cmd", 20);
  accelerator_cmd_pub = n.advertise<pacmod_msgs::SystemCmdFloat>("/pacmod/as_rx/accel_cmd", 20);
  steering_set_position_with_speed_limit_pub = n.advertise<pacmod_msgs::SteerSystemCmd>("/pacmod/as_rx/steer_cmd", 20);
  brake_set_position_pub = n.advertise<pacmod_msgs::SystemCmdFloat>("/pacmod/as_rx/brake_cmd", 20);
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

    if (local_enable == true || local_enable != prev_enable)
    {
      // Steering
      publish_steering_message(msg);

      // Turn signals
      publish_turn_signal_message(msg);

      // Door signals
      publish_rear_pass_door_message(msg);

      // Shifting
      publish_shifting_message(msg);

      // Accelerator
      publish_accelerator_message(msg);

      // Brake
      publish_brake_message(msg);

      // Lights and horn
      publish_lights_horn_wipers_message(msg);
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
void PublishControl::callback_veh_speed(const pacmod_msgs::VehicleSpeedRpt::ConstPtr& msg)
{
  speed_mutex.lock();
  last_speed_rpt = msg;
  speed_mutex.unlock();
}

void PublishControl::callback_shift_rpt(const pacmod_msgs::SystemRptInt::ConstPtr& msg)
{
  shift_mutex.lock();
  // Store the latest value read from the gear state to be sent on enable/disable
  last_shift_cmd = msg->output;
  shift_mutex.unlock();
}

void PublishControl::callback_turn_rpt(const pacmod_msgs::SystemRptInt::ConstPtr& msg)
{
  turn_mutex.lock();
  // Store the latest value read from the gear state to be sent on enable/disable
  last_turn_cmd = msg->output;
  turn_mutex.unlock();
}

void PublishControl::callback_rear_pass_door_rpt(const pacmod_msgs::SystemRptInt::ConstPtr& msg)
{
  rear_pass_door_mutex.lock();
  // Store the latest value read to be sent on enable/disable
  last_rear_pass_door_cmd = msg->output;
  rear_pass_door_mutex.unlock();
}

// Publishing
void PublishControl::publish_steering_message(const sensor_msgs::Joy::ConstPtr& msg)
{
  // Steering
  // Axis 0 is left thumbstick, axis 3 is right. Speed in rad/sec.
  pacmod_msgs::SteerSystemCmd steer_msg;

  steer_msg.enable = local_enable;
  steer_msg.ignore_overrides = false;

  // If the enable flag just went to true, send an override clear and clear_faults
  if (!prev_enable && local_enable)
  {
    steer_msg.clear_override = true;
    steer_msg.clear_faults = true;
  }

  float range_scale;
  if (vehicle_type == VEHICLE_4 ||
      vehicle_type == VEHICLE_6 ||
      vehicle_type == LEXUS_RX_450H ||
      vehicle_type == FREIGHTLINER_CASCADIA ||
      vehicle_type == JUPITER_SPIRIT)
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
    if (current_speed < max_veh_speed)
      speed_scale = STEER_OFFSET - fabs(
        (current_speed / (max_veh_speed * STEER_SCALE_FACTOR)));  // this could go negative.
    else
       speed_scale = 0.33333;  // clips the equation assuming 1 offset and 1.5 scale_factor

  steer_msg.command = (range_scale * max_rot_rad) * msg->axes[axes[steering_axis]];
  steer_msg.rotation_rate = steering_max_speed * speed_scale;
  steering_set_position_with_speed_limit_pub.publish(steer_msg);
}

void PublishControl::publish_turn_signal_message(const sensor_msgs::Joy::ConstPtr& msg)
{
  pacmod_msgs::SystemCmdInt turn_signal_cmd_pub_msg;

  turn_signal_cmd_pub_msg.enable = local_enable;
  turn_signal_cmd_pub_msg.ignore_overrides = false;

  // If the enable flag just went to true, send an override clear
  if (!prev_enable && local_enable)
  {
    turn_signal_cmd_pub_msg.clear_override = true;
    turn_signal_cmd_pub_msg.clear_faults = true;
  }

  // Hazard lights (both left and right turn signals), and HRI support
  if (controller == HRI_SAFE_REMOTE)
  {
    // Axis 2 is the "left trigger" and axis 5 is the "right trigger" single
    // axis joysticks on the back of the controller
    if (msg->axes[4] < -0.5)
      turn_signal_cmd_pub_msg.command = pacmod_msgs::SystemCmdInt::TURN_HAZARDS;
    else if (msg->axes[3] > 0.5)
      turn_signal_cmd_pub_msg.command = pacmod_msgs::SystemCmdInt::TURN_LEFT;
    else if (msg->axes[3] < -0.5)
      turn_signal_cmd_pub_msg.command = pacmod_msgs::SystemCmdInt::TURN_RIGHT;
    else
      turn_signal_cmd_pub_msg.command = pacmod_msgs::SystemCmdInt::TURN_NONE;

    if (last_axes.empty() ||
        last_axes[4] != msg->axes[4] ||
        last_axes[3] != msg->axes[3] ||
        local_enable != prev_enable)
      turn_signal_cmd_pub.publish(turn_signal_cmd_pub_msg);
  }
  else  // Every other controller
  {
    if (msg->axes[axes[DPAD_LR]] == AXES_MAX)
      turn_signal_cmd_pub_msg.command = pacmod_msgs::SystemCmdInt::TURN_LEFT;
    else if (msg->axes[axes[DPAD_LR]] == AXES_MIN)
      turn_signal_cmd_pub_msg.command = pacmod_msgs::SystemCmdInt::TURN_RIGHT;
    else if (msg->axes[axes[DPAD_UD]] == AXES_MIN && msg->buttons[btns[LEFT_BUMPER]] != BUTTON_DOWN)
      turn_signal_cmd_pub_msg.command = pacmod_msgs::SystemCmdInt::TURN_HAZARDS;
    else if (local_enable != prev_enable)
    {
      if (vehicle_type == VEHICLE_6)
        turn_signal_cmd_pub_msg.command = pacmod_msgs::SystemCmdInt::TURN_NONE;
      else
        turn_signal_cmd_pub_msg.command = last_turn_cmd;
    }
    else
      turn_signal_cmd_pub_msg.command = pacmod_msgs::SystemCmdInt::TURN_NONE;

    if (last_axes.empty() ||
        last_axes[axes[DPAD_LR]] != msg->axes[axes[DPAD_LR]] ||
        last_axes[axes[DPAD_UD]] != msg->axes[axes[DPAD_UD]] ||
        local_enable != prev_enable)
    {
      turn_signal_cmd_pub.publish(turn_signal_cmd_pub_msg);
    }
  }
}

void PublishControl::publish_shifting_message(const sensor_msgs::Joy::ConstPtr& msg)
{
  // Only shift if brake command is higher than 25%
  if (last_brake_cmd > 0.25)
  {
    pacmod_msgs::SystemCmdInt shift_cmd_pub_msg;
    shift_cmd_pub_msg.enable = local_enable;
    shift_cmd_pub_msg.ignore_overrides = false;

    // If the enable flag just went to true, send an override clear and a faults_clear
    if (!prev_enable && local_enable)
    {
      shift_cmd_pub_msg.clear_override = true;
      shift_cmd_pub_msg.clear_faults = true;
    }

    uint8_t desired_gear = 0x0;
    desired_gear |= (msg->buttons[btns[RIGHT_BTN]]  == BUTTON_DOWN) << pacmod_msgs::SystemCmdInt::SHIFT_REVERSE
                  | (msg->buttons[btns[BOTTOM_BTN]] == BUTTON_DOWN) << pacmod_msgs::SystemCmdInt::SHIFT_HIGH
                  | (msg->buttons[btns[TOP_BTN]]    == BUTTON_DOWN) << pacmod_msgs::SystemCmdInt::SHIFT_PARK
                  | (msg->buttons[btns[LEFT_BTN]]   == BUTTON_DOWN) << pacmod_msgs::SystemCmdInt::SHIFT_NEUTRAL;

    switch (desired_gear)
    {
      case 1 << pacmod_msgs::SystemCmdInt::SHIFT_REVERSE:
        shift_cmd_pub_msg.command = pacmod_msgs::SystemCmdInt::SHIFT_REVERSE;
        break;
      case 1 << pacmod_msgs::SystemCmdInt::SHIFT_HIGH:
        shift_cmd_pub_msg.command = pacmod_msgs::SystemCmdInt::SHIFT_HIGH;
        break;
      case 1 << pacmod_msgs::SystemCmdInt::SHIFT_PARK:
        shift_cmd_pub_msg.command = pacmod_msgs::SystemCmdInt::SHIFT_PARK;
        break;
      case 1 << pacmod_msgs::SystemCmdInt::SHIFT_NEUTRAL:
        shift_cmd_pub_msg.command = pacmod_msgs::SystemCmdInt::SHIFT_NEUTRAL;
        break;
      // If we've got an invalid command (or multiple buttons pressed) return and don't publish the message
      default: return;
    }
     shift_cmd_pub.publish(shift_cmd_pub_msg);
  }
  else if (local_enable != prev_enable)   // If only an enable/disable button was pressed
  {
    pacmod_msgs::SystemCmdInt shift_cmd_pub_msg;
    shift_cmd_pub_msg.enable = local_enable;
    shift_cmd_pub_msg.ignore_overrides = false;

    // If the enable flag just went to true, send an override clear
    if (!prev_enable && local_enable)
    {
      shift_cmd_pub_msg.clear_override = true;
      shift_cmd_pub_msg.clear_faults = true;
    }

    shift_cmd_pub_msg.command = last_shift_cmd;
    shift_cmd_pub.publish(shift_cmd_pub_msg);
  }
}

void PublishControl::publish_accelerator_message(const sensor_msgs::Joy::ConstPtr& msg)
{
  pacmod_msgs::SystemCmdFloat accelerator_cmd_pub_msg;

  accelerator_cmd_pub_msg.enable = local_enable;
  accelerator_cmd_pub_msg.ignore_overrides = false;

  // If the enable flag just went to true, send an override clear
  if (!prev_enable && local_enable)
  {
    accelerator_cmd_pub_msg.clear_override = true;
    accelerator_cmd_pub_msg.clear_faults = true;
  }

  if (controller == HRI_SAFE_REMOTE)
  {
    // Accelerator
    if (msg->axes[5] <= 0.0)
    {
      // only consider center-to-up range as accelerator motion
      accelerator_cmd_pub_msg.command = accel_scale_val * -(msg->axes[5]);
    }
  }
  else if (controller == LOGITECH_G29)
  {
    if (msg->axes[axes[RIGHT_TRIGGER_AXIS]] != 0)
      PublishControl::accel_0_rcvd = true;

    if (PublishControl::accel_0_rcvd)
    {
      if (vehicle_type == POLARIS_RANGER ||
          vehicle_type == LEXUS_RX_450H ||
          vehicle_type == FREIGHTLINER_CASCADIA ||
          vehicle_type == JUPITER_SPIRIT ||
          vehicle_type == VEHICLE_4 ||
          vehicle_type == VEHICLE_5 ||
          vehicle_type == VEHICLE_6)
        accelerator_cmd_pub_msg.command =
          accel_scale_val * (0.5 * (msg->axes[axes[RIGHT_TRIGGER_AXIS]] + 1.0));
      else
        accelerator_cmd_pub_msg.command =
          accel_scale_val * (0.5 * (msg->axes[axes[RIGHT_TRIGGER_AXIS]] + 1.0)) * ACCEL_SCALE_FACTOR + ACCEL_OFFSET;
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
      if (vehicle_type == POLARIS_RANGER ||
          vehicle_type == LEXUS_RX_450H ||
          vehicle_type == FREIGHTLINER_CASCADIA ||
          vehicle_type == JUPITER_SPIRIT ||
          vehicle_type == VEHICLE_4 ||
          vehicle_type == VEHICLE_5 ||
          vehicle_type == VEHICLE_6)
        accelerator_cmd_pub_msg.command =
          accel_scale_val * (-0.5 * (msg->axes[axes[RIGHT_TRIGGER_AXIS]] - 1.0));
      else
        accelerator_cmd_pub_msg.command =
          accel_scale_val * (-0.5 * (msg->axes[axes[RIGHT_TRIGGER_AXIS]] - 1.0)) * ACCEL_SCALE_FACTOR + ACCEL_OFFSET;
    }
    else
    {
      accelerator_cmd_pub_msg.command = 0;
    }
  }

  accelerator_cmd_pub.publish(accelerator_cmd_pub_msg);
}

void PublishControl::publish_brake_message(const sensor_msgs::Joy::ConstPtr& msg)
{
  pacmod_msgs::SystemCmdFloat brake_msg;

  brake_msg.enable = local_enable;
  brake_msg.ignore_overrides = false;

  // If the enable flag just went to true, send an override clear
  if (!prev_enable && local_enable)
  {
    brake_msg.clear_override = true;
    brake_msg.clear_faults = true;
  }
  if (controller == HRI_SAFE_REMOTE)
  {
    brake_msg.command = (msg->axes[2] > 0.0) ? 0.0 : (brake_scale_val * -msg->axes[2]);
  }
  else if (controller == LOGITECH_G29)
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
      float brake_value = -((msg->axes[axes[LEFT_TRIGGER_AXIS]] - 1.0) / 2.0) * brake_scale_val;
      if (vehicle_type == LEXUS_RX_450H)
      {
        // These constants came from playing around in excel until stuff looked good. Seems to work okay
        brake_msg.command = fmin(pow(brake_value, 3) * 2.0F - pow(brake_value, 2) * 1.5F + brake_value * 0.625F, 1.0F);
      }
      else
      {
        brake_msg.command = brake_value;
      }
    }
    else
    {
      brake_msg.command = 0;
    }
  }

  last_brake_cmd = brake_msg.command;

  brake_set_position_pub.publish(brake_msg);
}

void PublishControl::publish_lights_horn_wipers_message(const sensor_msgs::Joy::ConstPtr& msg)
{
  if ((vehicle_type == LEXUS_RX_450H ||
       vehicle_type == VEHICLE_5 ||
       vehicle_type == VEHICLE_6 ||
       vehicle_type == FREIGHTLINER_CASCADIA ||
       vehicle_type == JUPITER_SPIRIT) &&
      controller != HRI_SAFE_REMOTE)
  {
    pacmod_msgs::SystemCmdInt headlight_cmd_pub_msg;
    headlight_cmd_pub_msg.enable = local_enable;
    headlight_cmd_pub_msg.ignore_overrides = false;

    // Headlights
    if (msg->axes[axes[DPAD_UD]] == AXES_MAX && msg->buttons[btns[LEFT_BUMPER]] != BUTTON_DOWN)
    {
      if (vehicle_type == VEHICLE_5)
      {
        if (PublishControl::headlight_state == 1)
          PublishControl::headlight_state = 2;
        else
          PublishControl::headlight_state = 1;
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
        headlight_cmd_pub_msg.clear_faults = true;
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
    {
      horn_cmd_pub_msg.clear_override = true;
      horn_cmd_pub_msg.clear_faults = true;
    }

    if (msg->buttons[btns[RIGHT_BUMPER]] == BUTTON_DOWN)
      horn_cmd_pub_msg.command = 1;
    else
      horn_cmd_pub_msg.command = 0;

    horn_cmd_pub.publish(horn_cmd_pub_msg);
  }

  if (vehicle_type == INTERNATIONAL_PROSTAR && controller != HRI_SAFE_REMOTE)  // Semi
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
        wiper_cmd_pub_msg.clear_faults = true;
        PublishControl::wiper_state = WIPER_STATE_START_VALUE;
      }

      wiper_cmd_pub_msg.command = PublishControl::wiper_state;
    }

    wiper_cmd_pub.publish(wiper_cmd_pub_msg);
  }
}

void PublishControl::publish_rear_pass_door_message(const sensor_msgs::Joy::ConstPtr& msg)
{
  pacmod_msgs::SystemCmdInt rear_pass_door_cmd_pub_msg;

  rear_pass_door_cmd_pub_msg.enable = local_enable;
  rear_pass_door_cmd_pub_msg.ignore_overrides = false;

  // If the enable flag just went to true, send an override clear
  if (!prev_enable && local_enable)
  {
    rear_pass_door_cmd_pub_msg.clear_override = true;
    rear_pass_door_cmd_pub_msg.clear_faults = true;
  }

  if (vehicle_type == JUPITER_SPIRIT)
  {
    if (controller != HRI_SAFE_REMOTE)
    {
      if (msg->axes[axes[DPAD_UD]] == AXES_MAX && msg->buttons[btns[LEFT_BUMPER]] == BUTTON_DOWN)
        rear_pass_door_cmd_pub_msg.command = pacmod_msgs::SystemCmdInt::DOOR_CLOSE;
      else if (msg->axes[axes[DPAD_UD]] == AXES_MIN && msg->buttons[btns[LEFT_BUMPER]] == BUTTON_DOWN)
        rear_pass_door_cmd_pub_msg.command = pacmod_msgs::SystemCmdInt::DOOR_OPEN;
      else if (local_enable != prev_enable)
        rear_pass_door_cmd_pub_msg.command = last_rear_pass_door_cmd;
      else
        rear_pass_door_cmd_pub_msg.command = pacmod_msgs::SystemCmdInt::DOOR_NEUTRAL;
      // Send messages when enabled, or when the state changes between axes[DPAD_UD], or between enabled/disabled
      if (last_axes.empty() ||
          last_axes[axes[DPAD_UD]] != msg->axes[axes[DPAD_UD]] ||
          local_enable != prev_enable)
      {
          rear_pass_door_cmd_pub.publish(rear_pass_door_cmd_pub_msg);
      }
    }
  }
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
    if (msg->axes[axes[DPAD_LR]] <= -0.9 && !local_enable)
    {
      std_msgs::Bool bool_pub_msg;
      bool_pub_msg.data = true;
      local_enable = true;
      enable_pub.publish(bool_pub_msg);

      state_changed = true;
    }

    // Disable
    if (msg->axes[axes[DPAD_LR]] >= +0.9 && local_enable)
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
    // Enable
    if (msg->buttons[btns[START_PLUS]] == BUTTON_DOWN &&
        msg->buttons[btns[BACK_SELECT_MINUS]] == BUTTON_DOWN && !local_enable)
    {
      std_msgs::Bool bool_pub_msg;
      bool_pub_msg.data = true;
      local_enable = true;
      enable_pub.publish(bool_pub_msg);

      state_changed = true;
    }

    // Disable
    if (msg->buttons[btns[BACK_SELECT_MINUS]] == BUTTON_DOWN &&
        msg->buttons[btns[START_PLUS]] != BUTTON_DOWN && local_enable)
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
  }
}
