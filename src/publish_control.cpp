/*
 * Unpublished Copyright (c) 2009-2021 AutonomouStuff, LLC, All Rights Reserved.
 *
 * This file is part of the PACMod ROS 1.0 driver which is released under the MIT license.
 * See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
 */

#include "pacmod_game_control/publish_control.h"

#include <unordered_map>

#include <pacmod3_msgs/SteeringCmd.h>

void PublishControl::init()
{
  if (run_startup_checks_error())
  {
    ros::shutdown();
  }

  // Subs
  joy_sub = n.subscribe("joy", 1000, &PublishControl::callback_control, this);
  speed_sub = n.subscribe("pacmod/vehicle_speed_rpt", 20, &PublishControl::callback_veh_speed, this);
  enable_sub = n.subscribe("pacmod/enabled", 20, &PublishControl::callback_pacmod_enable, this);
  shift_sub = n.subscribe("pacmod/shift_rpt", 20, &PublishControl::callback_shift_rpt, this);
  turn_sub = n.subscribe("pacmod/turn_rpt", 20, &PublishControl::callback_turn_rpt, this);
  lights_sub = n.subscribe("pacmod/headlight_rpt", 10, &PublishControl::callback_lights_rpt, this);
  horn_sub = n.subscribe("pacmod/horn_rpt", 10, &PublishControl::callback_horn_rpt, this);
  wiper_sub = n.subscribe("pacmod/wiper_rpt", 10, &PublishControl::callback_wiper_rpt, this);

  // Pubs
  enable_pub = n.advertise<std_msgs::Bool>("pacmod/enable", 20);
  turn_signal_cmd_pub = n.advertise<pacmod3_msgs::SystemCmdInt>("pacmod/turn_cmd", 20);
  headlight_cmd_pub = n.advertise<pacmod3_msgs::SystemCmdInt>("pacmod/headlight_cmd", 20);
  horn_cmd_pub = n.advertise<pacmod3_msgs::SystemCmdBool>("pacmod/horn_cmd", 20);
  wiper_cmd_pub = n.advertise<pacmod3_msgs::SystemCmdInt>("pacmod/wiper_cmd", 20);
  shift_cmd_pub = n.advertise<pacmod3_msgs::SystemCmdInt>("pacmod/shift_cmd", 20);
  accelerator_cmd_pub = n.advertise<pacmod3_msgs::SystemCmdFloat>("pacmod/accel_cmd", 20);
  steering_set_position_with_speed_limit_pub = n.advertise<pacmod3_msgs::SteeringCmd>("pacmod/steering_cmd", 20);
  brake_set_position_pub = n.advertise<pacmod3_msgs::SystemCmdFloat>("pacmod/brake_cmd", 20);
}

void PublishControl::callback_control(const sensor_msgs::Joy::ConstPtr& msg)
{
  controller->set_controller_input(*msg);
  try
  {
    // Only send messages when enabled, or when the state changes between enabled/disabled
    check_is_enabled();

    if (local_enable == true || local_enable != prev_enable)
    {
      publish_steering_message();
      publish_turn_signal_message();
      publish_shifting_message();
      publish_accelerator_message();
      publish_brake_message();
      publish_wipers();
    }

    prev_enable = local_enable;
  }
  catch (const std::out_of_range& oor)
  {
    ROS_ERROR("An out-of-range exception was caught. This probably means you selected the wrong controller_type type.");
  }

  last_axes.clear();
  last_axes.insert(last_axes.end(), msg->axes.begin(), msg->axes.end());
}

void PublishControl::callback_pacmod_enable(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data == false && PublishControl::last_pacmod_state == true)
    prev_enable = false;

  std::unique_lock<std::mutex> lock(enable_mutex);
  pacmod_enable = msg->data;

  PublishControl::last_pacmod_state = msg->data;
}

// Feedback callbacks
void PublishControl::callback_veh_speed(const pacmod3_msgs::VehicleSpeedRpt::ConstPtr& msg)
{
  std::unique_lock<std::mutex> lock(speed_mutex);
  last_speed_rpt = msg;
}

void PublishControl::callback_shift_rpt(const pacmod3_msgs::SystemRptInt::ConstPtr& msg)
{
  std::unique_lock<std::mutex> lock(shift_mutex);
  last_shift_cmd = msg->output;
}

void PublishControl::callback_turn_rpt(const pacmod3_msgs::SystemRptInt::ConstPtr& msg)
{
  std::unique_lock<std::mutex> lock(turn_mutex);
  turn_signal_rpt = msg->output;
}

void PublishControl::callback_lights_rpt(const pacmod3_msgs::SystemRptInt::ConstPtr& msg)
{
  lights_api_available = true;
  ROS_INFO("Headlights API detected");
}

void PublishControl::callback_horn_rpt(const pacmod3_msgs::SystemRptBool::ConstPtr& msg)
{
  horn_api_available = true;
  ROS_INFO("Horn API detected");
}

void PublishControl::callback_wiper_rpt(const pacmod3_msgs::SystemRptInt::ConstPtr& msg)
{
  wiper_api_available = true;
  ROS_INFO("Wiper API detected");
}

// Publishing
void PublishControl::publish_steering_message()
{
  pacmod3_msgs::SteeringCmd steer_msg;

  steer_msg.enable = local_enable;
  steer_msg.ignore_overrides = false;

  // If the enable flag just went to true, send an override clear
  if (!prev_enable && local_enable)
  {
    steer_msg.clear_override = true;
  }

  float range_scale;
  if (vehicle_type == VehicleType::POLARIS_GEM)
  {
    range_scale =
        fabs(controller->steering_value()) * (STEER_OFFSET - ROT_RANGE_SCALER_LB) + ROT_RANGE_SCALER_LB;
  }
  else
  {
    range_scale = 1.0;
  }

  // Decreases the angular rotation rate of the steering wheel when moving faster
  float speed_based_damping = 1.0;
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
      speed_based_damping =
          STEER_OFFSET - fabs((current_speed / (max_veh_speed * STEER_SCALE_FACTOR)));  // this could go negative.
    else
      speed_based_damping = 0.33333;  // clips the equation assuming 1 offset and 1.5 scale_factor

  steer_msg.command = (range_scale * max_rot_rad) * controller->steering_value();
  steer_msg.rotation_rate = steering_max_speed * speed_based_damping;
  steering_set_position_with_speed_limit_pub.publish(steer_msg);
}

void PublishControl::publish_turn_signal_message()
{
  pacmod3_msgs::SystemCmdInt turn_signal_cmd_pub_msg;

  turn_signal_cmd_pub_msg.enable = local_enable;
  turn_signal_cmd_pub_msg.ignore_overrides = false;

  // If the enable flag just went to true, send an override clear
  if (!prev_enable && local_enable)
  {
    turn_signal_cmd_pub_msg.clear_override = true;
  }

  int turn_signal_cmd = controller->turn_signal_cmd();

  if (local_enable != prev_enable)
  {
    // TODO(icolwell-as): What is special about vehicle 6?
    if (vehicle_type == VehicleType::VEHICLE_6)
      turn_signal_cmd = pacmod3_msgs::SystemCmdInt::TURN_NONE;
    else
      turn_signal_cmd = turn_signal_rpt;
  }

  // Only publish if we are requesting a different turn signal than is currently active, or we just engaged and need to
  // clear override
  if (turn_signal_cmd != turn_signal_rpt || local_enable != prev_enable)
  {
    turn_signal_cmd_pub_msg.command = turn_signal_cmd;
    turn_signal_cmd_pub.publish(turn_signal_cmd_pub_msg);
  }
}

void PublishControl::publish_shifting_message()
{
  // Only shift if brake command is higher than 25%
  if (last_brake_cmd > 0.25)
  {
    pacmod3_msgs::SystemCmdInt shift_cmd_pub_msg;
    shift_cmd_pub_msg.enable = local_enable;
    shift_cmd_pub_msg.ignore_overrides = false;

    // If the enable flag just went to true, send an override clear and a faults_clear
    if (!prev_enable && local_enable)
    {
      shift_cmd_pub_msg.clear_override = true;
    }

    int shift_cmd = controller->shift_cmd();

    // Skip if invalid (multiple buttons pressed)
    if (shift_cmd == -1)
    {
      return;
    }
    shift_cmd_pub_msg.command = shift_cmd;
    shift_cmd_pub.publish(shift_cmd_pub_msg);
  }
  else if (local_enable != prev_enable)  // If only an enable/disable button was pressed
  {
    pacmod3_msgs::SystemCmdInt shift_cmd_pub_msg;
    shift_cmd_pub_msg.enable = local_enable;
    shift_cmd_pub_msg.ignore_overrides = false;

    // If the enable flag just went to true, send an override clear
    if (!prev_enable && local_enable)
    {
      shift_cmd_pub_msg.clear_override = true;
    }

    shift_cmd_pub_msg.command = last_shift_cmd;
    shift_cmd_pub.publish(shift_cmd_pub_msg);
  }
}

void PublishControl::publish_accelerator_message()
{
  pacmod3_msgs::SystemCmdFloat accelerator_cmd_pub_msg;

  accelerator_cmd_pub_msg.enable = local_enable;
  accelerator_cmd_pub_msg.ignore_overrides = false;

  // If the enable flag just went to true, send an override clear
  if (!prev_enable && local_enable)
  {
    accelerator_cmd_pub_msg.clear_override = true;
  }

  if (vehicle_type == VehicleType::POLARIS_GEM)
  {
    accelerator_cmd_pub_msg.command =
        accel_scale_val * controller->accelerator_value() * ACCEL_SCALE_FACTOR + ACCEL_OFFSET;
  }
  else
  {
    accelerator_cmd_pub_msg.command = accel_scale_val * controller->accelerator_value();
  }

  accelerator_cmd_pub.publish(accelerator_cmd_pub_msg);
}

void PublishControl::publish_brake_message()
{
  pacmod3_msgs::SystemCmdFloat brake_msg;

  brake_msg.enable = local_enable;
  brake_msg.ignore_overrides = false;

  // If the enable flag just went to true, send an override clear
  if (!prev_enable && local_enable)
  {
    brake_msg.clear_override = true;
  }

  brake_msg.command = brake_scale_val * controller->brake_value();
  last_brake_cmd = brake_msg.command;
  brake_set_position_pub.publish(brake_msg);
}

void PublishControl::publish_lights()
{
  if (!lights_api_available)
  {
    return;
  }

  pacmod3_msgs::SystemCmdInt headlight_cmd_pub_msg;
  headlight_cmd_pub_msg.enable = local_enable;
  headlight_cmd_pub_msg.ignore_overrides = false;

  // Headlights
  if (controller->headlight_change())
  {
    // TODO(icolwell-as): What is special about vehicle 5?
    if (vehicle_type == VehicleType::VEHICLE_5)
    {
      if (headlight_state == 1)
        headlight_state = 2;
      else
        headlight_state = 1;
    }
    else
    {
      // Rotate through headlight states as button is pressed
      if (!headlight_state_change)
      {
        headlight_state++;
        headlight_state_change = true;
      }

      if (headlight_state >= NUM_HEADLIGHT_STATES)
        headlight_state = HEADLIGHT_STATE_START_VALUE;
    }

    // If the enable flag just went to true, send an override clear
    if (!prev_enable && local_enable)
    {
      headlight_cmd_pub_msg.clear_override = true;
      headlight_state = HEADLIGHT_STATE_START_VALUE;
    }
  }
  else
  {
    headlight_state_change = false;
  }

  headlight_cmd_pub_msg.command = headlight_state;
  headlight_cmd_pub.publish(headlight_cmd_pub_msg);
}

void PublishControl::publish_horn()
{
  if (!horn_api_available)
  {
    return;
  }

  pacmod3_msgs::SystemCmdBool horn_cmd_pub_msg;
  horn_cmd_pub_msg.enable = local_enable;
  horn_cmd_pub_msg.ignore_overrides = false;

  // If the enable flag just went to true, send an override clear
  if (!prev_enable && local_enable)
  {
    horn_cmd_pub_msg.clear_override = true;
  }

  horn_cmd_pub_msg.command = controller->horn_cmd();
  horn_cmd_pub.publish(horn_cmd_pub_msg);
}

void PublishControl::publish_wipers()
{
  if (!wiper_api_available)
  {
    return;
  }

  pacmod3_msgs::SystemCmdInt wiper_cmd_pub_msg;
  wiper_cmd_pub_msg.enable = local_enable;
  wiper_cmd_pub_msg.ignore_overrides = false;

  if (controller->wiper_change())
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

void PublishControl::check_is_enabled()
{
  bool state_changed = false;

  enable_mutex.lock();
  local_enable = pacmod_enable;
  enable_mutex.unlock();

  // Enable
  if (controller->enable() && !local_enable)
  {
    std_msgs::Bool bool_pub_msg;
    bool_pub_msg.data = true;
    local_enable = true;
    enable_pub.publish(bool_pub_msg);

    state_changed = true;
  }

  // Disable
  if (controller->disable() && local_enable)
  {
    std_msgs::Bool bool_pub_msg;
    bool_pub_msg.data = false;
    local_enable = false;
    enable_pub.publish(bool_pub_msg);

    state_changed = true;
  }

  if (state_changed)
  {
    std::unique_lock<std::mutex> lock(enable_mutex);
    pacmod_enable = local_enable;
  }
}
