/*
 * Unpublished Copyright (c) 2009-2021 AutonomouStuff, LLC, All Rights Reserved.
 *
 * This file is part of the PACMod ROS 1.0 driver which is released under the MIT license.
 * See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
 */

#include "pacmod_game_control/pacmod_game_control.h"

#include <unordered_map>

#include <pacmod3_msgs/SteeringCmd.h>

void GameControl::init()
{
  if (run_startup_checks_error())
  {
    ros::shutdown();
  }

  // Pubs
  enable_pub_ = n.advertise<std_msgs::Bool>("pacmod/enable", 20);
  turn_signal_cmd_pub_ = n.advertise<pacmod3_msgs::SystemCmdInt>("pacmod/turn_cmd", 20);
  headlight_cmd_pub_ = n.advertise<pacmod3_msgs::SystemCmdInt>("pacmod/headlight_cmd", 20);
  horn_cmd_pub_ = n.advertise<pacmod3_msgs::SystemCmdBool>("pacmod/horn_cmd", 20);
  wiper_cmd_pub_ = n.advertise<pacmod3_msgs::SystemCmdInt>("pacmod/wiper_cmd", 20);
  shift_cmd_pub_ = n.advertise<pacmod3_msgs::SystemCmdInt>("pacmod/shift_cmd", 20);
  accelerator_cmd_pub_ = n.advertise<pacmod3_msgs::SystemCmdFloat>("pacmod/accel_cmd", 20);
  steering_cmd_pub_ = n.advertise<pacmod3_msgs::SteeringCmd>("pacmod/steering_cmd", 20);
  brake_cmd_pub_ = n.advertise<pacmod3_msgs::SystemCmdFloat>("pacmod/brake_cmd", 20);

  // Subs
  joy_sub_ = n.subscribe("joy", 1000, &GameControl::callback_control, this);
  speed_sub_ = n.subscribe("pacmod/vehicle_speed_rpt", 20, &GameControl::callback_veh_speed, this);
  enable_sub_ = n.subscribe("pacmod/enabled", 20, &GameControl::callback_pacmod_enable, this);
  shift_sub_ = n.subscribe("pacmod/shift_rpt", 20, &GameControl::callback_shift_rpt, this);
  turn_sub_ = n.subscribe("pacmod/turn_rpt", 20, &GameControl::callback_turn_rpt, this);
  lights_sub_ = n.subscribe("pacmod/headlight_rpt", 10, &GameControl::callback_lights_rpt, this);
  horn_sub_ = n.subscribe("pacmod/horn_rpt", 10, &GameControl::callback_horn_rpt, this);
  wiper_sub_ = n.subscribe("pacmod/wiper_rpt", 10, &GameControl::callback_wiper_rpt, this);
}

void GameControl::callback_control(const sensor_msgs::Joy::ConstPtr& msg)
{
  controller_->set_controller_input(*msg);
  try
  {
    // Only send messages when enabled, or when the state changes between enabled/disabled
    check_is_enabled();

    if (local_enable_ == true || local_enable_ != prev_enable_)
    {
      publish_steering_message();
      publish_turn_signal_message();
      publish_shifting_message();
      publish_accelerator_message();
      publish_brake_message();
      publish_lights();
      publish_horn();
      publish_wipers();
    }

    prev_enable_ = local_enable_;
  }
  catch (const std::out_of_range& oor)
  {
    ROS_ERROR("An out-of-range exception was caught. This probably means you selected the wrong controller_type type.");
  }
}

void GameControl::callback_pacmod_enable(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data == false && last_pacmod_state_ == true)
    prev_enable_ = false;

  std::unique_lock<std::mutex> lock(enable_mutex);
  pacmod_enable_ = msg->data;

  last_pacmod_state_ = msg->data;
}

// Feedback callbacks
void GameControl::callback_veh_speed(const pacmod3_msgs::VehicleSpeedRpt::ConstPtr& msg)
{
  std::unique_lock<std::mutex> lock(speed_mutex);
  veh_speed_rpt_ = msg;
}

void GameControl::callback_shift_rpt(const pacmod3_msgs::SystemRptInt::ConstPtr& msg)
{
  std::unique_lock<std::mutex> lock(shift_mutex);
  shift_rpt_ = msg->output;
}

void GameControl::callback_turn_rpt(const pacmod3_msgs::SystemRptInt::ConstPtr& msg)
{
  std::unique_lock<std::mutex> lock(turn_mutex);
  turn_signal_rpt_ = msg->output;
}

void GameControl::callback_lights_rpt(const pacmod3_msgs::SystemRptInt::ConstPtr& msg)
{
  lights_api_available_ = true;
  ROS_INFO("Headlights API detected");
}

void GameControl::callback_horn_rpt(const pacmod3_msgs::SystemRptBool::ConstPtr& msg)
{
  horn_api_available_ = true;
  ROS_INFO("Horn API detected");
}

void GameControl::callback_wiper_rpt(const pacmod3_msgs::SystemRptInt::ConstPtr& msg)
{
  wiper_api_available_ = true;
  ROS_INFO("Wiper API detected");
}

// Publishing
void GameControl::publish_steering_message()
{
  pacmod3_msgs::SteeringCmd steer_msg;

  steer_msg.enable = local_enable_;
  steer_msg.ignore_overrides = false;

  // If the enable flag just went to true, send an override clear
  if (!prev_enable_ && local_enable_)
  {
    steer_msg.clear_override = true;
  }

  float range_scale;
  if (vehicle_type_ == VehicleType::POLARIS_GEM)
  {
    range_scale =
        fabs(controller_->steering_value()) * (STEER_OFFSET - ROT_RANGE_SCALER_LB) + ROT_RANGE_SCALER_LB;
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

  if (veh_speed_rpt_ != NULL)
    speed_valid = veh_speed_rpt_->vehicle_speed_valid;

  if (speed_valid)
    current_speed = veh_speed_rpt_->vehicle_speed;

  speed_mutex.unlock();

  if (speed_valid)
    if (current_speed < max_veh_speed_)
      speed_based_damping =
          STEER_OFFSET - fabs((current_speed / (max_veh_speed_ * STEER_SCALE_FACTOR)));  // this could go negative.
    else
      speed_based_damping = 0.33333;  // clips the equation assuming 1 offset and 1.5 scale_factor

  steer_msg.command = (range_scale * max_rot_rad_) * controller_->steering_value();
  steer_msg.rotation_rate = steering_max_speed_ * speed_based_damping;
  steering_cmd_pub_.publish(steer_msg);
}

void GameControl::publish_turn_signal_message()
{
  pacmod3_msgs::SystemCmdInt turn_signal_cmd_pub_msg;

  turn_signal_cmd_pub_msg.enable = local_enable_;
  turn_signal_cmd_pub_msg.ignore_overrides = false;

  // If the enable flag just went to true, send an override clear
  if (!prev_enable_ && local_enable_)
  {
    turn_signal_cmd_pub_msg.clear_override = true;
  }

  int turn_signal_cmd = controller_->turn_signal_cmd();

  if (local_enable_ != prev_enable_)
  {
    // TODO(icolwell-as): What is special about vehicle 6?
    if (vehicle_type_ == VehicleType::VEHICLE_6)
      turn_signal_cmd = pacmod3_msgs::SystemCmdInt::TURN_NONE;
    else
      turn_signal_cmd = turn_signal_rpt_;
  }

  // Only publish if we are requesting a different turn signal than is currently active, or we just engaged and need to
  // clear override
  if (turn_signal_cmd != turn_signal_rpt_ || local_enable_ != prev_enable_)
  {
    turn_signal_cmd_pub_msg.command = turn_signal_cmd;
    turn_signal_cmd_pub_.publish(turn_signal_cmd_pub_msg);
  }
}

void GameControl::publish_shifting_message()
{
  // Only shift if brake command is higher than 25%
  if (last_brake_cmd_ > 0.25)
  {
    pacmod3_msgs::SystemCmdInt shift_cmd_pub_msg;
    shift_cmd_pub_msg.enable = local_enable_;
    shift_cmd_pub_msg.ignore_overrides = false;

    // If the enable flag just went to true, send an override clear and a faults_clear
    if (!prev_enable_ && local_enable_)
    {
      shift_cmd_pub_msg.clear_override = true;
    }

    int shift_cmd = controller_->shift_cmd();

    // Skip if invalid (multiple buttons pressed)
    if (shift_cmd == -1)
    {
      return;
    }
    shift_cmd_pub_msg.command = shift_cmd;
    shift_cmd_pub_.publish(shift_cmd_pub_msg);
  }
  else if (local_enable_ != prev_enable_)  // If only an enable/disable button was pressed
  {
    pacmod3_msgs::SystemCmdInt shift_cmd_pub_msg;
    shift_cmd_pub_msg.enable = local_enable_;
    shift_cmd_pub_msg.ignore_overrides = false;

    // If the enable flag just went to true, send an override clear
    if (!prev_enable_ && local_enable_)
    {
      shift_cmd_pub_msg.clear_override = true;
    }

    shift_cmd_pub_msg.command = shift_rpt_;
    shift_cmd_pub_.publish(shift_cmd_pub_msg);
  }
}

void GameControl::publish_accelerator_message()
{
  pacmod3_msgs::SystemCmdFloat accelerator_cmd_pub_msg;

  accelerator_cmd_pub_msg.enable = local_enable_;
  accelerator_cmd_pub_msg.ignore_overrides = false;

  // If the enable flag just went to true, send an override clear
  if (!prev_enable_ && local_enable_)
  {
    accelerator_cmd_pub_msg.clear_override = true;
  }

  if (vehicle_type_ == VehicleType::POLARIS_GEM)
  {
    accelerator_cmd_pub_msg.command =
        accel_scale_val_ * controller_->accelerator_value() * ACCEL_SCALE_FACTOR + ACCEL_OFFSET;
  }
  else
  {
    accelerator_cmd_pub_msg.command = accel_scale_val_ * controller_->accelerator_value();
  }

  accelerator_cmd_pub_.publish(accelerator_cmd_pub_msg);
}

void GameControl::publish_brake_message()
{
  pacmod3_msgs::SystemCmdFloat brake_msg;

  brake_msg.enable = local_enable_;
  brake_msg.ignore_overrides = false;

  // If the enable flag just went to true, send an override clear
  if (!prev_enable_ && local_enable_)
  {
    brake_msg.clear_override = true;
  }

  brake_msg.command = brake_scale_val_ * controller_->brake_value();
  last_brake_cmd_ = brake_msg.command;
  brake_cmd_pub_.publish(brake_msg);
}

void GameControl::publish_lights()
{
  if (!lights_api_available_)
  {
    return;
  }

  pacmod3_msgs::SystemCmdInt headlight_cmd_pub_msg;
  headlight_cmd_pub_msg.enable = local_enable_;
  headlight_cmd_pub_msg.ignore_overrides = false;

  // Headlights
  if (controller_->headlight_change())
  {
    // Rotate through headlight states
    headlight_state_++;

    if (headlight_state_ >= NUM_HEADLIGHT_STATES)
      headlight_state_ = HEADLIGHT_STATE_START_VALUE;

    // If the enable flag just went to true, send an override clear
    if (!prev_enable_ && local_enable_)
    {
      headlight_cmd_pub_msg.clear_override = true;
      headlight_state_ = HEADLIGHT_STATE_START_VALUE;
    }
  }

  headlight_cmd_pub_msg.command = headlight_state_;
  headlight_cmd_pub_.publish(headlight_cmd_pub_msg);
}

void GameControl::publish_horn()
{
  if (!horn_api_available_)
  {
    return;
  }

  pacmod3_msgs::SystemCmdBool horn_cmd_pub_msg;
  horn_cmd_pub_msg.enable = local_enable_;
  horn_cmd_pub_msg.ignore_overrides = false;

  // If the enable flag just went to true, send an override clear
  if (!prev_enable_ && local_enable_)
  {
    horn_cmd_pub_msg.clear_override = true;
  }

  horn_cmd_pub_msg.command = controller_->horn_cmd();
  horn_cmd_pub_.publish(horn_cmd_pub_msg);
}

void GameControl::publish_wipers()
{
  if (!wiper_api_available_)
  {
    return;
  }

  pacmod3_msgs::SystemCmdInt wiper_cmd_pub_msg;
  wiper_cmd_pub_msg.enable = local_enable_;
  wiper_cmd_pub_msg.ignore_overrides = false;

  if (controller_->wiper_change())
  {
    // Rotate through wiper states as button is pressed
    wiper_state_++;

    if (wiper_state_ >= NUM_WIPER_STATES)
      wiper_state_ = WIPER_STATE_START_VALUE;

    // If the enable flag just went to true, send an override clear
    if (!prev_enable_ && local_enable_)
    {
      wiper_cmd_pub_msg.clear_override = true;
      wiper_state_ = WIPER_STATE_START_VALUE;
    }

    wiper_cmd_pub_msg.command = wiper_state_;
  }

  wiper_cmd_pub_.publish(wiper_cmd_pub_msg);
}

void GameControl::check_is_enabled()
{
  bool state_changed = false;

  enable_mutex.lock();
  local_enable_ = pacmod_enable_;
  enable_mutex.unlock();

  // Enable
  if (controller_->enable() && !local_enable_)
  {
    std_msgs::Bool bool_pub_msg;
    bool_pub_msg.data = true;
    local_enable_ = true;
    enable_pub_.publish(bool_pub_msg);

    state_changed = true;
  }

  // Disable
  if (controller_->disable() && local_enable_)
  {
    std_msgs::Bool bool_pub_msg;
    bool_pub_msg.data = false;
    local_enable_ = false;
    enable_pub_.publish(bool_pub_msg);

    state_changed = true;
  }

  if (state_changed)
  {
    std::unique_lock<std::mutex> lock(enable_mutex);
    pacmod_enable_ = local_enable_;
  }
}
