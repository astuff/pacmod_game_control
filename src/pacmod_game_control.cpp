/*
 * Unpublished Copyright (c) 2009-2021 AutonomouStuff, LLC, All Rights Reserved.
 *
 * This file is part of the PACMod ROS 1.0 driver which is released under the MIT license.
 * See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
 */

#include "pacmod_game_control/pacmod_game_control.h"

#include <unordered_map>

#include <pacmod3_msgs/SteeringCmd.h>

void GameControl::Init()
{
  if (RunStartupChecks())
  {
    ros::shutdown();
  }

  // Pubs
  turn_signal_cmd_pub_ = nh_.advertise<pacmod3_msgs::SystemCmdInt>("pacmod/turn_cmd", 20);
  headlight_cmd_pub_ = nh_.advertise<pacmod3_msgs::SystemCmdInt>("pacmod/headlight_cmd", 20);
  horn_cmd_pub_ = nh_.advertise<pacmod3_msgs::SystemCmdBool>("pacmod/horn_cmd", 20);
  wiper_cmd_pub_ = nh_.advertise<pacmod3_msgs::SystemCmdInt>("pacmod/wiper_cmd", 20);
  shift_cmd_pub_ = nh_.advertise<pacmod3_msgs::SystemCmdInt>("pacmod/shift_cmd", 20);
  accelerator_cmd_pub_ = nh_.advertise<pacmod3_msgs::SystemCmdFloat>("pacmod/accel_cmd", 20);
  steering_cmd_pub_ = nh_.advertise<pacmod3_msgs::SteeringCmd>("pacmod/steering_cmd", 20);
  brake_cmd_pub_ = nh_.advertise<pacmod3_msgs::SystemCmdFloat>("pacmod/brake_cmd", 20);

  // Subs
  joy_sub_ = nh_.subscribe("joy", 1000, &GameControl::GamepadCb, this);
  speed_sub_ = nh_.subscribe("pacmod/vehicle_speed_rpt", 20, &GameControl::VehicleSpeedCb, this);
  enable_sub_ = nh_.subscribe("pacmod/enabled", 20, &GameControl::PacmodEnabledCb, this);
  lights_sub_ = nh_.subscribe("pacmod/headlight_rpt", 10, &GameControl::LightsRptCb, this);
  horn_sub_ = nh_.subscribe("pacmod/horn_rpt", 10, &GameControl::HornRptCb, this);
  wiper_sub_ = nh_.subscribe("pacmod/wiper_rpt", 10, &GameControl::WiperRptCb, this);
}

void GameControl::GamepadCb(const sensor_msgs::Joy::ConstPtr& msg)
{
  if (controller_ == nullptr)
  {
    return;
  }

  controller_->set_controller_input(*msg);
  try
  {
    // Enable
    if (controller_->enable() && !pacmod_enabled_rpt_)
    {
      enable_cmd_ = true;
      clear_override_cmd_ = true;
    }

    // Disable
    if (controller_->disable() && pacmod_enabled_rpt_)
    {
      enable_cmd_ = false;
    }

    // Only send messages when enabled, or when the state changes between enabled/disabled
    if (pacmod_enabled_rpt_ || enable_cmd_)
    {
      PublishCommands();
      clear_override_cmd_ = false;
    }
  }
  catch (const std::out_of_range& oor)
  {
    ROS_ERROR("An out-of-range exception was caught. This probably means you selected the wrong controller_type type.");
  }
}

void GameControl::PacmodEnabledCb(const std_msgs::Bool::ConstPtr& msg)
{
  if (controller_ == nullptr)
  {
    return;
  }

  bool prev_pacmod_enabled_rpt = pacmod_enabled_rpt_;
  pacmod_enabled_rpt_ = msg->data;

  // Stop trying to enable if pacmod just disabled from an override or something
  if (prev_pacmod_enabled_rpt && !pacmod_enabled_rpt_)
  {
    enable_cmd_ = false;
    PublishCommands();
  }
}

// Feedback callbacks
void GameControl::VehicleSpeedCb(const pacmod3_msgs::VehicleSpeedRpt::ConstPtr& msg)
{
  veh_speed_rpt_ = msg;
}

void GameControl::LightsRptCb(const pacmod3_msgs::SystemRptInt::ConstPtr& msg)
{
  if (!lights_api_available_)
  {
    lights_api_available_ = true;
    ROS_INFO("Headlights API detected");
  }
}

void GameControl::HornRptCb(const pacmod3_msgs::SystemRptBool::ConstPtr& msg)
{
  if (!horn_api_available_)
  {
    horn_api_available_ = true;
    ROS_INFO("Horn API detected");
  }
}

void GameControl::WiperRptCb(const pacmod3_msgs::SystemRptInt::ConstPtr& msg)
{
  if (!wiper_api_available_)
  {
    wiper_api_available_ = true;
    ROS_INFO("Wiper API detected");
  }
}

// Publishing
void GameControl::PublishCommands()
{
  PublishAccelerator();
  PublishBrake();
  PublishSteering();
  PublishShifting();
  PublishTurnSignal();
  PublishLights();
  PublishHorn();
  PublishWipers();
}

void GameControl::PublishAccelerator()
{
  pacmod3_msgs::SystemCmdFloat accelerator_cmd_pub_msg;

  accelerator_cmd_pub_msg.enable = enable_cmd_;
  accelerator_cmd_pub_msg.clear_override = clear_override_cmd_;
  accelerator_cmd_pub_msg.ignore_overrides = false;

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

void GameControl::PublishBrake()
{
  pacmod3_msgs::SystemCmdFloat brake_msg;

  brake_msg.enable = enable_cmd_;
  brake_msg.clear_override = clear_override_cmd_;
  brake_msg.ignore_overrides = false;

  brake_msg.command = brake_scale_val_ * controller_->brake_value();
  last_brake_cmd_ = brake_msg.command;
  brake_cmd_pub_.publish(brake_msg);
}

void GameControl::PublishSteering()
{
  pacmod3_msgs::SteeringCmd steer_msg;

  steer_msg.enable = enable_cmd_;
  steer_msg.clear_override = clear_override_cmd_;
  steer_msg.ignore_overrides = false;

  float range_scale;
  if (vehicle_type_ == VehicleType::POLARIS_GEM)
  {
    range_scale = fabs(controller_->steering_value()) * (STEER_OFFSET - ROT_RANGE_SCALER_LB) + ROT_RANGE_SCALER_LB;
  }
  else
  {
    range_scale = 1.0;
  }

  // Decreases the angular rotation rate of the steering wheel when moving faster
  float speed_based_damping = 1.0;
  bool speed_valid = false;
  float current_speed = 0.0;

  if (veh_speed_rpt_ != NULL)
    speed_valid = veh_speed_rpt_->vehicle_speed_valid;

  if (speed_valid)
    current_speed = veh_speed_rpt_->vehicle_speed;

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

void GameControl::PublishShifting()
{
  pacmod3_msgs::SystemCmdInt shift_cmd_pub_msg;
  shift_cmd_pub_msg.enable = enable_cmd_;
  shift_cmd_pub_msg.clear_override = clear_override_cmd_;
  shift_cmd_pub_msg.ignore_overrides = false;

  int shift_cmd = pacmod3_msgs::SystemCmdInt::SHIFT_NONE;

  // Only shift if brake command is higher than 25%
  if (last_brake_cmd_ > 0.5)
  {
    shift_cmd = controller_->shift_cmd();
  }

  shift_cmd_pub_msg.command = shift_cmd;
  shift_cmd_pub_.publish(shift_cmd_pub_msg);
}

void GameControl::PublishTurnSignal()
{
  pacmod3_msgs::SystemCmdInt turn_signal_cmd_pub_msg;

  turn_signal_cmd_pub_msg.enable = enable_cmd_;
  turn_signal_cmd_pub_msg.clear_override = clear_override_cmd_;
  turn_signal_cmd_pub_msg.ignore_overrides = false;

  int turn_signal_cmd = controller_->turn_signal_cmd();

  turn_signal_cmd_pub_msg.command = turn_signal_cmd;
  turn_signal_cmd_pub_.publish(turn_signal_cmd_pub_msg);
}

void GameControl::PublishLights()
{
  if (!lights_api_available_)
  {
    return;
  }

  pacmod3_msgs::SystemCmdInt headlight_cmd_pub_msg;
  headlight_cmd_pub_msg.enable = enable_cmd_;
  headlight_cmd_pub_msg.clear_override = clear_override_cmd_;
  headlight_cmd_pub_msg.ignore_overrides = false;

  // Headlights
  if (controller_->headlight_change())
  {
    // Rotate through headlight states
    headlight_cmd_++;

    if (headlight_cmd_ >= NUM_HEADLIGHT_STATES)
    {
      headlight_cmd_ = HEADLIGHT_STATE_START_VALUE;
    }

    // Reset
    if (clear_override_cmd_)
    {
      headlight_cmd_ = HEADLIGHT_STATE_START_VALUE;
    }
  }

  headlight_cmd_pub_msg.command = headlight_cmd_;
  headlight_cmd_pub_.publish(headlight_cmd_pub_msg);
}

void GameControl::PublishHorn()
{
  if (!horn_api_available_)
  {
    return;
  }

  pacmod3_msgs::SystemCmdBool horn_cmd_pub_msg;
  horn_cmd_pub_msg.enable = enable_cmd_;
  horn_cmd_pub_msg.clear_override = clear_override_cmd_;
  horn_cmd_pub_msg.ignore_overrides = false;

  horn_cmd_pub_msg.command = controller_->horn_cmd();
  horn_cmd_pub_.publish(horn_cmd_pub_msg);
}

void GameControl::PublishWipers()
{
  if (!wiper_api_available_)
  {
    return;
  }

  pacmod3_msgs::SystemCmdInt wiper_cmd_pub_msg;
  wiper_cmd_pub_msg.enable = enable_cmd_;
  wiper_cmd_pub_msg.clear_override = clear_override_cmd_;
  wiper_cmd_pub_msg.ignore_overrides = false;

  if (controller_->wiper_change())
  {
    // Rotate through wiper states as button is pressed
    wiper_cmd_++;

    if (wiper_cmd_ >= NUM_WIPER_STATES)
      wiper_cmd_ = WIPER_STATE_START_VALUE;

    // Reset
    if (clear_override_cmd_)
    {
      wiper_cmd_ = WIPER_STATE_START_VALUE;
    }

    wiper_cmd_pub_msg.command = wiper_cmd_;
  }

  wiper_cmd_pub_.publish(wiper_cmd_pub_msg);
}
