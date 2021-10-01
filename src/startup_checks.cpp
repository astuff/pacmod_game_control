/*
 * Unpublished Copyright (c) 2009-2021 AutonomouStuff, LLC, All Rights Reserved.
 *
 * This file is part of the PACMod ROS 1.0 driver which is released under the MIT license.
 * See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
 */
/*
 * Series of startup checks for the pacmod game control node.
 */

#include "pacmod_game_control/publish_control.h"

#include <memory>
#include <string>

bool PublishControl::run_startup_checks_error()
{
  bool willExit = false;

  ros::NodeHandle pnh("~");

  // Run startup checks
  willExit = willExit || check_steering_stick_left_right(pnh);
  willExit = willExit || check_vehicle_type(pnh);
  willExit = willExit || check_controller_type(pnh);
  willExit = willExit || check_scale_values(pnh);

  return willExit;
}

// Check which steering stick we should use on the joypad
bool PublishControl::check_steering_stick_left_right(const ros::NodeHandle& nodeH)
{
  std::string steering_stick_string;
  bool exit = false;

  if (nodeH.getParam("steering_stick", steering_stick_string))
  {
    ROS_INFO("Got steering_stick: %s", steering_stick_string.c_str());

    if (steering_stick_string == "LEFT")
    {
      steering_axis = JoyAxis::LEFT_STICK_LR;
    }
    else if (steering_stick_string == "RIGHT")
    {
      steering_axis = JoyAxis::RIGHT_STICK_LR;
    }
    else
    {
      ROS_ERROR("steering_stick is invalid. Exiting.");
      exit = true;
    }
  }
  else
  {
    ROS_ERROR("Parameter steering_stick is missing. Exiting.");
    exit = true;
  }

  return exit;
}

bool PublishControl::check_vehicle_type(const ros::NodeHandle& nodeH)
{
  bool exit = false;
  std::string vehicle_type_string;
  VehicleType vehicle_type;

  if (nodeH.getParam("pacmod_vehicle_type", vehicle_type_string))
  {
    ROS_INFO("Got pacmod_vehicle_type: %s", vehicle_type_string.c_str());

    if (vehicle_type_string == "POLARIS_GEM")
      vehicle_type = VehicleType::POLARIS_GEM;
    else if (vehicle_type_string == "POLARIS_RANGER")
      vehicle_type = VehicleType::POLARIS_RANGER;
    else if (vehicle_type_string == "LEXUS_RX_450H")
      vehicle_type = VehicleType::LEXUS_RX_450H;
    else if (vehicle_type_string == "JUPITER_SPIRIT")
      vehicle_type = VehicleType::JUPITER_SPIRIT;
    else if (vehicle_type_string == "INTERNATIONAL_PROSTAR_122")
      vehicle_type = VehicleType::INTERNATIONAL_PROSTAR;
    else if (vehicle_type_string == "FREIGHTLINER_CASCADIA")
      vehicle_type = VehicleType::FREIGHTLINER_CASCADIA;
    else if (vehicle_type_string == "VEHICLE_4")
      vehicle_type = VehicleType::VEHICLE_4;
    else if (vehicle_type_string == "VEHICLE_5")
      vehicle_type = VehicleType::VEHICLE_5;
    else if (vehicle_type_string == "VEHICLE_6")
      vehicle_type = VehicleType::VEHICLE_6;
    else
    {
      ROS_ERROR("pacmod_vehicle_type is invalid");
      exit = true;
    }
  }
  else
  {
    ROS_ERROR("Parameter pacmod_vehicle_type is missing. Exiting.");
    exit = true;
  }

  if (vehicle_type == VehicleType::LEXUS_RX_450H)
    max_rot_rad = MAX_ROT_RAD_VEHICLE2;
  else if (vehicle_type == VehicleType::FREIGHTLINER_CASCADIA)
    max_rot_rad = MAX_ROT_RAD_FREIGHTLINER_CASCADIA;
  else if (vehicle_type == VehicleType::JUPITER_SPIRIT)
    max_rot_rad = MAX_ROT_RAD_JUPITER_SPIRIT;
  else if (vehicle_type == VehicleType::VEHICLE_4)
    max_rot_rad = MAX_ROT_RAD_VEHICLE4;
  else if (vehicle_type == VehicleType::VEHICLE_5)
    max_rot_rad = MAX_ROT_RAD_VEHICLE5;
  else if (vehicle_type == VehicleType::VEHICLE_6)
    max_rot_rad = MAX_ROT_RAD_VEHICLE6;

  vehicle_type = vehicle_type;

  return exit;
}

bool PublishControl::check_controller_type(const ros::NodeHandle& nodeH)
{
  std::string controller_string;
  bool exit = false;

  if (nodeH.getParam("controller_type", controller_string))
  {
    ROS_INFO("Got controller_type: %s", controller_string.c_str());

    if (controller_string == "LOGITECH_F310" || controller_string == "XBOX_ONE")
    {
      controller_type = (controller_string == "LOGITECH_F310") ? GamepadType::LOGITECH_F310 : GamepadType::XBOX_ONE;
      controller = std::make_unique<Controller>();
    }
    else if (controller_string == "HRI_SAFE_REMOTE")
    {
      controller_type = GamepadType::HRI_SAFE_REMOTE;
      controller = std::make_unique<HriSafeController>();
    }
    else if (controller_string == "LOGITECH_G29")
    {
      controller_type = GamepadType::LOGITECH_G29;
      controller = std::make_unique<LogitechG29Controller>();

      // Set to match the G29 controller_type's max center-to-lock steering angle (radians).
      max_rot_rad = 7.85;
    }
    else
    {
      ROS_ERROR("Provided controller_type is invalid. Exiting.");
      exit = true;
    }
  }
  else
  {
    ROS_ERROR("Parameter controller_type is missing. Exiting.");
    exit = true;
  }

  return exit;
}

bool PublishControl::check_scale_values(const ros::NodeHandle& nodeH)
{
  bool exit = false;

  if (nodeH.getParam("steering_max_speed", steering_max_speed))
  {
    ROS_INFO("Got steering_max_speed: %f", steering_max_speed);

    if (steering_max_speed <= 0)
    {
      ROS_ERROR("Parameter steering_max_speed is invalid. Exiting.");
      exit = true;
    }
  }
  else
  {
    ROS_ERROR("Parameter steering_max_speed_scale_val is missing. Exiting.");
    exit = true;
  }

  if (nodeH.getParam("max_veh_speed", max_veh_speed))
  {
    ROS_INFO("Got max_veh_speed: %f", max_veh_speed);

    if (max_veh_speed <= 0)
    {
      ROS_ERROR("Parameter max_veh_speed is invalid. Exiting.");
      exit = true;
    }
  }
  else
  {
    ROS_ERROR("Parameter max_veh_speed is missing. Exiting.");
    exit = true;
  }

  if (nodeH.getParam("accel_scale_val", accel_scale_val))
  {
    ROS_INFO("Got accel_scale_val: %f", accel_scale_val);

    if (accel_scale_val <= 0 || accel_scale_val > 1.0)
    {
      ROS_ERROR("Parameter accel_scale_val is invalid. Exiting.");
      exit = true;
    }
  }
  else
  {
    ROS_ERROR("Parameter accel_scale_val is missing. Exiting.");
    exit = true;
  }

  if (nodeH.getParam("brake_scale_val", brake_scale_val))
  {
    ROS_INFO("Got brake_scale_val: %f", brake_scale_val);

    if (brake_scale_val <= 0 || brake_scale_val > 1.0)
    {
      ROS_ERROR("Parameter brake_scale_val is invalid. Exiting.");
      exit = true;
    }
  }
  else
  {
    ROS_ERROR("Parameter brake_scale_val is missing. Exiting.");
    exit = true;
  }

  return exit;
}
