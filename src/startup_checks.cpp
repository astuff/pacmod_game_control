/*
 * Unpublished Copyright (c) 2009-2021 AutonomouStuff, LLC, All Rights Reserved.
 *
 * This file is part of the PACMod ROS 1.0 driver which is released under the MIT license.
 * See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
 */
/*
 * Series of startup checks for the pacmod game control node.
 */

#include "pacmod_game_control/pacmod_game_control_node.h"

#include <memory>
#include <string>

bool GameControlNode::RunStartupChecks()
{
  bool willExit = false;

  // Run startup checks
  willExit = willExit || CheckVehicleType();
  willExit = willExit || CheckControllerType();
  willExit = willExit || CheckScaleValues();

  return willExit;
}

bool GameControlNode::CheckVehicleType()
{
  bool exit = false;

  std::string vehicle_type_string = this->declare_parameter("pacmod_vehicle_type", "LEXUS_RX_450H");
  RCLCPP_INFO(this->get_logger(), "Loaded pacmod_vehicle_type: %s", vehicle_type_string.c_str());
  if (vehicle_type_string == "POLARIS_GEM")
  {
    vehicle_type_ = VehicleType::POLARIS_GEM;
  }
  else if (vehicle_type_string == "POLARIS_RANGER")
  {
    vehicle_type_ = VehicleType::POLARIS_RANGER;
  }
  else if (vehicle_type_string == "LEXUS_RX_450H")
  {
    vehicle_type_ = VehicleType::LEXUS_RX_450H;
    max_rot_rad_ = MAX_ROT_RAD_LEXUS_RX_450H;
  }
  else if (vehicle_type_string == "JUPITER_SPIRIT")
  {
    vehicle_type_ = VehicleType::JUPITER_SPIRIT;
    max_rot_rad_ = MAX_ROT_RAD_JUPITER_SPIRIT;
  }
  else if (vehicle_type_string == "INTERNATIONAL_PROSTAR_122")
  {
    vehicle_type_ = VehicleType::INTERNATIONAL_PROSTAR;
  }
  else if (vehicle_type_string == "FREIGHTLINER_CASCADIA")
  {
    vehicle_type_ = VehicleType::FREIGHTLINER_CASCADIA;
    max_rot_rad_ = MAX_ROT_RAD_FREIGHTLINER_CASCADIA;
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "pacmod_vehicle_type is invalid");
    exit = true;
  }

  return exit;
}

bool GameControlNode::CheckControllerType()
{
  bool exit = false;

  std::string controller_string = this->declare_parameter("controller_type", "LOGITECH_F310");
  RCLCPP_INFO(this->get_logger(), "Loaded controller_type: %s", controller_string.c_str());

  if (controller_string == "LOGITECH_F310" || controller_string == "XBOX_ONE")
  {
    controller_ = std::make_unique<controllers::Controller>();
  }
  else if (controller_string == "HRI_SAFE_REMOTE")
  {
    controller_ = std::make_unique<controllers::HriSafeController>();
  }
  else if (controller_string == "LOGITECH_G29")
  {
    controller_ = std::make_unique<controllers::LogitechG29Controller>();

    // Set to match the G29 controller_type's max center-to-lock steering angle (radians).
    max_rot_rad_ = 7.85;
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Provided controller_type is invalid. Exiting.");
    exit = true;
  }

  return exit;
}

bool GameControlNode::CheckScaleValues()
{
  bool exit = false;

  steering_max_speed_ = this->declare_parameter("steering_max_speed", 3.3);
  RCLCPP_INFO(this->get_logger(), "Loaded steering_max_speed: %f", steering_max_speed_);
  if (steering_max_speed_ <= 0)
  {
    RCLCPP_ERROR(this->get_logger(), "Parameter steering_max_speed is invalid. Exiting.");
    exit = true;
  }

  max_veh_speed_ = this->declare_parameter("max_veh_speed", 11.176);
  RCLCPP_INFO(this->get_logger(), "Loaded max_veh_speed: %f", max_veh_speed_);
  if (max_veh_speed_ <= 0)
  {
    RCLCPP_ERROR(this->get_logger(), "Parameter max_veh_speed is invalid. Exiting.");
    exit = true;
  }

  accel_scale_val_ = this->declare_parameter("accel_scale_val", 1.0);
  RCLCPP_INFO(this->get_logger(), "Loaded accel_scale_val: %f", accel_scale_val_);
  if (accel_scale_val_ <= 0 || accel_scale_val_ > 1.0)
  {
    RCLCPP_ERROR(this->get_logger(), "Parameter accel_scale_val is invalid. Exiting.");
    exit = true;
  }

  brake_scale_val_ = this->declare_parameter("brake_scale_val", 1.0);
  RCLCPP_INFO(this->get_logger(), "Loaded brake_scale_val: %f", brake_scale_val_);
  if (brake_scale_val_ <= 0 || brake_scale_val_ > 1.0)
  {
    RCLCPP_ERROR(this->get_logger(), "Parameter brake_scale_val is invalid. Exiting.");
    exit = true;
  }

  return exit;
}
