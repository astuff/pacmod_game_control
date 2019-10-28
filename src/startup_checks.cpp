/*
* Unpublished Copyright (c) 2009-2018 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the PACMod ROS 1.0 driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/
/*
 * Series of startup checks for the pacmod game control node.
 */

#include "pacmod_game_control/startup_checks.h"

#include <string>

// Check which steering stick we should use on the joypad
bool AS::Joystick::check_steering_stick_left_right(ros::NodeHandle * nodeH)
{
  std::string steering_stick_string;
  bool exit = false;

  if (nodeH->getParam("steering_stick", steering_stick_string))
  {
    ROS_INFO("Got steering_stick: %s", steering_stick_string.c_str());

    if (steering_stick_string == "LEFT")
    {
      PublishControl::steering_axis = LEFT_STICK_LR;
    }
    else if (steering_stick_string == "RIGHT")
    {
      PublishControl::steering_axis = RIGHT_STICK_LR;
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

bool AS::Joystick::check_vehicle_type(ros::NodeHandle * nodeH)
{
  bool exit = false;
  std::string vehicle_type_string;
  int vehicle_type = -1;

  if (nodeH->getParam("pacmod_vehicle_type", vehicle_type_string))
  {
    ROS_INFO("Got pacmod_vehicle_type: %s", vehicle_type_string.c_str());

    if (vehicle_type_string == "POLARIS_GEM")
      vehicle_type = POLARIS_GEM;
    else if (vehicle_type_string == "POLARIS_RANGER")
      vehicle_type = POLARIS_RANGER;
    else if (vehicle_type_string == "LEXUS_RX_450H")
      vehicle_type = LEXUS_RX_450H;
    else if (vehicle_type_string == "JUPITER_SPIRIT")
      vehicle_type = JUPITER_SPIRIT;
    else if (vehicle_type_string == "INTERNATIONAL_PROSTAR_122")
      vehicle_type = INTERNATIONAL_PROSTAR;
    else if (vehicle_type_string == "FREIGHTLINER_CASCADIA")
      vehicle_type = FREIGHTLINER_CASCADIA;
    else if (vehicle_type_string == "VEHICLE_4")
      vehicle_type = VEHICLE_4;
    else if (vehicle_type_string == "VEHICLE_5")
      vehicle_type = VEHICLE_5;
    else if (vehicle_type_string == "VEHICLE_6")
      vehicle_type = VEHICLE_6;
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

  if (vehicle_type == LEXUS_RX_450H)
    PublishControl::max_rot_rad = MAX_ROT_RAD_VEHICLE2;
  else if (vehicle_type == FREIGHTLINER_CASCADIA)
    PublishControl::max_rot_rad = MAX_ROT_RAD_FREIGHTLINER_CASCADIA;
  else if (vehicle_type == JUPITER_SPIRIT)
    PublishControl::max_rot_rad = MAX_ROT_RAD_JUPITER_SPIRIT;
  else if (vehicle_type == VEHICLE_4)
    PublishControl::max_rot_rad = MAX_ROT_RAD_VEHICLE4;
  else if (vehicle_type == VEHICLE_5)
    PublishControl::max_rot_rad = MAX_ROT_RAD_VEHICLE5;
  else if (vehicle_type == VEHICLE_6)
    PublishControl::max_rot_rad = MAX_ROT_RAD_VEHICLE6;

  PublishControl::vehicle_type = vehicle_type;

  return exit;
}

bool AS::Joystick::check_controller_type(ros::NodeHandle * nodeH)
{
  std::string controller_string;
  bool exit = false;

  if (nodeH->getParam("controller_type", controller_string))
  {
    ROS_INFO("Got controller_type: %s", controller_string.c_str());

    if (controller_string == "LOGITECH_F310" || controller_string == "XBOX_ONE")
    {
      PublishControl::controller = (controller_string == "LOGITECH_F310") ? LOGITECH_F310 : XBOX_ONE;

      PublishControl::axes[LEFT_STICK_LR] = 0;
      PublishControl::axes[LEFT_STICK_UD] = 1;
      PublishControl::axes[LEFT_TRIGGER_AXIS] = 2;
      PublishControl::axes[RIGHT_STICK_LR] = 3;
      PublishControl::axes[RIGHT_STICK_UD] = 4;
      PublishControl::axes[RIGHT_TRIGGER_AXIS] = 5;
      PublishControl::axes[DPAD_LR] = 6;
      PublishControl::axes[DPAD_UD] = 7;

      PublishControl::btns[BOTTOM_BTN] = 0;
      PublishControl::btns[RIGHT_BTN] = 1;
      PublishControl::btns[LEFT_BTN] = 2;
      PublishControl::btns[TOP_BTN] = 3;
      PublishControl::btns[LEFT_BUMPER] = 4;
      PublishControl::btns[RIGHT_BUMPER] = 5;
      PublishControl::btns[BACK_SELECT_MINUS] = 6;
      PublishControl::btns[START_PLUS] = 7;
      PublishControl::btns[LEFT_STICK_PUSH] = 9;
      PublishControl::btns[RIGHT_STICK_PUSH] = 10;
    }
    else if (controller_string == "HRI_SAFE_REMOTE")
    {
      PublishControl::controller = HRI_SAFE_REMOTE;

      // TODO(jwhitleyastuff): Complete missing buttons
      PublishControl::axes[LEFT_STICK_LR] = 0;
      PublishControl::axes[RIGHT_STICK_LR] = 3;
      PublishControl::axes[RIGHT_STICK_UD] = 4;
      PublishControl::axes[DPAD_LR] = 6;
      PublishControl::axes[DPAD_UD] = 7;

      PublishControl::btns[BOTTOM_BTN] = 0;
      PublishControl::btns[RIGHT_BTN] = 1;
      PublishControl::btns[TOP_BTN] = 2;
      PublishControl::btns[LEFT_BTN] = 3;
    }
    else if (controller_string == "LOGITECH_G29")
    {
      PublishControl::controller = LOGITECH_G29;

      // Set to match the G29 controller's max center-to-lock steering angle (radians).
      PublishControl::max_rot_rad = 7.85;
      // steering wheel, not right stick
      PublishControl::axes[RIGHT_STICK_LR] = 0;
      // throttle pedal, not right trigger
      PublishControl::axes[RIGHT_TRIGGER_AXIS] = 2;
      // brake pedal, not left trigger
      PublishControl::axes[LEFT_TRIGGER_AXIS] = 3;
      PublishControl::axes[DPAD_LR] = 4;
      PublishControl::axes[DPAD_UD] = 5;

      PublishControl::btns[BOTTOM_BTN] = 0;
      PublishControl::btns[RIGHT_BTN] = 2;
      PublishControl::btns[LEFT_BTN] = 1;
      PublishControl::btns[TOP_BTN] = 3;

      // Following two are two blue buttons on the left
      PublishControl::btns[LEFT_BUMPER] = 7;
      PublishControl::btns[BACK_SELECT_MINUS] = 11;
      // Following two are two blue buttons on the right
      PublishControl::btns[RIGHT_BUMPER] = 6;
      PublishControl::btns[START_PLUS] = 10;
    }
    else if (controller_string == "NINTENDO_SWITCH_WIRED_PLUS")
    {
      PublishControl::controller = NINTENDO_SWITCH_WIRED_PLUS;

      PublishControl::axes[LEFT_STICK_LR] = 0;
      PublishControl::axes[LEFT_STICK_UD] = 1;
      PublishControl::axes[RIGHT_STICK_LR] = 2;
      PublishControl::axes[RIGHT_STICK_UD] = 3;
      PublishControl::axes[DPAD_LR] = 4;
      PublishControl::axes[DPAD_UD] = 5;

      PublishControl::btns[LEFT_BTN] = 0;
      PublishControl::btns[BOTTOM_BTN] = 1;
      PublishControl::btns[RIGHT_BTN] = 2;
      PublishControl::btns[TOP_BTN] = 3;
      PublishControl::btns[LEFT_BUMPER] = 4;
      PublishControl::btns[RIGHT_BUMPER] = 5;
      PublishControl::btns[LEFT_TRIGGER_BTN] = 6;
      PublishControl::btns[RIGHT_TRIGGER_BTN] = 7;
      PublishControl::btns[BACK_SELECT_MINUS] = 8;
      PublishControl::btns[START_PLUS] = 9;
      PublishControl::btns[LEFT_STICK_PUSH] = 10;
      PublishControl::btns[RIGHT_STICK_PUSH] = 11;
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

bool AS::Joystick::check_scale_values(ros::NodeHandle * nodeH)
{
  bool exit = false;

  if (nodeH->getParam("steering_max_speed", PublishControl::steering_max_speed))
  {
    ROS_INFO("Got steering_max_speed: %f", PublishControl::steering_max_speed);

    if (PublishControl::steering_max_speed <= 0)
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

  if (nodeH->getParam("max_veh_speed", PublishControl::max_veh_speed))
  {
    ROS_INFO("Got max_veh_speed: %f", PublishControl::max_veh_speed);

    if (PublishControl::max_veh_speed <= 0)
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

  if (nodeH->getParam("accel_scale_val", PublishControl::accel_scale_val))
  {
    ROS_INFO("Got accel_scale_val: %f", PublishControl::accel_scale_val);

    if (PublishControl::accel_scale_val <= 0 ||
        PublishControl::accel_scale_val > 1.0)
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

  if (nodeH->getParam("brake_scale_val", PublishControl::brake_scale_val))
  {
    ROS_INFO("Got brake_scale_val: %f", PublishControl::brake_scale_val);

    if (PublishControl::brake_scale_val <= 0 ||
        PublishControl::brake_scale_val > 1.0)
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

bool AS::Joystick::run_startup_checks_error(ros::NodeHandle * nodeH)
{
  bool willExit = false;

  // Run startup checks
  willExit = check_steering_stick_left_right(nodeH);
  willExit = check_vehicle_type(nodeH);
  willExit = check_controller_type(nodeH);
  willExit = check_scale_values(nodeH);

  return willExit;
}
