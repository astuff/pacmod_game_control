/*
* Unpublished Copyright (c) 2009-2018 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the PACMod ROS 1.0 driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/
/*
 * Series of startup checks for the pacmod game control node.
 */

#include "startup_checks.h"

// Check which steering stick we should use on the joypad
bool check_steering_stick_left_right(ros::NodeHandle * nodeH)
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

bool check_vehicle_type(ros::NodeHandle * nodeH)
{
  bool exit = false;
  int vehicle_type = -1;
  
  // Vehicle type 0 is Polaris GEM, type 1 is Polaris Ranger, type 3 is semi
  if (nodeH->getParam("vehicle_type", vehicle_type))
  {
    ROS_INFO("Got vehicle_type: %d", vehicle_type);

    // TODO : macro for vehicle type
    if (vehicle_type != POLARIS_GEM &&
        vehicle_type != POLARIS_RANGER &&
        vehicle_type != VEHICLE_2 &&
        vehicle_type != INTERNATIONAL_PROSTAR &&
        vehicle_type != VEHICLE_4)
    {
      ROS_ERROR("vehicle_type is invalid");
      exit = true;
    }
  }
  else
  {
    ROS_ERROR("Parameter vehicle_type is missing. Exiting.");
    exit = true;
  }

  // TODO : fix this 
  if (vehicle_type == 2)
    PublishControl::max_rot_rad = MAX_ROT_RAD_VEHICLE2;

  if (vehicle_type == 4)
    PublishControl::max_rot_rad = MAX_ROT_RAD_VEHICLE4;
    
  PublishControl::vehicle_type = vehicle_type;
    
  return exit;
}

bool check_controller_type(ros::NodeHandle * nodeH)
{
  std::string controller_string;
  bool exit = false;

  if (nodeH->getParam("controller_type", controller_string))
  {
    ROS_INFO("Got controller_type: %s", controller_string.c_str());

    if (controller_string == "LOGITECH_F310")
    {
      PublishControl::controller = LOGITECH_F310;

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

      // TODO: Complete missing buttons
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
      // TODO: Complete missing buttons
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

bool check_scale_values(ros::NodeHandle * nodeH)
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

bool run_startup_checks_error(ros::NodeHandle * nodeH)
{
  bool willExit = false;
  
  // Run startup checks
  willExit = check_steering_stick_left_right(nodeH);
  willExit = check_vehicle_type(nodeH);
  willExit = check_controller_type(nodeH);
  willExit = check_scale_values(nodeH);
  
  return willExit;
}
