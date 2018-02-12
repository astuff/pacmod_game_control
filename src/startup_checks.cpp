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

 
bool check_steering_stick_left_right(ros::NodeHandle * nodeH)
{
  std::string steering_stick_string;
  bool exit = false;

  if (nodeH->getParam("steering_stick", steering_stick_string))
  {
    ROS_INFO("Got steering_stick: %s", steering_stick_string.c_str());

    if (steering_stick_string == "LEFT")
    {
      publish_control::steering_axis = LEFT_STICK_LR;
    }
    else if (steering_stick_string == "RIGHT")
    {
      publish_control::steering_axis = RIGHT_STICK_LR;
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
    if (vehicle_type != 0 &&
        vehicle_type != 1 &&
        vehicle_type != 2 &&
        vehicle_type != 3 &&
        vehicle_type != 4)
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
	publish_control::max_rot_rad = 6.5;

  if (vehicle_type == 4)
	publish_control::max_rot_rad = 5.236;
    
  publish_control::vehicle_type = vehicle_type;
    
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
      publish_control::controller = LOGITECH_F310;

      publish_control::axes[LEFT_STICK_LR] = 0;
      publish_control::axes[LEFT_STICK_UD] = 1;
      publish_control::axes[LEFT_TRIGGER_AXIS] = 2;
      publish_control::axes[RIGHT_STICK_LR] = 3;
      publish_control::axes[RIGHT_STICK_UD] = 4;
      publish_control::axes[RIGHT_TRIGGER_AXIS] = 5;
      publish_control::axes[DPAD_LR] = 6;
      publish_control::axes[DPAD_UD] = 7;

      publish_control::btns[BOTTOM_BTN] = 0;
      publish_control::btns[RIGHT_BTN] = 1;
      publish_control::btns[LEFT_BTN] = 2;
      publish_control::btns[TOP_BTN] = 3;
      publish_control::btns[LEFT_BUMPER] = 4;
      publish_control::btns[RIGHT_BUMPER] = 5;
      publish_control::btns[BACK_SELECT_MINUS] = 6;
      publish_control::btns[START_PLUS] = 7;
      publish_control::btns[LEFT_STICK_PUSH] = 9;
      publish_control::btns[RIGHT_STICK_PUSH] = 10;
    }
    else if (controller_string == "HRI_SAFE_REMOTE")
    {
      publish_control::controller = HRI_SAFE_REMOTE;

      // TODO: Complete missing buttons
      publish_control::axes[LEFT_STICK_LR] = 0;
      publish_control::axes[RIGHT_STICK_LR] = 3;
      publish_control::axes[RIGHT_STICK_UD] = 4;
      publish_control::axes[DPAD_LR] = 6;
      publish_control::axes[DPAD_UD] = 7;

      publish_control::btns[BOTTOM_BTN] = 0;
      publish_control::btns[RIGHT_BTN] = 1;
      publish_control::btns[TOP_BTN] = 2;
      publish_control::btns[LEFT_BTN] = 3;
    }
    else if (controller_string == "LOGITECH_G29")
    {
    	publish_control::controller = LOGITECH_G29;
      // TODO: Complete missing buttons
    }
    else if (controller_string == "NINTENDO_SWITCH_WIRED_PLUS")
    {
      publish_control::controller = NINTENDO_SWITCH_WIRED_PLUS;

      publish_control::axes[LEFT_STICK_LR] = 0;
      publish_control::axes[LEFT_STICK_UD] = 1;
      publish_control::axes[RIGHT_STICK_LR] = 2;
      publish_control::axes[RIGHT_STICK_UD] = 3;
      publish_control::axes[DPAD_LR] = 4;
      publish_control::axes[DPAD_UD] = 5;

      publish_control::btns[LEFT_BTN] = 0;
      publish_control::btns[BOTTOM_BTN] = 1;
      publish_control::btns[RIGHT_BTN] = 2;
      publish_control::btns[TOP_BTN] = 3;
      publish_control::btns[LEFT_BUMPER] = 4;
      publish_control::btns[RIGHT_BUMPER] = 5;
      publish_control::btns[LEFT_TRIGGER_BTN] = 6;
      publish_control::btns[RIGHT_TRIGGER_BTN] = 7;
      publish_control::btns[BACK_SELECT_MINUS] = 8;
      publish_control::btns[START_PLUS] = 9;
      publish_control::btns[LEFT_STICK_PUSH] = 10;
      publish_control::btns[RIGHT_STICK_PUSH] = 11;
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
  
  if (nodeH->getParam("steering_max_speed", publish_control::steering_max_speed))
  {
    ROS_INFO("Got steering_max_speed: %f", publish_control::steering_max_speed);

    if (publish_control::steering_max_speed <= 0)
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

  if (nodeH->getParam("max_veh_speed", publish_control::max_veh_speed))
  {
    ROS_INFO("Got max_veh_speed: %f", publish_control::max_veh_speed);

    if (publish_control::max_veh_speed <= 0)
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

  if (nodeH->getParam("accel_scale_val", publish_control::accel_scale_val))
  {
    ROS_INFO("Got accel_scale_val: %f", publish_control::accel_scale_val);

    if (publish_control::accel_scale_val <= 0 ||
        publish_control::accel_scale_val > 1.0)
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

  if (nodeH->getParam("brake_scale_val", publish_control::brake_scale_val))
  {
    ROS_INFO("Got brake_scale_val: %f", publish_control::brake_scale_val);

    if (publish_control::brake_scale_val <= 0 ||
        publish_control::brake_scale_val > 1.0)
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
