/*
* Unpublished Copyright (c) 2009-2018 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the PACMod ROS 1.0 driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/
 
#ifndef PUBLISH_CONTROL_BOARD_REV3_H
#define PUBLISH_CONTROL_BOARD_REV3_H

#include "globals.h"
#include "publish_control.h"

#include <pacmod_msgs/SteerSystemCmd.h>
#include <pacmod_msgs/SystemCmdBool.h>
#include <pacmod_msgs/SystemCmdFloat.h>
#include <pacmod_msgs/SystemCmdInt.h>

namespace AS
{
namespace Joystick
{

class PublishControlBoardRev3 :
  public PublishControl
{
public:

    PublishControlBoardRev3();

private:
  
  // private functions
  void publish_steering_message(const sensor_msgs::Joy::ConstPtr& msg);
  void publish_turn_signal_message(const sensor_msgs::Joy::ConstPtr& msg);
  void publish_shifting_message(const sensor_msgs::Joy::ConstPtr& msg);
  void publish_accelerator_message(const sensor_msgs::Joy::ConstPtr& msg);
  void publish_brake_message(const sensor_msgs::Joy::ConstPtr& msg);
  void publish_lights_horn_wipers_message(const sensor_msgs::Joy::ConstPtr& msg);

};

}
}

#endif
