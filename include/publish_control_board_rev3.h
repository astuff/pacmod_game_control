/*
* Unpublished Copyright (c) 2009-2018 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the PACMod ROS 1.0 driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/
 
#ifndef PUBLISH_CONTROL_BOARD_REV3_H
#define PUBLISH_CONTROL_BOARD_REV3_H

#include "globals.h"

namespace AS
{
namespace Joystick
{

/*
 * Called when a game controller message is received
 */
void publish_control_board_rev3(const sensor_msgs::Joy::ConstPtr& msg);

}
}

#endif
