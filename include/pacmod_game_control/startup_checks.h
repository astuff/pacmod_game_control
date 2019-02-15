/*
* Unpublished Copyright (c) 2009-2018 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the PACMod ROS 1.0 driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/
/*
 * Series of startup checks for the pacmod game control node.
 */

#ifndef PACMOD_GAME_CONTROL_STARTUP_CHECKS_H
#define PACMOD_GAME_CONTROL_STARTUP_CHECKS_H

#include "pacmod_game_control/globals.h"
#include "pacmod_game_control/publish_control.h"

namespace AS
{
namespace Joystick
{

bool check_steering_stick_left_right(ros::NodeHandle*);
bool check_vehicle_type(ros::NodeHandle * nodeH);
bool check_controller_type(ros::NodeHandle * nodeH);
bool check_scale_values(ros::NodeHandle * nodeH);
bool run_startup_checks_error(ros::NodeHandle * nodeH);

}  // namespace Joystick
}  // namespace AS

#endif  // PACMOD_GAME_CONTROL_STARTUP_CHECKS_H
