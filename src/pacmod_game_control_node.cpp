/*
* Unpublished Copyright (c) 2009-2018 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the PACMod ROS 1.0 driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

// #include "pacmod_game_control/publish_control_board_rev3.h"
#include "pacmod_game_control/publish_control.h"
#include "pacmod_game_control/globals.h"
#include "pacmod_game_control/startup_checks.h"

#include <memory>

using namespace AS::Joystick;  // NOLINT

/*
 * Main method running the ROS Node
 */
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pacmod_gamepad_control");
  ros::AsyncSpinner spinner(2);
  ros::NodeHandle priv("~");

  // Wait for time to be valid
  ros::Time::waitForValid();

  if (run_startup_checks_error(&priv) == true)
    return 0;

  PublishControl pub_control;

  spinner.start();
  ros::waitForShutdown();

  return 0;
}
