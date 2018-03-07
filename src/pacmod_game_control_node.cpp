/*
* Unpublished Copyright (c) 2009-2018 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the PACMod ROS 1.0 driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#include "publish_control_board_rev2.h"
#include "globals.h"
#include "startup_checks.h"

using namespace AS::Joystick;

/*
 * Main method running the ROS Node
 */
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pacmod_gamepad_control");
  ros::AsyncSpinner spinner(2);
  ros::NodeHandle priv("~");

  // Wait for time to be valid
  while (ros::Time::now().nsec == 0);
  
  if(run_startup_checks_error(&priv) == true)
    return 0;

  PublishControlBoardRev2 publish_control_board_rev2;

  // TODO : add code for multiple board revs

  spinner.start();
  ros::waitForShutdown();

  return 0;
}

