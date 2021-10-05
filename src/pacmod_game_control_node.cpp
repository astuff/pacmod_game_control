/*
 * Unpublished Copyright (c) 2009-2021 AutonomouStuff, LLC, All Rights Reserved.
 *
 * This file is part of the PACMod ROS 1.0 driver which is released under the MIT license.
 * See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
 */

#include "pacmod_game_control/pacmod_game_control.h"

#include <memory>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "pacmod_game_control");

  GameControl pub_control;
  pub_control.init();

  ros::spin();

  return 0;
}
