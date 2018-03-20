/*
* Unpublished Copyright (c) 2009-2018 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the PACMod ROS 1.0 driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#include "publish_control_factory.h"

using namespace AS::Joystick;

PublishControlFactory::PublishControlFactory()
{}

std::unique_ptr<PublishControl> PublishControlFactory::create(int board_rev)
{
  if (board_rev == 1 || board_rev == 2)
    return std::unique_ptr<PublishControlBoardRev2>(new PublishControlBoardRev2);
  else if (board_rev == 3)
    return std::unique_ptr<PublishControlBoardRev3>(new PublishControlBoardRev3);
  else
    throw std::invalid_argument("Invalid option.");
}
