/*
* Unpublished Copyright (c) 2009-2018 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the PACMod ROS 1.0 driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#include "publish_control_factory.h"

using namespace AS::Joystick;

PublishControlFactory::PublishControlFactory() { }

PublishControl* PublishControlFactory::create(bool is_pacmod_3) {
  if (is_pacmod_3)
    return new PublishControlBoardRev3;
  else
    return new PublishControlBoardRev2;
}
