/*
* Unpublished Copyright (c) 2009-2018 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the PACMod ROS 1.0 driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#ifndef PACMOD_GAME_CONTROL_PUBLISH_CONTROL_BOARD_REV2_H
#define PACMOD_GAME_CONTROL_PUBLISH_CONTROL_BOARD_REV2_H

#include "pacmod_game_control/globals.h"
#include "pacmod_game_control/publish_control.h"

#include <pacmod_msgs/PositionWithSpeed.h>
#include <pacmod_msgs/PacmodCmd.h>

namespace AS
{
namespace Joystick
{

// Publish control class, child class from publish control
class PublishControlBoardRev2 :
  public PublishControl
{
  public:
    PublishControlBoardRev2();

  private:
    // private functions
    void publish_steering_message(const sensor_msgs::Joy::ConstPtr& msg);
    void publish_turn_signal_message(const sensor_msgs::Joy::ConstPtr& msg);
    void publish_rear_pass_door_message(const sensor_msgs::Joy::ConstPtr& msg);
    void publish_shifting_message(const sensor_msgs::Joy::ConstPtr& msg);
    void publish_accelerator_message(const sensor_msgs::Joy::ConstPtr& msg);
    void publish_brake_message(const sensor_msgs::Joy::ConstPtr& msg);
    void publish_lights_horn_wipers_message(const sensor_msgs::Joy::ConstPtr& msg);
};

}  // namespace Joystick
}  // namespace AS

#endif  // PACMOD_GAME_CONTROL_PUBLISH_CONTROL_BOARD_REV2_H
