/*
* Unpublished Copyright (c) 2009-2018 AutonomouStuff, LLC, All Rights Reserved.
*
* This file is part of the PACMod ROS 1.0 driver which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#ifndef PACMOD_GAME_CONTROL_PUBLISH_CONTROL_BOARD_REV3_H
#define PACMOD_GAME_CONTROL_PUBLISH_CONTROL_BOARD_REV3_H

#include "pacmod_game_control/globals.h"
#include "pacmod_game_control/publish_control.h"

#include <pacmod_msgs/SteerSystemCmd.h>
#include <pacmod_msgs/SystemCmdBool.h>
#include <pacmod_msgs/SystemCmdFloat.h>
#include <pacmod_msgs/SystemCmdInt.h>
#include <pacmod_msgs/SystemRptInt.h>

namespace AS
{
namespace Joystick
{

class PublishControlBoardRev3 :
  public PublishControl
{
  public:
    PublishControlBoardRev3();
    static void callback_shift_rpt(const pacmod_msgs::SystemRptInt::ConstPtr& msg);
    static void callback_turn_rpt(const pacmod_msgs::SystemRptInt::ConstPtr& msg);
    static void callback_rear_pass_door_rpt(const pacmod_msgs::SystemRptInt::ConstPtr& msg);

    // Variables
    static int last_shift_cmd;
    static int last_turn_cmd;
    static int last_rear_pass_door_cmd;
    static float last_brake_cmd;

  private:
    // private functions
    void publish_steering_message(const sensor_msgs::Joy::ConstPtr& msg);
    void publish_turn_signal_message(const sensor_msgs::Joy::ConstPtr& msg);
    void publish_rear_pass_door_message(const sensor_msgs::Joy::ConstPtr& msg);
    void publish_shifting_message(const sensor_msgs::Joy::ConstPtr& msg);
    void publish_accelerator_message(const sensor_msgs::Joy::ConstPtr& msg);
    void publish_brake_message(const sensor_msgs::Joy::ConstPtr& msg);
    void publish_lights_horn_wipers_message(const sensor_msgs::Joy::ConstPtr& msg);

    // ROS Subscribers
    ros::Subscriber shift_sub;
    ros::Subscriber turn_sub;
    ros::Subscriber rear_pass_door_sub;
};

}  // namespace Joystick
}  // namespace AS

#endif  // PACMOD_GAME_CONTROL_PUBLISH_CONTROL_BOARD_REV3_H
