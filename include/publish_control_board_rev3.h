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

#include <pacmod3/SteerSystemCmd.h>
#include <pacmod3/SystemCmdBool.h>
#include <pacmod3/SystemCmdFloat.h>
#include <pacmod3/SystemCmdInt.h>
#include <pacmod3/SystemRptInt.h>
#include <pacmod3/GlobalRpt2.h>
#include <pacmod3/GlobalCmd.h>

namespace AS
{
namespace Joystick
{

class PublishControlBoardRev3 :
  public PublishControl
{
  public:
    PublishControlBoardRev3();
    static void callback_shift_rpt(const pacmod3::SystemRptInt::ConstPtr& msg);
    static void callback_turn_rpt(const pacmod3::SystemRptInt::ConstPtr& msg);

    // Variables
    static int last_shift_cmd;
    static int last_turn_cmd;
    static float last_brake_cmd;
    
  private:
    // private functions
    void publish_steering_message(const sensor_msgs::Joy::ConstPtr& msg);
    void publish_turn_signal_message(const sensor_msgs::Joy::ConstPtr& msg);
    void publish_shifting_message(const sensor_msgs::Joy::ConstPtr& msg);
    void publish_accelerator_message(const sensor_msgs::Joy::ConstPtr& msg);
    void publish_brake_message(const sensor_msgs::Joy::ConstPtr& msg);
    void publish_lights_horn_wipers_message(const sensor_msgs::Joy::ConstPtr& msg);
    void publish_global_message(const sensor_msgs::Joy::ConstPtr& msg);
    void publish_hazard_message(const sensor_msgs::Joy::ConstPtr& msg);

    // ROS Subscribers
    ros::Subscriber shift_sub;
    ros::Subscriber turn_sub;
};

}
}

#endif
