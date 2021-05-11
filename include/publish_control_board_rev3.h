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
#include <pacmod3/SystemRptFloat.h>
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
    static void callback_accel_rpt(const pacmod3::SystemRptFloat::ConstPtr& msg);
    static void callback_brake_rpt(const pacmod3::SystemRptFloat::ConstPtr& msg);
    static void callback_steer_rpt(const pacmod3::SystemRptFloat::ConstPtr& msg);
    static void callback_shift_rpt(const pacmod3::SystemRptInt::ConstPtr& msg);
    static void callback_turn_rpt(const pacmod3::SystemRptInt::ConstPtr& msg);
    static void callback_global_rpt2(const pacmod3::SystemRptInt::ConstPtr& msg);

    // Variables
    static int last_shift_cmd;
    static int last_turn_cmd;
    static float last_brake_cmd;
    static bool disable_all_systems;
    static bool accel_enabled;
    static bool accel_override_active;
    static bool brake_enabled;
    static bool brake_override_active;
    static bool steer_enabled;
    static bool steer_override_active;
    static bool shift_enabled;
    static bool shift_override_active;
    static bool turn_enabled;
    static bool turn_override_active;

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
    void publish_disable_on_all_systems(bool disable_all);

    // ROS Subscribers
    ros::Subscriber accel_sub;
    ros::Subscriber brake_sub;
    ros::Subscriber steer_sub;
    ros::Subscriber shift_sub;
    ros::Subscriber turn_sub;
};

}
}

#endif
